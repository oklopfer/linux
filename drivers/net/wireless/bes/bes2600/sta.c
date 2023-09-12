/*
 * Mac80211 STA API for BES2600 drivers
 *
 * Copyright (c) 2022, Bestechnic
 * Author:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/vmalloc.h>
#include <linux/sched.h>
#include <linux/firmware.h>
#include <linux/if_arp.h>
#include <linux/ipv6.h>
#include <linux/icmpv6.h>
#include <net/ndisc.h>

#include "bes2600.h"
#include "sta.h"
#include "ap.h"
#include "fwio.h"
#include "bh.h"
#include "debug.h"
#include "wsm.h"
#include "net/mac80211.h"
#include "bes2600_driver_mode.h"
#include "bes_chardev.h"

#include "epta_request.h"
#include "epta_coex.h"

#include "txrx_opt.h"

#define WEP_ENCRYPT_HDR_SIZE    4
#define WEP_ENCRYPT_TAIL_SIZE   4
#define WPA_ENCRYPT_HDR_SIZE    8
#define WPA_ENCRYPT_TAIL_SIZE   12
#define WPA2_ENCRYPT_HDR_SIZE   8
#define WPA2_ENCRYPT_TAIL_SIZE  8
#define WAPI_ENCRYPT_HDR_SIZE   18
#define WAPI_ENCRYPT_TAIL_SIZE  16
#define MAX_ARP_REPLY_TEMPLATE_SIZE     120
#define MAX_TCP_ALIVE_TEMPLATE_SIZE     256

static inline void __bes2600_free_event_queue(struct list_head *list)
{
	while (!list_empty(list)) {
		struct bes2600_wsm_event *event =
			list_first_entry(list, struct bes2600_wsm_event,
			link);
		list_del(&event->link);
		kfree(event);
	}
}

static inline void __bes2600_bf_configure(struct bes2600_vif *priv)
{
	priv->bf_table.numOfIEs = __cpu_to_le32(3);
	priv->bf_table.entry[0].ieId = WLAN_EID_VENDOR_SPECIFIC;
	priv->bf_table.entry[0].actionFlags = WSM_BEACON_FILTER_IE_HAS_CHANGED |
					WSM_BEACON_FILTER_IE_NO_LONGER_PRESENT |
					WSM_BEACON_FILTER_IE_HAS_APPEARED;
	priv->bf_table.entry[0].oui[0] = 0x50;
	priv->bf_table.entry[0].oui[1] = 0x6F;
	priv->bf_table.entry[0].oui[2] = 0x9A;

	priv->bf_table.entry[1].ieId = WLAN_EID_ERP_INFO;
	priv->bf_table.entry[1].actionFlags = WSM_BEACON_FILTER_IE_HAS_CHANGED |
					WSM_BEACON_FILTER_IE_NO_LONGER_PRESENT |
					WSM_BEACON_FILTER_IE_HAS_APPEARED;

	priv->bf_table.entry[2].ieId = WLAN_EID_HT_OPERATION;
	priv->bf_table.entry[2].actionFlags = WSM_BEACON_FILTER_IE_HAS_CHANGED |
					WSM_BEACON_FILTER_IE_NO_LONGER_PRESENT |
					WSM_BEACON_FILTER_IE_HAS_APPEARED;

	priv->bf_control.enabled = WSM_BEACON_FILTER_ENABLE;
}

/* ******************************************************************** */
/* STA API								*/

int bes2600_start(struct ieee80211_hw *dev)
{
	struct bes2600_common *hw_priv = dev->priv;
	int ret = 0;

	bes2600_info(BES2600_DBG_STA, "bes2600_start\n");

	if (!bes2600_chrdev_is_signal_mode()) {
		return -1;
	}

	bes2600_pwr_prepare(hw_priv);

	/* Assign Max SSIDs supported based on the firmware revision*/
	if (hw_priv->hw_revision <= BES2600_HW_REV_CUT22) {
		dev->wiphy->max_scan_ssids = 2; /* for backward compatibility */
	}
	down(&hw_priv->conf_lock);

	ret = bes2600_wifi_start(hw_priv);
	if (ret)
		goto out;
	tx_policy_init(hw_priv);

	memcpy(hw_priv->mac_addr, dev->wiphy->perm_addr, ETH_ALEN);
	hw_priv->softled_state = 0;

	atomic_inc(&hw_priv->netdevice_start);
	coex_start(hw_priv);

	bes2600_info(BES2600_DBG_STA, "%s %pM.\n", __func__, hw_priv->mac_addr);
	ret = bes2600_setup_mac(hw_priv);
	if (WARN_ON(ret))
		goto out;

	bwifi_change_current_status(hw_priv, BWIFI_STATUS_IDLE);

out:
	up(&hw_priv->conf_lock);
	if(ret != 0)
		bes2600_pwr_complete(hw_priv);
	return ret;
}

void bes2600_stop(struct ieee80211_hw *dev)
{
	struct bes2600_common *hw_priv = dev->priv;
	struct bes2600_vif *priv = NULL;
	LIST_HEAD(list);
	int i;

	bes2600_info(BES2600_DBG_STA, "bes2600_stop\n");

	atomic_dec(&hw_priv->netdevice_start);

	wsm_lock_tx(hw_priv);

	while (down_trylock(&hw_priv->scan.lock)) {
		/* Scan is in progress. Force it to stop. */
		hw_priv->scan.req = NULL;
		schedule();
	}
	up(&hw_priv->scan.lock);

	cancel_delayed_work_sync(&hw_priv->scan.probe_work);
	cancel_delayed_work_sync(&hw_priv->scan.timeout);

	flush_workqueue(hw_priv->workqueue);
	del_timer_sync(&hw_priv->ba_timer);

	down(&hw_priv->conf_lock);

	hw_priv->softled_state = 0;
	/* bes2600_set_leds(hw_priv); */

	spin_lock(&hw_priv->event_queue_lock);
	list_splice_init(&hw_priv->event_queue, &list);
	spin_unlock(&hw_priv->event_queue_lock);
	__bes2600_free_event_queue(&list);

	for (i = 0; i < 4; i++)
		bes2600_queue_clear(&hw_priv->tx_queue[i], CW12XX_ALL_IFS);

	/* HACK! */
	if (atomic_xchg(&hw_priv->tx_lock, 1) != 1)
		bes2600_dbg(BES2600_DBG_STA, "[STA] TX is force-unlocked "
			"due to stop request.\n");

	bes2600_for_each_vif(hw_priv, priv, i) {
		if (!priv)
			continue;
		priv->mode = NL80211_IFTYPE_UNSPECIFIED;
		priv->listening = false;
		priv->delayed_link_loss = 0;
		priv->join_status = BES2600_JOIN_STATUS_PASSIVE;
		cancel_delayed_work_sync(&priv->join_timeout);
		cancel_delayed_work_sync(&priv->bss_loss_work);
		cancel_delayed_work_sync(&priv->connection_loss_work);
		cancel_delayed_work_sync(&priv->link_id_gc_work);
		del_timer_sync(&priv->mcast_timeout);
	}

	cancel_work_sync(&hw_priv->coex_work);
	coex_stop(hw_priv);

	bes2600_wifi_stop(hw_priv);

	tx_policy_deinit(hw_priv);

	wsm_unlock_tx(hw_priv);

	up(&hw_priv->conf_lock);
	bes2600_pwr_complete(hw_priv);
}


int bes2600_add_interface(struct ieee80211_hw *dev,
			 struct ieee80211_vif *vif)
{
	int ret;
	struct bes2600_common *hw_priv = dev->priv;
	struct bes2600_vif *priv;
	struct bes2600_vif **drv_priv = (void *)vif->drv_priv;
	bes2600_err(BES2600_DBG_STA, " !!! %s: type %d p2p %d addr %pM\n",
		__func__, vif->type, vif->p2p, vif->addr);

	priv = cw12xx_get_vif_from_ieee80211(vif);
	atomic_set(&priv->enabled, 0);

	*drv_priv = priv;
	/* __le32 auto_calibration_mode = __cpu_to_le32(1); */

	down(&hw_priv->conf_lock);

	priv->mode = vif->type;

	spin_lock(&hw_priv->vif_list_lock);
	if (atomic_read(&hw_priv->num_vifs) < CW12XX_MAX_VIFS) {
		if (!memcmp(vif->addr, hw_priv->addresses[0].addr, ETH_ALEN)) {
			priv->if_id = 0;
		} else if (!memcmp(vif->addr, hw_priv->addresses[1].addr,
			ETH_ALEN)) {
			priv->if_id = 2;
		} else if (!memcmp(vif->addr, hw_priv->addresses[2].addr,
			ETH_ALEN)) {
			priv->if_id = 1;
		}
		bes2600_info(BES2600_DBG_STA, "%s: if_id %d mac %pM\n",
				__func__, priv->if_id, vif->addr);
		priv->hw_priv = hw_priv;
		priv->hw = dev;
		priv->vif = vif;
	} else {
		spin_unlock(&hw_priv->vif_list_lock);
		up(&hw_priv->conf_lock);
		return -EOPNOTSUPP;
	}
	spin_unlock(&hw_priv->vif_list_lock);
	/* TODO:COMBO :Check if MAC address matches the one expected by FW */
	memcpy(hw_priv->mac_addr, vif->addr, ETH_ALEN);

	/* Enable auto-calibration */
	/* Exception in subsequent channel switch; disabled.
	WARN_ON(wsm_write_mib(hw_priv, WSM_MIB_ID_SET_AUTO_CALIBRATION_MODE,
		&auto_calibration_mode, sizeof(auto_calibration_mode)));
	*/
	bes2600_info(BES2600_DBG_STA, "[STA] Interface ID:%d of type:%d added\n",
		   priv->if_id, priv->mode);

	up(&hw_priv->conf_lock);

	bes2600_vif_setup(priv);

	ret = WARN_ON(bes2600_setup_mac_pvif(priv));

	return ret;
}

void bes2600_remove_interface(struct ieee80211_hw *dev,
			     struct ieee80211_vif *vif)
{
	struct bes2600_common *hw_priv = dev->priv;
	struct bes2600_vif *priv = cw12xx_get_vif_from_ieee80211(vif);
	struct wsm_reset reset = {
		.reset_statistics = true,
	};
	int i;
	bool is_htcapie = false;
	struct bes2600_vif *tmp_priv;

	bes2600_info(BES2600_DBG_STA, " !!! %s: type %d p2p %d addr %pM\n",
		__func__, vif->type, vif->p2p, vif->addr);
	atomic_set(&priv->enabled, 0);
	down(&hw_priv->scan.lock);
	down(&hw_priv->conf_lock);
	bes2600_tx_queues_lock(hw_priv);
	wsm_lock_tx(hw_priv);
	switch (priv->join_status) {
	case BES2600_JOIN_STATUS_STA:
		wsm_lock_tx(hw_priv);
		if (queue_work(hw_priv->workqueue, &priv->unjoin_work) <= 0)
			wsm_unlock_tx(hw_priv);
		break;
	case BES2600_JOIN_STATUS_AP:
		for (i = 0; priv->link_id_map; ++i) {
			if (priv->link_id_map & BIT(i)) {
				cw12xx_unmap_link(priv, i);
				priv->link_id_map &= ~BIT(i);
			}
		}
		memset(priv->link_id_db, 0,
				sizeof(priv->link_id_db));
		priv->sta_asleep_mask = 0;
		priv->enable_beacon = false;
		priv->tx_multicast = false;
		priv->aid0_bit_set = false;
		priv->buffered_multicasts = false;
		priv->pspoll_mask = 0;
		reset.link_id = 0;
		wsm_reset(hw_priv, &reset, priv->if_id);
		bes2600_for_each_vif(hw_priv, tmp_priv, i) {
			if ((i == (CW12XX_MAX_VIFS - 1)) || !tmp_priv)
				continue;
			if ((tmp_priv->join_status == BES2600_JOIN_STATUS_STA) && tmp_priv->htcap)
				is_htcapie = true;
		}

		if (is_htcapie) {
			hw_priv->vif0_throttle = CW12XX_HOST_VIF0_11N_THROTTLE;
			hw_priv->vif1_throttle = CW12XX_HOST_VIF1_11N_THROTTLE;
			bes2600_info(BES2600_DBG_STA, "AP REMOVE HTCAP 11N %d\n",hw_priv->vif0_throttle);
		} else {
			hw_priv->vif0_throttle = CW12XX_HOST_VIF0_11BG_THROTTLE;
			hw_priv->vif1_throttle = CW12XX_HOST_VIF1_11BG_THROTTLE;
			bes2600_info(BES2600_DBG_STA, "AP REMOVE 11BG %d\n",hw_priv->vif0_throttle);
		}
		bes2600_pwr_clear_busy_event(hw_priv, BES2600_JOIN_STATUS_AP);
		break;
	case BES2600_JOIN_STATUS_MONITOR:
		bes2600_disable_listening(priv);
		break;
	default:
		break;
	}
	/* TODO:COMBO: Change Queue Module */
	if (!__bes2600_flush(hw_priv, false, priv->if_id))
		wsm_unlock_tx(hw_priv);

	cancel_delayed_work_sync(&priv->bss_loss_work);
	cancel_delayed_work_sync(&priv->connection_loss_work);
	cancel_delayed_work_sync(&priv->link_id_gc_work);
	cancel_delayed_work_sync(&priv->join_timeout);
	cancel_delayed_work_sync(&priv->set_cts_work);
	cancel_delayed_work_sync(&priv->pending_offchanneltx_work);

	del_timer_sync(&priv->mcast_timeout);
	/* TODO:COMBO: May be reset of these variables "delayed_link_loss and
	 * join_status to default can be removed as dev_priv will be freed by
	 * mac80211 */
	priv->delayed_link_loss = 0;
	priv->join_status = BES2600_JOIN_STATUS_PASSIVE;
	wsm_unlock_tx(hw_priv);

	if ((priv->if_id ==1) && (priv->mode == NL80211_IFTYPE_AP
		|| priv->mode == NL80211_IFTYPE_P2P_GO)) {
		hw_priv->is_go_thru_go_neg = false;
	}
	spin_lock(&hw_priv->vif_list_lock);
	spin_lock(&priv->vif_lock);
	hw_priv->vif_list[priv->if_id] = NULL;
	hw_priv->if_id_slot &= (~BIT(priv->if_id));
	atomic_dec(&hw_priv->num_vifs);
	if (atomic_read(&hw_priv->num_vifs) == 0) {
		bes2600_free_keys(hw_priv);
		memset(hw_priv->mac_addr, 0, ETH_ALEN);
	}
	spin_unlock(&priv->vif_lock);
	spin_unlock(&hw_priv->vif_list_lock);
	priv->listening = false;

	bes2600_debug_release_priv(priv);

	bes2600_tx_queues_unlock(hw_priv);
	up(&hw_priv->conf_lock);

	if (atomic_read(&hw_priv->num_vifs) == 0)
		flush_workqueue(hw_priv->workqueue);
	memset(priv, 0, sizeof(struct bes2600_vif));
	up(&hw_priv->scan.lock);
}

int bes2600_change_interface(struct ieee80211_hw *dev,
				struct ieee80211_vif *vif,
				enum nl80211_iftype new_type,
				bool p2p)
{
	int ret = 0;
	bes2600_info(BES2600_DBG_STA, " !!! %s: type %d (%d), p2p %d (%d)\n",
		__func__, new_type, vif->type, p2p, vif->p2p);
	if (new_type != vif->type || vif->p2p != p2p) {
		bes2600_remove_interface(dev, vif);
		vif->type = new_type;
		vif->p2p = p2p;
		ret = bes2600_add_interface(dev, vif);
	}

	return ret;
}

int bes2600_config(struct ieee80211_hw *dev, u32 changed)
{
	int ret = 0;
	struct bes2600_common *hw_priv = dev->priv;
	struct ieee80211_conf *conf = &dev->conf;
	/* TODO:COMBO: adjust to multi vif interface
	 * IEEE80211_CONF_CHANGE_IDLE is still handled per bes2600_vif*/
	int if_id = 0;
	struct bes2600_vif *priv;

	bes2600_info(BES2600_DBG_STA, "CONFIG CHANGED:	%08x\n", changed);

	if (changed &
		(IEEE80211_CONF_CHANGE_MONITOR|IEEE80211_CONF_CHANGE_IDLE)) {
		/* TBD: It looks like it's transparent
		 * there's a monitor interface present -- use this
		 * to determine for example whether to calculate
		 * timestamps for packets or not, do not use instead
		 * of filter flags! */
		bes2600_info(BES2600_DBG_STA,
				"ignore IEEE80211_CONF_CHANGE_MONITOR (%d)"
				"IEEE80211_CONF_CHANGE_IDLE (%d)\n",
			(changed & IEEE80211_CONF_CHANGE_MONITOR) ? 1 : 0,
			(changed & IEEE80211_CONF_CHANGE_IDLE) ? 1 : 0);
		return ret;
	}

	down(&hw_priv->scan.lock);
	down(&hw_priv->conf_lock);
	priv = __cw12xx_hwpriv_to_vifpriv(hw_priv, hw_priv->scan.if_id);
	/* TODO: IEEE80211_CONF_CHANGE_QOS */
	/* TODO:COMBO:Change when support is available mac80211*/
	if (changed & IEEE80211_CONF_CHANGE_POWER) {
		/*hw_priv->output_power = conf->power_level;*/
		bes2600_info(BES2600_DBG_STA, "Output power ++%d\n",conf->power_level);
		hw_priv->output_power = 20;
		bes2600_info(BES2600_DBG_STA, "Output power --%d\n",hw_priv->output_power);
		bes2600_info(BES2600_DBG_STA, "[STA] TX power: %d\n",
				hw_priv->output_power);
		WARN_ON(wsm_set_output_power(hw_priv,
					     hw_priv->output_power * 10,
					     if_id));
	}

	if (changed & IEEE80211_CONF_CHANGE_CHANNEL) {
		/* Switch Channel commented for CC Mode */
		struct ieee80211_channel *ch = conf->chandef.chan;

		bes2600_info(BES2600_DBG_STA, "[STA] Freq %d (wsm ch: %d, type: %d).\n",
			   ch->center_freq, ch->hw_value,
			   cfg80211_get_chandef_type(&conf->chandef));
		/* Earlier there was a call to __bes2600_flush().
		   Removed as deemed unnecessary */

		hw_priv->channel = ch;

	}

	if (changed & IEEE80211_CONF_CHANGE_RETRY_LIMITS) {
		bes2600_info(BES2600_DBG_STA, "[STA] Retry limits: %d (long), %d (short).\n",
			 conf->long_frame_max_tx_count,
			 conf->short_frame_max_tx_count);
		spin_lock_bh(&hw_priv->tx_policy_cache.lock);
		hw_priv->long_frame_max_tx_count = conf->long_frame_max_tx_count;
		hw_priv->short_frame_max_tx_count =
			(conf->short_frame_max_tx_count < 0x0F) ?
			conf->short_frame_max_tx_count : 0x0F;
		hw_priv->hw->max_rate_tries = hw_priv->short_frame_max_tx_count;
		spin_unlock_bh(&hw_priv->tx_policy_cache.lock);
	}
	up(&hw_priv->conf_lock);
	up(&hw_priv->scan.lock);

	return ret;
}

void bes2600_update_filtering(struct bes2600_vif *priv)
{
	int ret;
	struct bes2600_common *hw_priv = cw12xx_vifpriv_to_hwpriv(priv);
	bool bssid_filtering = !priv->rx_filter.bssid;
	static struct wsm_beacon_filter_control bf_disabled = {
		.enabled = 0,
		.bcn_count = 1,
	};
	bool ap_mode = 0;
	static struct wsm_beacon_filter_table bf_table_auto = {
		.numOfIEs = __cpu_to_le32(2),
		.entry[0].ieId = WLAN_EID_VENDOR_SPECIFIC,
		.entry[0].actionFlags = WSM_BEACON_FILTER_IE_HAS_CHANGED |
					WSM_BEACON_FILTER_IE_NO_LONGER_PRESENT |
					WSM_BEACON_FILTER_IE_HAS_APPEARED,
		.entry[0].oui[0] = 0x50,
		.entry[0].oui[1] = 0x6F,
		.entry[0].oui[2] = 0x9A,

		.entry[1].ieId = WLAN_EID_HT_OPERATION,
		.entry[1].actionFlags = WSM_BEACON_FILTER_IE_HAS_CHANGED |
					WSM_BEACON_FILTER_IE_NO_LONGER_PRESENT |
					WSM_BEACON_FILTER_IE_HAS_APPEARED,
	};
	static struct wsm_beacon_filter_control bf_auto = {
		.enabled = WSM_BEACON_FILTER_ENABLE |
			WSM_BEACON_FILTER_AUTO_ERP,
		.bcn_count = 1,
	};
	bf_auto.bcn_count = priv->bf_control.bcn_count;

	if (priv->join_status == BES2600_JOIN_STATUS_PASSIVE)
		return;
	else if (priv->join_status == BES2600_JOIN_STATUS_MONITOR)
		bssid_filtering = false;

	if (priv->vif && (priv->vif->type == NL80211_IFTYPE_AP))
		ap_mode = true;
	/*
	* When acting as p2p client being connected to p2p GO, in order to
	* receive frames from a different p2p device, turn off bssid filter.
	*
	* WARNING: FW dependency!
	* This can only be used with FW WSM371 and its successors.
	* In that FW version even with bssid filter turned off,
	* device will block most of the unwanted frames.
	*/
	if (priv->vif && priv->vif->p2p)
		bssid_filtering = false;

	ret = wsm_set_rx_filter(hw_priv, &priv->rx_filter, priv->if_id);
	if (!ret && !ap_mode) {
		if (priv->vif) {
			if (priv->vif->p2p || NL80211_IFTYPE_STATION != priv->vif->type)
				ret = wsm_set_beacon_filter_table(hw_priv, &priv->bf_table,
							priv->if_id);
			else
				ret = wsm_set_beacon_filter_table(hw_priv, &bf_table_auto,
							priv->if_id);
		} else
			WARN_ON(1);
	}
	if (!ret && !ap_mode) {
		if (priv->disable_beacon_filter)
			ret = wsm_beacon_filter_control(hw_priv,
					&bf_disabled, priv->if_id);
		else {
			if (priv->vif) {
				if (priv->vif->p2p || NL80211_IFTYPE_STATION != priv->vif->type)
					ret = wsm_beacon_filter_control(hw_priv,
						&priv->bf_control, priv->if_id);
				else
					ret = wsm_beacon_filter_control(hw_priv,
						&bf_auto, priv->if_id);
			} else
				WARN_ON(1);
		}
	}

	if (!ret)
		ret = wsm_set_bssid_filtering(hw_priv, bssid_filtering,
					priv->if_id);

	if (!ret) {
		ret = wsm_set_multicast_filter(hw_priv, &priv->multicast_filter,
						priv->if_id);
	}

	if (ret)
		wiphy_err(priv->hw->wiphy,
				"%s: Update filtering failed: %d.\n",
				__func__, ret);
	return;
}

void bes2600_update_filtering_work(struct work_struct *work)
{
	struct bes2600_vif *priv =
		container_of(work, struct bes2600_vif,
		update_filtering_work);

	bes2600_update_filtering(priv);
}

void bes2600_set_beacon_wakeup_period_work(struct work_struct *work)
{
	struct bes2600_vif *priv =
		container_of(work, struct bes2600_vif,
		set_beacon_wakeup_period_work);

	WARN_ON(wsm_set_beacon_wakeup_period(priv->hw_priv,
		priv->beacon_int * priv->join_dtim_period >
		MAX_BEACON_SKIP_TIME_MS ? 1 :
		priv->join_dtim_period, 0, priv->if_id));
}

u64 bes2600_prepare_multicast(struct ieee80211_hw *hw,
			     struct netdev_hw_addr_list *mc_list)
{
	static u8 broadcast_ipv6[ETH_ALEN] = {
		0x33, 0x33, 0x00, 0x00, 0x00, 0x01
	};
	static u8 broadcast_ipv4[ETH_ALEN] = {
		0x01, 0x00, 0x5e, 0x00, 0x00, 0x01
	};
	struct bes2600_common *hw_priv = hw->priv;
	struct bes2600_vif *priv = __cw12xx_hwpriv_to_vifpriv(hw_priv, 0);
	struct netdev_hw_addr *ha;
	int count = 0;

	if(priv == NULL){
		bes2600_info(BES2600_DBG_STA, "wlan0 removed before p2p-device\n");
		return netdev_hw_addr_list_count(mc_list);
	}

	/* Disable multicast filtering */
	priv->has_multicast_subscription = false;
	memset(&priv->multicast_filter, 0x00, sizeof(priv->multicast_filter));

	if (netdev_hw_addr_list_count(mc_list) > WSM_MAX_GRP_ADDRTABLE_ENTRIES)
		return 0;

	/* Enable if requested */
	netdev_hw_addr_list_for_each(ha, mc_list) {
		bes2600_info(BES2600_DBG_STA, "[STA] multicast: %pM\n", ha->addr);
		memcpy(&priv->multicast_filter.macAddress[count],
		       ha->addr, ETH_ALEN);
		if (memcmp(ha->addr, broadcast_ipv4, ETH_ALEN) &&
				memcmp(ha->addr, broadcast_ipv6, ETH_ALEN))
			priv->has_multicast_subscription = true;
		count++;
	}

	if (count) {
		priv->multicast_filter.enable = __cpu_to_le32(1);
		priv->multicast_filter.numOfAddresses = __cpu_to_le32(count);
	}

	return netdev_hw_addr_list_count(mc_list);
}

void bes2600_configure_filter(struct ieee80211_hw *hw,
			     unsigned int changed_flags,
			     unsigned int *total_flags,
			     u64 multicast)
{
	struct bes2600_common *hw_priv = hw->priv;
	struct bes2600_vif *priv = __cw12xx_hwpriv_to_vifpriv(hw_priv, 0);
	*total_flags &= FIF_OTHER_BSS |
			FIF_FCSFAIL |
			FIF_BCN_PRBRESP_PROMISC |
			FIF_PROBE_REQ;

	if(priv != NULL){
		down(&hw_priv->scan.lock);
		down(&hw_priv->conf_lock);

		priv->rx_filter.promiscuous = 0;
		priv->rx_filter.bssid = (*total_flags & (FIF_OTHER_BSS |
				FIF_PROBE_REQ)) ? 1 : 0;
		priv->rx_filter.fcs = (*total_flags & FIF_FCSFAIL) ? 1 : 0;
		priv->bf_control.bcn_count = (*total_flags &
				(FIF_BCN_PRBRESP_PROMISC | FIF_PROBE_REQ)) ? 1 : 0;
		bes2600_update_filtering(priv);
		up(&hw_priv->conf_lock);
		up(&hw_priv->scan.lock);
	}
}

int bes2600_conf_tx(struct ieee80211_hw *dev, struct ieee80211_vif *vif,
		    unsigned int link_id, u16 queue,
		    const struct ieee80211_tx_queue_params *params)
{
	struct bes2600_common *hw_priv = dev->priv;
	struct bes2600_vif *priv = cw12xx_get_vif_from_ieee80211(vif);
	int ret = 0;
	/* To prevent re-applying PM request OID again and again*/
	bool old_uapsdFlags;

	if (WARN_ON(!priv))
		return -EOPNOTSUPP;

	if (priv->if_id == CW12XX_GENERIC_IF_ID)
		return 0;
	down(&hw_priv->conf_lock);

	if (queue < dev->queues) {
		old_uapsdFlags = priv->uapsd_info.uapsdFlags;

		WSM_TX_QUEUE_SET(&priv->tx_queue_params, queue, 0, 0, 0);
		ret = wsm_set_tx_queue_params(hw_priv,
				&priv->tx_queue_params.params[queue],
				queue, priv->if_id);
		if (ret) {
			ret = -EINVAL;
			goto out;
		}

                WSM_EDCA_SET(&priv->edca, queue, params->aifs,
                                params->cw_min, params->cw_max, params->txop, 0xc8,
                                params->uapsd);
		ret = wsm_set_edca_params(hw_priv, &priv->edca, priv->if_id);
		if (ret) {
			ret = -EINVAL;
			goto out;
		}

		if (priv->mode == NL80211_IFTYPE_STATION) {
			ret = bes2600_set_uapsd_param(priv, &priv->edca);
		}
	} else
		ret = -EINVAL;

out:
	up(&hw_priv->conf_lock);

	return ret;
}

int bes2600_get_stats(struct ieee80211_hw *dev,
		     struct ieee80211_low_level_stats *stats)
{
	struct bes2600_common *hw_priv = dev->priv;

	memcpy(stats, &hw_priv->stats, sizeof(*stats));
	return 0;
}

/*
int bes2600_get_tx_stats(struct ieee80211_hw *dev,
			struct ieee80211_tx_queue_stats *stats)
{
	int i;
	struct bes2600_common *priv = dev->priv;

	for (i = 0; i < dev->queues; ++i)
		bes2600_queue_get_stats(&priv->tx_queue[i], &stats[i]);

	return 0;
}
*/

int bes2600_set_pm(struct bes2600_vif *priv, const struct wsm_set_pm *arg)
{
	struct wsm_set_pm pm = *arg;

	if (priv->uapsd_info.uapsdFlags != 0)
		pm.pmMode &= ~WSM_PSM_FAST_PS_FLAG;

	if (memcmp(&pm, &priv->firmware_ps_mode,
			sizeof(struct wsm_set_pm))) {
		priv->firmware_ps_mode = pm;
		return wsm_set_pm(priv->hw_priv, &pm,
				priv->if_id);
	} else {
		return 0;
	}
}

int bes2600_set_key(struct ieee80211_hw *dev, enum set_key_cmd cmd,
		   struct ieee80211_vif *vif, struct ieee80211_sta *sta,
		   struct ieee80211_key_conf *key)
{
	int ret = -EOPNOTSUPP;
	struct bes2600_common *hw_priv = dev->priv;
	struct bes2600_vif *priv = cw12xx_get_vif_from_ieee80211(vif);
	struct wsm_protected_mgmt_policy mgmt_policy;

	WARN_ON(priv->if_id == CW12XX_GENERIC_IF_ID);
	memset(&mgmt_policy, 0, sizeof(mgmt_policy));
	down(&hw_priv->conf_lock);
	bes2600_info(BES2600_DBG_STA, "%s, cmd:%d cipher:0x%08x idx:%d\n",
			__func__, cmd, key->cipher, key->keyidx);

	if (cmd == SET_KEY) {
		u8 *peer_addr = NULL;
		int pairwise = (key->flags & IEEE80211_KEY_FLAG_PAIRWISE) ?
			1 : 0;
		int idx = bes2600_alloc_key(hw_priv);
		struct wsm_add_key *wsm_key = &hw_priv->keys[idx];

		if (idx < 0) {
			ret = -EINVAL;
			goto finally;
		}

		BUG_ON(pairwise && !sta);
		if (sta)
			peer_addr = sta->addr;

		key->flags |= IEEE80211_KEY_FLAG_PUT_IV_SPACE |
			      IEEE80211_KEY_FLAG_RESERVE_TAILROOM;

		/* only need to update cipher type of pairwise key */
		if (pairwise)
			priv->cipherType = key->cipher;

		switch (key->cipher) {
		case WLAN_CIPHER_SUITE_WEP40:
		case WLAN_CIPHER_SUITE_WEP104:
			if (key->keylen > 16) {
				bes2600_free_key(hw_priv, idx);
				ret = -EINVAL;
				goto finally;
			}

			if (pairwise) {
				wsm_key->type = WSM_KEY_TYPE_WEP_PAIRWISE;
				memcpy(wsm_key->wepPairwiseKey.peerAddress,
					 peer_addr, ETH_ALEN);
				memcpy(wsm_key->wepPairwiseKey.keyData,
					&key->key[0], key->keylen);
				wsm_key->wepPairwiseKey.keyLength = key->keylen;
			} else {
				wsm_key->type = WSM_KEY_TYPE_WEP_DEFAULT;
				memcpy(wsm_key->wepGroupKey.keyData,
					&key->key[0], key->keylen);
				wsm_key->wepGroupKey.keyLength = key->keylen;
				wsm_key->wepGroupKey.keyId = key->keyidx;
			}
			break;
		case WLAN_CIPHER_SUITE_TKIP:
			if (pairwise) {
				wsm_key->type = WSM_KEY_TYPE_TKIP_PAIRWISE;
				memcpy(wsm_key->tkipPairwiseKey.peerAddress,
					peer_addr, ETH_ALEN);
				memcpy(wsm_key->tkipPairwiseKey.tkipKeyData,
					&key->key[0],  16);
				memcpy(wsm_key->tkipPairwiseKey.txMicKey,
					&key->key[16],  8);
				memcpy(wsm_key->tkipPairwiseKey.rxMicKey,
					&key->key[24],  8);
			} else {
				size_t mic_offset =
					(priv->mode == NL80211_IFTYPE_AP) ?
					16 : 24;
				wsm_key->type = WSM_KEY_TYPE_TKIP_GROUP;
				memcpy(wsm_key->tkipGroupKey.tkipKeyData,
					&key->key[0],  16);
				memcpy(wsm_key->tkipGroupKey.rxMicKey,
					&key->key[mic_offset],  8);

				/* TODO: Where can I find TKIP SEQ? */
				memset(wsm_key->tkipGroupKey.rxSeqCounter,
					0,		8);
				wsm_key->tkipGroupKey.keyId = key->keyidx;

			}
			break;
		case WLAN_CIPHER_SUITE_CCMP:
			if (pairwise) {
				wsm_key->type = WSM_KEY_TYPE_AES_PAIRWISE;
				memcpy(wsm_key->aesPairwiseKey.peerAddress,
					peer_addr, ETH_ALEN);
				memcpy(wsm_key->aesPairwiseKey.aesKeyData,
					&key->key[0],  16);
			} else {
				wsm_key->type = WSM_KEY_TYPE_AES_GROUP;
				memcpy(wsm_key->aesGroupKey.aesKeyData,
					&key->key[0],  16);
				/* TODO: Where can I find AES SEQ? */
				memset(wsm_key->aesGroupKey.rxSeqCounter,
					0,              8);
				wsm_key->aesGroupKey.keyId = key->keyidx;
			}
			break;
		case WLAN_CIPHER_SUITE_AES_CMAC:
			wsm_key->type = WSM_KEY_TYPE_IGTK_GROUP;
			memcpy(wsm_key->igtkGroupKey.IGTKKeyData, &key->key[0], 16);
			memset(wsm_key->igtkGroupKey.IPN, 0, 8);
			wsm_key->igtkGroupKey.keyId = key->keyidx;
			mgmt_policy.protectedMgmtEnable = 1;
			wsm_set_protected_mgmt_policy(hw_priv, &mgmt_policy, priv->if_id);
			break;
		default:
			WARN_ON(1);
			bes2600_free_key(hw_priv, idx);
			ret = -EOPNOTSUPP;
			goto finally;
		}

		ret = WARN_ON(wsm_add_key(hw_priv, wsm_key, priv->if_id));
		if (!ret) {
			key->hw_key_idx = idx;
		} else {
			bes2600_free_key(hw_priv, idx);
		}

		if (!ret && (pairwise
			|| wsm_key->type == WSM_KEY_TYPE_WEP_DEFAULT)
			&& (priv->filter4.enable & 0x2))
				bes2600_set_arpreply(dev, vif);

	} else if (cmd == DISABLE_KEY) {
		struct wsm_remove_key wsm_key = {
			.entryIndex = key->hw_key_idx,
		};

		if (wsm_key.entryIndex > WSM_KEY_MAX_IDX) {
			ret = -EINVAL;
			goto finally;
		}

		bes2600_free_key(hw_priv, wsm_key.entryIndex);
		ret = wsm_remove_key(hw_priv, &wsm_key, priv->if_id);
	} else {
		BUG_ON("Unsupported command");
	}

finally:
	up(&hw_priv->conf_lock);
	return ret;
}

void bes2600_wep_key_work(struct work_struct *work)
{
	struct bes2600_vif *priv =
		container_of(work, struct bes2600_vif , wep_key_work);
	struct bes2600_common *hw_priv = cw12xx_vifpriv_to_hwpriv(priv);
	u8 queueId = bes2600_queue_get_queue_id(hw_priv->pending_frame_id);
	struct bes2600_queue *queue = &hw_priv->tx_queue[queueId];
	__le32 wep_default_key_id = __cpu_to_le32(
		priv->wep_default_key_id);

	BUG_ON(queueId >= 4);

	bes2600_dbg(BES2600_DBG_STA, "[STA] Setting default WEP key: %d\n",
		priv->wep_default_key_id);
	wsm_flush_tx(hw_priv);
	WARN_ON(wsm_write_mib(hw_priv, WSM_MIB_ID_DOT11_WEP_DEFAULT_KEY_ID,
		&wep_default_key_id, sizeof(wep_default_key_id), priv->if_id));
	bes2600_queue_requeue(queue, hw_priv->pending_frame_id, true);
	wsm_unlock_tx(hw_priv);
}

int bes2600_set_rts_threshold(struct ieee80211_hw *hw, u32 value)
{
	struct bes2600_common *hw_priv = hw->priv;
	int ret;
	__le32 val32;
	bes2600_dbg(BES2600_DBG_TXRX_OPT,"set RTS threshold = %d\n\r", hw_priv->rtsvalue);

	spin_lock(&hw_priv->rtsvalue_lock);
	if (hw_priv->rtsvalue != value) {
		hw_priv->rtsvalue = value;
		//bes2600_dbg(BES2600_DBG_TXRX_OPT,"set RTS value = %d\n\r", hw_priv->rtsvalue);
	} else {
		spin_unlock(&hw_priv->rtsvalue_lock);
		return 0;
	}
	spin_unlock(&hw_priv->rtsvalue_lock);

	if (value != (u32) -1)
		val32 = __cpu_to_le32(value);
	else
		val32 = 0; /* disabled */
	/* mutex_lock(&priv->conf_mutex); */
	ret = WARN_ON(wsm_write_mib(hw_priv, WSM_MIB_ID_DOT11_RTS_THRESHOLD,
		&val32, sizeof(val32), 0));
	/* mutex_unlock(&priv->conf_mutex); */

	return ret;
}

/* TODO: COMBO: Flush only a particular interface specific parts */
int __bes2600_flush(struct bes2600_common *hw_priv, bool drop, int if_id)
{
	int i, ret;
	struct bes2600_vif *priv =
		__cw12xx_hwpriv_to_vifpriv(hw_priv, if_id);

	/* clear tx queue directly if there is a bus error */
	if(hw_priv->bh_error)
		drop = true;

	for (;;) {
		/* TODO: correct flush handling is required when dev_stop.
		 * Temporary workaround: 2s
		 */
		if (drop) {
			for (i = 0; i < 4; ++i)
				bes2600_queue_clear(&hw_priv->tx_queue[i],
					if_id);
		} else {
			ret = wait_event_timeout(
				hw_priv->tx_queue_stats.wait_link_id_empty,
				bes2600_queue_stats_is_empty(
					&hw_priv->tx_queue_stats, -1, if_id),
				2 * HZ);
		}

		if (!drop && unlikely(ret <= 0)) {
			bes2600_err(BES2600_DBG_STA, "__bes2600_flush: ETIMEDOUT.....\n");
			ret = -ETIMEDOUT;
			break;
		} else {
			ret = 0;
		}

		wsm_vif_lock_tx(priv);
		if (unlikely(!bes2600_queue_stats_is_empty(
				&hw_priv->tx_queue_stats, -1, if_id))) {
			/* Highly unlekely: WSM requeued frames. */
			wsm_unlock_tx(hw_priv);
			continue;
		}
		break;
	}
	return ret;
}

void bes2600_flush(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		  u32 queues, bool drop)
{
	struct bes2600_vif *priv = NULL;
	struct bes2600_common *hw_priv = hw->priv;
	unsigned int ret = 0;
	int i;

	/* get queue status */
	if (vif) {
		priv = cw12xx_get_vif_from_ieee80211(vif);
		ret |= !bes2600_queue_stats_is_empty(
				&hw_priv->tx_queue_stats, -1, priv->if_id);
	} else {
		bes2600_for_each_vif(hw_priv, priv, i) {
			if (!priv)
				continue;
			if (!(hw_priv->if_id_slot & BIT(priv->if_id)))
				return;
			ret |= !bes2600_queue_stats_is_empty(
				&hw_priv->tx_queue_stats, -1, priv->if_id);
		}
	}

	/* no need to do the next work if queue was already clear */
	if(ret == 0) {
		bes2600_info(BES2600_DBG_STA, "no need to flush\n");
		return;
	}

	/* do flush operation */
	bes2600_pwr_set_busy_event(hw_priv, BES_PWR_LOCK_ON_FLUSH);
	if (vif) {
		priv = cw12xx_get_vif_from_ieee80211(vif);
		if (!(hw_priv->if_id_slot & BIT(priv->if_id)))
			return;
		if (!WARN_ON(__bes2600_flush(hw_priv, drop, priv->if_id)))
			wsm_unlock_tx(hw_priv);
	} else {
		bes2600_for_each_vif(hw_priv, priv, i) {
			if (!priv)
				continue;
			if (!(hw_priv->if_id_slot & BIT(priv->if_id)))
				return;
			if (!WARN_ON(__bes2600_flush(hw_priv, drop, priv->if_id)))
				wsm_unlock_tx(hw_priv);
		}
	}
	bes2600_pwr_clear_busy_event(hw_priv, BES_PWR_LOCK_ON_FLUSH);

	return;
}

int bes2600_remain_on_channel(struct ieee80211_hw *hw,
				 struct ieee80211_vif *vif,
				 struct ieee80211_channel *chan,
				 int duration,
				 enum ieee80211_roc_type type)
{
	int ret;
	struct bes2600_common *hw_priv = hw->priv;
	struct bes2600_vif *priv = cw12xx_get_vif_from_ieee80211(vif);
	int if_id = priv->if_id;

	bes2600_pwr_set_busy_event(hw_priv, BES_PWR_LOCK_ON_ROC);
	down(&hw_priv->scan.lock);
	down(&hw_priv->conf_lock);


	bes2600_info(BES2600_DBG_ROC, "ROC IN %d ch %d\n", priv->if_id, chan->hw_value);
	hw_priv->roc_if_id = priv->if_id;
	ret = WARN_ON(__bes2600_flush(hw_priv, false, if_id));
	wsm_unlock_tx(hw_priv);
	ret = bes2600_enable_listening(priv, chan);

	if (!ret) {
		atomic_set(&hw_priv->remain_on_channel, 1);
		queue_delayed_work(hw_priv->workqueue,
				   &hw_priv->rem_chan_timeout,
				   duration * HZ / 1000);
		priv->join_status = BES2600_JOIN_STATUS_MONITOR;
		ieee80211_ready_on_channel(hw);
	} else {
		hw_priv->roc_if_id = -1;
		up(&hw_priv->scan.lock);
		bes2600_pwr_clear_busy_event(hw_priv, BES_PWR_LOCK_ON_ROC);
	}

	bes2600_info(BES2600_DBG_ROC, "ROC OUT %d\n", priv->if_id);

	/* set the channel to supplied ieee80211_channel pointer, if it
        is not set. This is to remove the crash while sending a probe res
        in listen state. Later channel will updated on
        IEEE80211_CONF_CHANGE_CHANNEL event*/
	if(!hw_priv->channel) {
		hw_priv->channel = chan;
	}
	//hw_priv->roc_cookie = cookie;
	up(&hw_priv->conf_lock);
	return ret;
}

int bes2600_cancel_remain_on_channel(struct ieee80211_hw *hw,
				     struct ieee80211_vif *vif)
{
	struct bes2600_common *hw_priv = hw->priv;

	bes2600_info(BES2600_DBG_STA, "[STA] Cancel remain on channel\n");

	if (atomic_read(&hw_priv->remain_on_channel))
		cancel_delayed_work_sync(&hw_priv->rem_chan_timeout);

	if (atomic_read(&hw_priv->remain_on_channel))
		bes2600_rem_chan_timeout(&hw_priv->rem_chan_timeout.work);

	return 0;
}

/* ******************************************************************** */
/* WSM callbacks							*/

void bes2600_channel_switch_cb(struct bes2600_common *hw_priv)
{
	//wsm_unlock_tx(hw_priv);  /* already unlock in wsm_channel_switch_indication */
}

void bes2600_free_event_queue(struct bes2600_common *hw_priv)
{
	LIST_HEAD(list);

	spin_lock(&hw_priv->event_queue_lock);
	list_splice_init(&hw_priv->event_queue, &list);
	spin_unlock(&hw_priv->event_queue_lock);

	__bes2600_free_event_queue(&list);
}

void bes2600_event_handler(struct work_struct *work)
{
	struct bes2600_common *hw_priv =
		container_of(work, struct bes2600_common, event_handler);
	struct bes2600_vif *priv;
	struct bes2600_wsm_event *event;
	LIST_HEAD(list);

	spin_lock(&hw_priv->event_queue_lock);
	list_splice_init(&hw_priv->event_queue, &list);
	spin_unlock(&hw_priv->event_queue_lock);

	down(&hw_priv->conf_lock);
	list_for_each_entry(event, &list, link) {
		priv = __cw12xx_hwpriv_to_vifpriv(hw_priv, event->if_id);
		if (!priv) {
			bes2600_dbg(BES2600_DBG_STA, "[CQM] Event for non existing "
				   "interface, ignoring.\n");
			continue;
		}
		switch (event->evt.eventId) {
			case WSM_EVENT_ERROR:
				/* I even don't know what is it about.. */
				//STUB();
				break;
			case WSM_EVENT_BSS_LOST:
			{
				spin_lock(&priv->bss_loss_lock);
				if (priv->bss_loss_status > BES2600_BSS_LOSS_NONE) {
					spin_unlock(&priv->bss_loss_lock);
					break;
				}
				priv->bss_loss_status = BES2600_BSS_LOSS_CHECKING;
				spin_unlock(&priv->bss_loss_lock);
				bes2600_info(BES2600_DBG_STA, "[CQM] BSS lost.\n");
				bes2600_pwr_set_busy_event(hw_priv, BES_PWR_LOCK_ON_BSS_LOST);
				cancel_delayed_work_sync(&priv->bss_loss_work);
				cancel_delayed_work_sync(&priv->connection_loss_work);
				if (!down_trylock(&hw_priv->scan.lock)) {
					up(&hw_priv->scan.lock);
					priv->delayed_link_loss = 0;
					queue_delayed_work(hw_priv->workqueue,
							&priv->bss_loss_work, 0);
				} else {
					/* Scan is in progress. Delay reporting. */
					/* Scan complete will trigger bss_loss_work */
					priv->delayed_link_loss = 1;
					/* Also we're starting watchdog. */
					queue_delayed_work(hw_priv->workqueue,
							&priv->bss_loss_work, 10 * HZ);
				}
				break;
			}
			case WSM_EVENT_BSS_REGAINED:
			{
				bes2600_info(BES2600_DBG_STA, "[CQM] BSS regained.\n");
				priv->delayed_link_loss = 0;
				spin_lock(&priv->bss_loss_lock);
				priv->bss_loss_status = BES2600_BSS_LOSS_NONE;
				spin_unlock(&priv->bss_loss_lock);
				bes2600_pwr_clear_busy_event(hw_priv, BES_PWR_LOCK_ON_BSS_LOST);
				cancel_delayed_work_sync(&priv->bss_loss_work);
				cancel_delayed_work_sync(&priv->connection_loss_work);
				break;
			}
			case WSM_EVENT_RADAR_DETECTED:
				//STUB();
				break;
			case WSM_EVENT_RCPI_RSSI:
			{
				/* RSSI: signed Q8.0, RCPI: unsigned Q7.1
				 * RSSI = RCPI / 2 - 110
				 */
				int rcpi_rssi = (int)(event->evt.eventData & 0xFF);
				int cqm_evt;
				if (priv->cqm_use_rssi)
					rcpi_rssi = (s8)rcpi_rssi;
				else
					rcpi_rssi =  rcpi_rssi / 2 - 110;

				cqm_evt = (rcpi_rssi <= priv->cqm_rssi_thold) ?
					NL80211_CQM_RSSI_THRESHOLD_EVENT_LOW :
					NL80211_CQM_RSSI_THRESHOLD_EVENT_HIGH;
				bes2600_info(BES2600_DBG_STA, "[CQM] RSSI event: %d.\n", rcpi_rssi);
				ieee80211_cqm_rssi_notify(priv->vif, cqm_evt, rcpi_rssi, GFP_KERNEL);
				break;
			}
			case WSM_EVENT_BT_INACTIVE:
				//STUB();
				break;
			case WSM_EVENT_BT_ACTIVE:
				//STUB();
				break;
			case WSM_EVENT_INACTIVITY:
			{
				int link_id = ffs((u32)(event->evt.eventData)) - 1;
				struct sk_buff *skb;
				struct ieee80211_mgmt *deauth;
				struct bes2600_link_entry *entry = NULL;

				bes2600_info(BES2600_DBG_STA, "Inactivity Event Recieved for "
						"link_id %d\n", link_id);
				cw12xx_unmap_link(priv, link_id);

				skb = dev_alloc_skb(sizeof(struct ieee80211_mgmt) + 64);
				skb_reserve(skb, 64);
				deauth = (struct ieee80211_mgmt *)skb_put(skb, sizeof(struct ieee80211_mgmt));
				WARN_ON(!deauth);
				entry = &priv->link_id_db[link_id - 1];
				deauth->duration = 0;
				memcpy(deauth->da, priv->vif->addr, ETH_ALEN);
				memcpy(deauth->sa, entry->mac/*priv->link_id_db[i].mac*/, ETH_ALEN);
				memcpy(deauth->bssid, priv->vif->addr, ETH_ALEN);
				deauth->frame_control = cpu_to_le16(IEEE80211_FTYPE_MGMT |
						IEEE80211_STYPE_DEAUTH | IEEE80211_FCTL_TODS);
				deauth->u.deauth.reason_code = WLAN_REASON_DEAUTH_LEAVING;
				deauth->seq_ctrl = 0;
				ieee80211_rx_irqsafe(priv->hw, skb);
				bes2600_info(BES2600_DBG_STA, " Inactivity Deauth Frame sent for MAC SA %pM \t and DA %pM\n", deauth->sa, deauth->da);
				queue_work(priv->hw_priv->workqueue, &priv->set_tim_work);
				break;
			}
		case WSM_EVENT_PS_MODE_ERROR:
			{
				bes2600_err(BES2600_DBG_STA, " PS Mode Error, Reason:%u\n", event->evt.eventData);

				if (event->evt.eventData != WSM_PS_ERROR_AP_NO_DATA_AFTER_TIM &&
				    !priv->uapsd_info.uapsdFlags &&
				    (priv->user_pm_mode != WSM_PSM_PS))
				{
					bes2600_pwr_mark_ap_lp_bad(hw_priv);
				 	priv->powersave_mode.pmMode = WSM_PSM_ACTIVE;
				}
				break;
			}
		case WSM_EVENT_WAKEUP_EVENT:
			bes2600_info(BES2600_DBG_STA, "wifi wakeup, Reason:%u\n", event->evt.eventData);
			bes2600_chrdev_wifi_update_wakeup_reason(event->evt.eventData);
			break;
		}
	}
	up(&hw_priv->conf_lock);
	__bes2600_free_event_queue(&list);
}

void bes2600_bss_loss_work(struct work_struct *work)
{
	struct bes2600_vif *priv =
		container_of(work, struct bes2600_vif, bss_loss_work.work);
	struct bes2600_common *hw_priv = cw12xx_vifpriv_to_hwpriv(priv);
	int timeout; /* in beacons */
	struct sk_buff *skb;
	struct ieee80211_tx_info *info;
	static int bl_ck_cnt = 0;
	static int bl_cfm_cnt = 0;

	timeout = priv->cqm_link_loss_count -
		priv->cqm_beacon_loss_count;

	/* Skip the confimration procedure in P2P case */
	if (priv->vif->p2p)
		goto report;

	spin_lock(&priv->bss_loss_lock);
	if (!priv->vif->cfg.assoc) {
		priv->bss_loss_status = BES2600_BSS_LOSS_NONE;
		spin_unlock(&priv->bss_loss_lock);
		bl_ck_cnt = 0;
		bl_cfm_cnt = 0;
		return;
	}
	if (priv->bss_loss_status == BES2600_BSS_LOSS_CHECKING) {
		spin_unlock(&priv->bss_loss_lock);
		bes2600_info(BES2600_DBG_STA, "bl checking\n");
		priv->cmq_tx_success_count = 0;
		bl_ck_cnt = 0;
		bl_cfm_cnt = 0;
		skb = ieee80211_nullfunc_get(priv->hw, priv->vif, 0, false);
		if (!(WARN_ON(!skb))) {
			info = IEEE80211_SKB_CB(skb);
			info->control.vif = priv->vif;
			bes2600_tx(priv->hw, NULL, skb);
			/* Start watchdog -- if nullfunc TX doesn't fail
			 * in 1 sec, forward event to upper layers */
			queue_delayed_work(hw_priv->workqueue,
					   &priv->bss_loss_work, 1 * HZ);
		}
		return;
	} else if (priv->bss_loss_status == BES2600_BSS_LOSS_CONFIRMING) {
		/* reset cca to workaround rx stuck issue */
		// bes2600_cca_soft_reset();

		/* succeeded to send last null frame */
		bl_cfm_cnt = 0;
		/* keep checking to wait bss regain*/
		bes2600_info(BES2600_DBG_STA, "bl ck %d\n", bl_ck_cnt);
		if (bl_ck_cnt++ < BSS_LOSS_CK_THR) {
			spin_unlock(&priv->bss_loss_lock);
			priv->bss_loss_status = BES2600_BSS_LOSS_CHECKING;
			skb = ieee80211_nullfunc_get(priv->hw, priv->vif, 0, false);
			if (!(WARN_ON(!skb))) {
				info = IEEE80211_SKB_CB(skb);
				info->control.vif = priv->vif;
				bes2600_tx(priv->hw, NULL, skb);
				/* detect every 3s until receive bss regain event */
				queue_delayed_work(hw_priv->workqueue,
						   &priv->bss_loss_work, BSS_LOSS_CK_INV * HZ / 1000);
			}
			return;
		} else {
			bl_ck_cnt = 0;
		}
	} else if (priv->bss_loss_status == BES2600_BSS_LOSS_CONFIRMED) {
		/* failed to send last null frame */
		/* check if continous failures occur */
		if (priv->cmq_tx_success_count != 0) {
			bes2600_info(BES2600_DBG_STA, "bl reset ck cfm s_cnt=%d\n", priv->cmq_tx_success_count);
			bl_ck_cnt = 0;
			bl_cfm_cnt = 0;
			priv->cmq_tx_success_count = 0;
		}
		bes2600_info(BES2600_DBG_STA, "bl cfm %d\n", bl_cfm_cnt);
		if (bl_cfm_cnt++ < BSS_LOSS_CFM_THR) {
			spin_unlock(&priv->bss_loss_lock);
			priv->bss_loss_status = BES2600_BSS_LOSS_CHECKING;
			skb = ieee80211_nullfunc_get(priv->hw, priv->vif, 0, false);
			if (!(WARN_ON(!skb))) {
				info = IEEE80211_SKB_CB(skb);
				info->control.vif = priv->vif;
				bes2600_tx(priv->hw, NULL, skb);
				queue_delayed_work(hw_priv->workqueue,
						   &priv->bss_loss_work, BSS_LOSS_CK_INV * HZ / 1000);
			}
			return;
		} else {
			bl_cfm_cnt = 0;
		}
	}
	spin_unlock(&priv->bss_loss_lock);

report:
	if (priv->cqm_beacon_loss_count) {
		bes2600_info(BES2600_DBG_STA, "[CQM] Beacon loss.\n");
		if (timeout <= 0)
			timeout = 0;
	} else {
		timeout = 0;
	}

	cancel_delayed_work_sync(&priv->connection_loss_work);
	queue_delayed_work(hw_priv->workqueue,
		&priv->connection_loss_work,
		timeout * HZ / 10);

	spin_lock(&priv->bss_loss_lock);
	priv->bss_loss_status = BES2600_BSS_LOSS_NONE;
	spin_unlock(&priv->bss_loss_lock);
}

void bes2600_connection_loss_work(struct work_struct *work)
{
	struct bes2600_vif *priv =
		container_of(work, struct bes2600_vif,
				connection_loss_work.work);
	bes2600_info(BES2600_DBG_STA, "[CQM] Reporting connection loss.\n");
	bes2600_pwr_clear_busy_event(priv->hw_priv, BES_PWR_LOCK_ON_BSS_LOST);
	ieee80211_connection_loss(priv->vif);
	// set disconnected in BSS_CHANGED_ASSOC
	// bwifi_change_current_status(hw_priv, BWIFI_STATUS_DISCONNECTED);
}

void bes2600_tx_failure_work(struct work_struct *work)
{
	struct bes2600_vif *priv =
		container_of(work, struct bes2600_vif, tx_failure_work);
	bes2600_info(BES2600_DBG_STA, "[CQM] Reporting TX failure.\n");
	(void)priv;
}

/* ******************************************************************** */
/* Internal API								*/



/*
* This function is called to Parse the SDD file
 *to extract listen_interval and PTA related information
*/
int bes2600_parse_SDD_file(struct bes2600_common *hw_priv)
{
	u8 *sdd_data = (u8 *)hw_priv->sdd->data;
	struct bes2600_sdd {
		u8 id ;
		u8 length ;
		u8 data[] ;
	} *pElement;
	int parsedLength = 0;
	#define SDD_PTA_CFG_ELT_ID 0xEB
	#define FIELD_OFFSET(type, field) ((u8 *)&((type *)0)->field - (u8 *)0)

	hw_priv->is_BT_Present = false;

	pElement = (struct bes2600_sdd *)sdd_data;

	pElement = (struct bes2600_sdd *)((u8 *)pElement +
		FIELD_OFFSET(struct bes2600_sdd, data) + pElement->length);

	parsedLength += (FIELD_OFFSET(struct bes2600_sdd, data) +
			pElement->length);

	while (parsedLength <= hw_priv->sdd->size) {
		switch (pElement->id) {
		case SDD_PTA_CFG_ELT_ID:
		{
			hw_priv->conf_listen_interval =
				(*((u16 *)pElement->data+1) >> 7) & 0x1F;
			hw_priv->is_BT_Present = true;
			bes2600_dbg(BES2600_DBG_STA, "PTA element found.\n");
			bes2600_dbg(BES2600_DBG_STA, "Listen Interval %d\n",
						hw_priv->conf_listen_interval);
		}
		break;

		default:
		break;
		}

		pElement = (struct bes2600_sdd *)
			((u8 *)pElement + FIELD_OFFSET(struct bes2600_sdd, data)
					+ pElement->length);
		parsedLength +=
		(FIELD_OFFSET(struct bes2600_sdd, data) + pElement->length);
	}

	if (hw_priv->is_BT_Present == false) {
		bes2600_dbg(BES2600_DBG_STA, "PTA element NOT found.\n");
		hw_priv->conf_listen_interval = 0;
	}
        bes2600_dbg(BES2600_DBG_STA, "%s output power before %d\n",__func__,hw_priv->output_power);
        /*BUG:TX output power: Hardcoding to 20dbm if CCX is not enabled*/
        /*TODO: This might change*/
        if (!hw_priv->output_power)
                hw_priv->output_power=20;
        bes2600_dbg(BES2600_DBG_STA, "%s output power after %d\n",__func__,hw_priv->output_power);
	return 0;

	#undef SDD_PTA_CFG_ELT_ID
	#undef FIELD_OFFSET
}

#define USE_FEM 0
const uint8_t sdd_22[] =
{
	0xc0, 0x02, 0x02, 0x03,
	0xc1, 0x06, 0x00, 0x01, 0x00, 0x00, 0x4c, 0x00,
	0xc4, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xc5, 0x02, 0x90, 0x65,
	0xc7, 0x02, 0x05, 0x00,
	0xe9, 0x02, 0xB8, 0x0B,	// SddWakeupDriftInUsPerSecond=3000
	0xea, 0x0a, 0x00, 0x00, 0x38, 0x00, 0x80, 0x00, 0x78, 0x00, 0x80, 0x00,
	0xeb, 0x1e, 0x06, 0x00, 0x81, 0x02, 0x01, 0x34, 0x0d, 0x21, 0x11, 0x1d, 0x7c, 0x85, 0x6b, 0x6e, 0x0d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xfe, 0x02, 0x00, 0x00,
	0xc1, 0x06, 0x01, 0x01, 0x00, 0x00, 0x30, 0x00,
	0xf0, 0x22, 0x41, 0x44, 0x42, 0x5f, 0x32, 0x39, 0x30, 0x35, 0x5f, 0x31, 0x32, 0x35, 0x30, 0x5f, 0x52, 0x32, 0x30, 0x5f, 0x4c, 0x6f, 0x77, 0x5f, 0x53, 0x4d, 0x50, 0x53, 0x5f, 0x76, 0x37, 0x2e, 0x30, 0x31, 0x00, 0x00,
	0xfe, 0x02, 0x00, 0x00,
	0xc1, 0x06, 0x03, 0x01, 0x00, 0x00, 0xb4, 0x02,
	0xec, 0x12, 0x05, 0x00, 0x01, 0xc0, 0xc0, 0x02, 0xc0, 0xc0, 0x0b, 0xc0, 0xc0, 0x0c, 0xc0, 0xc0, 0x0d, 0xc0, 0xc0, 0x00,
	0xed, 0x0e, 0x05, 0x00, 0xb8, 0x78, 0x22, 0x78, 0x34, 0x78, 0x64, 0x78, 0x95, 0x78, 0x00, 0x00,
	0xe3, 0x1a, 0x1c, 0x01, 0x1c, 0x01, 0x18, 0x01, 0x18, 0x01, 0x18, 0x01, 0x18, 0x01, 0x18, 0x01, 0x10, 0x01, 0x00, 0x01, 0xf0, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xe4, 0x1a, 0x24, 0x01, 0x24, 0x01, 0x24, 0x01, 0x24, 0x01, 0x24, 0x01, 0x24, 0x01, 0x14, 0x01, 0x04, 0x01, 0xf4, 0x00, 0xe4, 0x00, 0xd4, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x20, 0x1a, 0x8d, 0x03, 0x73, 0x02, 0x73, 0x03, 0x40, 0x03, 0xb3, 0x02, 0x40, 0x04, 0xcd, 0x04, 0x5f, 0x00, 0x00, 0x05, 0x5f, 0x00, 0x33, 0x05, 0x5f, 0x00, 0x00, 0x00,
	0x21, 0x1a, 0x73, 0x03, 0x73, 0x02, 0x8d, 0x02, 0x40, 0x03, 0x1a, 0x02, 0x40, 0x04, 0x00, 0x04, 0x38, 0x00, 0xcd, 0x04, 0x66, 0x00, 0x00, 0x05, 0x57, 0x00, 0x00, 0x00,
	0x22, 0x02, 0x07, 0x00,
	0x23, 0x02, 0x07, 0x00,
	0x30, 0x02, 0x08, 0x00,
	0x31, 0x02, 0x08, 0x00,
	0xe0, 0x02, 0x3a, 0x00,
	0xe1, 0x02, 0x32, 0x00,
	0x40, 0x0a, 0xf4, 0xff, 0x42, 0x00, 0xdd, 0xff, 0x3f, 0x00, 0x00, 0x00,
	0x41, 0x0a, 0x80, 0xff, 0x63, 0x00, 0xd8, 0xff, 0x20, 0x00, 0x00, 0x00,
	0x46, 0x22, 0x00, 0x00, 0x04, 0x1a, 0x08, 0x05, 0x01, 0x9f, 0x00, 0x00, 0x9a, 0xfd, 0x92, 0xff, 0x0a, 0x0a, 0x00, 0x00, 0xc0, 0x02, 0x6c, 0x00, 0x7c, 0xf5, 0x00, 0x00, 0x64, 0xfe, 0xca, 0xff, 0x78, 0x06, 0x00, 0x00,
	0x47, 0x22, 0x00, 0x00, 0xbe, 0xf6, 0xe4, 0x00, 0x10, 0x05, 0x00, 0x00, 0x27, 0x01, 0x5c, 0x01, 0x44, 0x00, 0x00, 0x00, 0x14, 0xfe, 0x93, 0xff, 0xb0, 0x00, 0x00, 0x00, 0x70, 0x01, 0x4a, 0x00, 0x6f, 0xff, 0x00, 0x00,
	0x44, 0x22, 0x00, 0x00, 0xb0, 0x03, 0x53, 0xfc, 0xf9, 0x05, 0x00, 0x00, 0xf6, 0xfe, 0x84, 0x00, 0x36, 0xff, 0x00, 0x00, 0x3b, 0x01, 0x68, 0xff, 0xe8, 0x00, 0x00, 0x00, 0x78, 0xfe, 0x7d, 0x00, 0x51, 0xff, 0x00, 0x00,
	0x45, 0x22, 0x00, 0x00, 0xda, 0x03, 0xf0, 0xff, 0x34, 0xff, 0x00, 0x00, 0x9e, 0xff, 0xfd, 0xff, 0x0d, 0x00, 0x00, 0x00, 0xa8, 0x00, 0xe0, 0xff, 0xfe, 0xff, 0x00, 0x00, 0x7e, 0xff, 0x29, 0x00, 0xf0, 0xff, 0x00, 0x00,
	0x42, 0x22, 0x00, 0x00, 0x08, 0xf8, 0xf0, 0xff, 0x58, 0x00, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x00, 0xf8, 0xff, 0x00, 0x00, 0x89, 0xff, 0x06, 0x00, 0x05, 0x00, 0x00, 0x00, 0xd2, 0xff, 0xff, 0xff, 0xfe, 0xff, 0x00, 0x00,
	0x43, 0x22, 0x00, 0x00, 0xac, 0x00, 0x0a, 0xff, 0x63, 0x03, 0x00, 0x00, 0x31, 0xff, 0xf2, 0x00, 0xed, 0xfe, 0x00, 0x00, 0x58, 0x00, 0xa2, 0xff, 0x93, 0x00, 0x00, 0x00, 0xb4, 0xff, 0x65, 0x00, 0x74, 0xff, 0x00, 0x00,
	0x48, 0x16, 0x40, 0x01, 0x40, 0x01, 0x40, 0x01, 0x40, 0x01, 0x40, 0x01, 0x40, 0x01, 0x40, 0x01, 0x40, 0x01, 0x40, 0x01, 0x40, 0x01, 0x40, 0x01,
	0x49, 0x16, 0x60, 0x01, 0x60, 0x01, 0x60, 0x01, 0x60, 0x01, 0x60, 0x01, 0x60, 0x01, 0x60, 0x01, 0x60, 0x01, 0x60, 0x01, 0x60, 0x01, 0x60, 0x01,
	0x50, 0x06, 0xe4, 0x48, 0x87, 0x43, 0xef, 0xe8,
	0x51, 0x06, 0x9b, 0x0c, 0x67, 0xef, 0x4f, 0x16,
	0x52, 0x1a, 0x00, 0x00, 0xb4, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe9, 0x6a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0xae, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x53, 0x1a, 0x00, 0x00, 0x9a, 0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2b, 0xcb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5d, 0xaa, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x54, 0x1a, 0x00, 0x00, 0x06, 0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x85, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x43, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x55, 0x1a, 0x00, 0x00, 0x10, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x98, 0x39, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe4, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x56, 0x1a, 0x00, 0x00, 0xf3, 0xb5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x37, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0xd4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x57, 0x1a, 0x00, 0x00, 0xc0, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xd4, 0x4b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf5, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xc6, 0x22, 1,  1,  1,  1,  0,  0,  0,  0,  1,  1,  1,  1,  0,  0,  0,  0,  1,  1,  1,  1,  0,  0,  0,  0,  1,  1,  1,  1,  0,  0,  0,  0, 0, 0,
	0xfe, 0x02, 0x00, 0x00,
	0xc1, 0x06, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00
};

struct firmware g_sdd;

int bes2600_setup_mac(struct bes2600_common *hw_priv)
{
	int ret = 0, if_id;
	if (!hw_priv->sdd) {
		const char *sdd_path = NULL;
		struct wsm_configuration cfg = {
			.dot11StationId = &hw_priv->mac_addr[0],
		};

		switch (hw_priv->hw_revision) {
		case BES2600_HW_REV_CUT10:
			sdd_path = SDD_FILE_10;
			break;
		case BES2600_HW_REV_CUT11:
			sdd_path = SDD_FILE_11;
			break;
		case BES2600_HW_REV_CUT20:
			sdd_path = SDD_FILE_20;
			break;
		case BES2600_HW_REV_CUT22:
			sdd_path = SDD_FILE_22;
			break;
		case CW1250_HW_REV_CUT11:
			sdd_path = SDD_FILE_1250_11;
			break;
		default:
			BUG_ON(1);
		}

		g_sdd.data = sdd_22;
		g_sdd.size = sizeof(sdd_22);
		hw_priv->sdd = &g_sdd;
		cfg.dpdData = sdd_22;
		cfg.dpdData_size = g_sdd.size;

		for (if_id = 0; if_id < 2;
		     if_id++) {
			/* Set low-power mode. */
			ret |= WARN_ON(wsm_configuration(hw_priv, &cfg,
				       if_id));
		}
		/* Parse SDD file for PTA element */
		//bes2600_parse_SDD_file(hw_priv);
	}

	if (ret)
		return ret;

	return 0;
}

void bes2600_pending_offchanneltx_work(struct work_struct *work)
{
	struct bes2600_vif *priv =
	container_of(work, struct bes2600_vif, pending_offchanneltx_work.work);
	struct bes2600_common *hw_priv = cw12xx_vifpriv_to_hwpriv(priv);

	down(&hw_priv->conf_lock);

	bes2600_dbg(BES2600_DBG_ROC, "OFFCHAN PEND IN\n");
	bes2600_disable_listening(priv);
	hw_priv->roc_if_id = -1;
	bes2600_dbg(BES2600_DBG_ROC, "OFFCHAN PEND OUT\n");

	up(&hw_priv->scan.lock);
	up(&hw_priv->conf_lock);
}

void bes2600_offchannel_work(struct work_struct *work)
{
	struct bes2600_vif *priv =
		container_of(work, struct bes2600_vif, offchannel_work);
	struct bes2600_common *hw_priv = cw12xx_vifpriv_to_hwpriv(priv);
	u8 queueId = bes2600_queue_get_queue_id(hw_priv->pending_frame_id);
	struct bes2600_queue *queue = &hw_priv->tx_queue[queueId];

	BUG_ON(queueId >= 4);
	BUG_ON(!hw_priv->channel);

	if (unlikely(down_trylock(&hw_priv->scan.lock))) {
		int ret = 0;
		bes2600_dbg(BES2600_DBG_STA, "bes2600_offchannel_work***** drop frame\n");
		ret = bes2600_queue_remove(queue, hw_priv->pending_frame_id);
		if (ret)
			bes2600_err(BES2600_DBG_STA, "bes2600_offchannel_work: "
				       "queue_remove failed %d\n", ret);
		wsm_unlock_tx(hw_priv);
		return;
	}
	down(&hw_priv->conf_lock);
	bes2600_dbg(BES2600_DBG_ROC, "OFFCHAN WORK IN %d\n", priv->if_id);
	hw_priv->roc_if_id = priv->if_id;
	if (likely(!priv->join_status)) {
		wsm_vif_flush_tx(priv);
		bes2600_enable_listening(priv, hw_priv->channel);
		/* bes2600_update_filtering(priv); */
	}
	if (unlikely(!priv->join_status))
		bes2600_queue_remove(queue, hw_priv->pending_frame_id);
	else
		bes2600_queue_requeue(queue, hw_priv->pending_frame_id, false);

	queue_delayed_work(hw_priv->workqueue,
			&priv->pending_offchanneltx_work, 204 * HZ/1000);
	bes2600_dbg(BES2600_DBG_ROC, "OFFCHAN WORK OUT %d\n", priv->if_id);
	up(&hw_priv->conf_lock);
	wsm_unlock_tx(hw_priv);
}

void bes2600_join_work(struct work_struct *work)
{
	struct bes2600_vif *priv =
		container_of(work, struct bes2600_vif, join_work);
	struct bes2600_common *hw_priv = cw12xx_vifpriv_to_hwpriv(priv);
	u8 queueId = bes2600_queue_get_queue_id(hw_priv->pending_frame_id);
	struct bes2600_queue *queue = &hw_priv->tx_queue[queueId];
	const struct bes2600_txpriv *txpriv = NULL;
	struct sk_buff *skb = NULL;
	const struct wsm_tx *wsm;
	const struct ieee80211_hdr *frame;
	const u8 *bssid;
	struct cfg80211_bss *bss;
	const u8 *ssidie;
	const u8 *dtimie;
	const struct ieee80211_tim_ie *tim = NULL;
	struct wsm_protected_mgmt_policy mgmt_policy;
	struct ieee80211_conf *conf = &hw_priv->hw->conf;
	struct wsm_template_frame probe_tmp = {
		.frame_type = WSM_FRAME_TYPE_PROBE_REQUEST,
	};
	/*struct wsm_reset reset = {
		.reset_statistics = true,
	};*/


	BUG_ON(queueId >= 4);
	if (bes2600_queue_get_skb(queue, hw_priv->pending_frame_id,
			&skb, &txpriv)) {
		wsm_unlock_tx(hw_priv);
		return;
	}
	wsm = (struct wsm_tx *)&skb->data[0];
	frame = (struct ieee80211_hdr *)&skb->data[txpriv->offset];
	bssid = &frame->addr1[0]; /* AP SSID in a 802.11 frame */

	BUG_ON(!wsm);
	BUG_ON(!hw_priv->channel);

	if (unlikely(priv->join_status)) {
		atomic_set(&priv->connect_in_process, 0);
		wsm_lock_tx(hw_priv);
		bes2600_unjoin_work(&priv->unjoin_work);
	}

	cancel_delayed_work_sync(&priv->join_timeout);

	bss = cfg80211_get_bss(hw_priv->hw->wiphy, hw_priv->channel, bssid, NULL, 0,
			       IEEE80211_BSS_TYPE_ANY, IEEE80211_PRIVACY_ANY);

	if (!bss) {
		bes2600_queue_remove(queue, hw_priv->pending_frame_id);
		wsm_unlock_tx(hw_priv);
		return;
	}
	ssidie = ieee80211_bss_get_ie(bss, WLAN_EID_SSID);
	dtimie = ieee80211_bss_get_ie(bss, WLAN_EID_TIM);
	if (dtimie)
		tim = (struct ieee80211_tim_ie *)&dtimie[2];

	down(&hw_priv->conf_lock);
	{
		struct wsm_switch_channel channel;
		struct wsm_join join = {
			.mode = (bss->capability & WLAN_CAPABILITY_IBSS) ?
				WSM_JOIN_MODE_IBSS : WSM_JOIN_MODE_BSS,
			.preambleType = WSM_JOIN_PREAMBLE_SHORT,
			.probeForJoin = 1,
			/* dtimPeriod will be updated after association */
			.dtimPeriod = 1,
			.beaconInterval = bss->beacon_interval,
		};

		if (priv->if_id)
			join.flags |= WSM_FLAG_MAC_INSTANCE_1;
		else
			join.flags &= ~WSM_FLAG_MAC_INSTANCE_1;

		/* BT Coex related changes */
		if (hw_priv->is_BT_Present) {
			if (((hw_priv->conf_listen_interval * 100) %
					bss->beacon_interval) == 0)
				priv->listen_interval =
					((hw_priv->conf_listen_interval * 100) /
					bss->beacon_interval);
			else
				priv->listen_interval =
					((hw_priv->conf_listen_interval * 100) /
					bss->beacon_interval + 1);
		}

		if (tim && tim->dtim_period > 1) {
			join.dtimPeriod = tim->dtim_period;
			priv->join_dtim_period = tim->dtim_period;
		}
		priv->beacon_int = bss->beacon_interval;
		bes2600_info(BES2600_DBG_STA, "[STA] Join DTIM: %d, interval: %d\n",
				join.dtimPeriod, priv->beacon_int);

		hw_priv->is_go_thru_go_neg = false;
		join.channelNumber = hw_priv->channel->hw_value;

		/* basicRateSet will be updated after association.
		Currently these values are hardcoded */
		if (hw_priv->channel->band == NL80211_BAND_5GHZ) {
			join.band = WSM_PHY_BAND_5G;
			join.basicRateSet = 64; /*6 mbps*/
		}else{
			join.band = WSM_PHY_BAND_2_4G;
			join.basicRateSet = 7; /*1, 2, 5.5 mbps*/
		}
		memcpy(&join.bssid[0], bssid, sizeof(join.bssid));
		memcpy(&priv->join_bssid[0], bssid, sizeof(priv->join_bssid));

		if (ssidie) {
			join.ssidLength = ssidie[1];
			if (WARN_ON(join.ssidLength > sizeof(join.ssid)))
				join.ssidLength = sizeof(join.ssid);
			memcpy(&join.ssid[0], &ssidie[2], join.ssidLength);
			if(strstr(&join.ssid[0],"5.1.4"))
				msleep(200);
		}

		if (priv->vif->p2p) {
			join.flags |= WSM_JOIN_FLAGS_P2P_GO;
			join.flags |= (1 << 6);
			join.basicRateSet =
				bes2600_rate_mask_to_wsm(hw_priv, 0xFF0);
		}

		bes2600_pwr_set_busy_event(hw_priv, BES_PWR_LOCK_ON_JOIN);
		wsm_flush_tx(hw_priv);

		/* Queue unjoin if not associated in 3 sec. */
		queue_delayed_work(hw_priv->workqueue,
			&priv->join_timeout, 3 * HZ);

		bes2600_disable_listening(priv);

		//WARN_ON(wsm_reset(hw_priv, &reset, priv->if_id));
		WARN_ON(wsm_set_block_ack_policy(hw_priv,
			0, hw_priv->ba_tid_mask, priv->if_id));
		spin_lock_bh(&hw_priv->ba_lock);
		hw_priv->ba_ena = false;
		hw_priv->ba_cnt = 0;
		hw_priv->ba_acc = 0;
		hw_priv->ba_hist = 0;
		hw_priv->ba_cnt_rx = 0;
		hw_priv->ba_acc_rx = 0;
		spin_unlock_bh(&hw_priv->ba_lock);

		mgmt_policy.protectedMgmtEnable = 0;
		mgmt_policy.unprotectedMgmtFramesAllowed = 1;
		mgmt_policy.encryptionForAuthFrame = 1;
		wsm_set_protected_mgmt_policy(hw_priv, &mgmt_policy,
					      priv->if_id);

		/* need to switch channel before join */
		channel.channelMode = NL80211_CHAN_NO_HT << 4;
		channel.channelSwitchCount = 0;
		channel.newChannelNumber = conf->chandef.chan->hw_value;
		wsm_switch_channel(hw_priv, &channel,  priv->if_id);

		/* avoid lmac assert when wpa_supplicant connect to ap without scan */
		probe_tmp.skb = ieee80211_probereq_get(hw_priv->hw, priv->vif->addr, NULL, 0, 0);
		if (probe_tmp.skb) {
			wsm_set_template_frame(hw_priv, &probe_tmp, 0);
		}

		if (wsm_join(hw_priv, &join, priv->if_id)) {
			memset(&priv->join_bssid[0],
				0, sizeof(priv->join_bssid));
			bes2600_queue_remove(queue, hw_priv->pending_frame_id);
			cancel_delayed_work_sync(&priv->join_timeout);
			bes2600_pwr_clear_busy_event(priv->hw_priv, BES_PWR_LOCK_ON_JOIN);
		} else {
			/* Upload keys */
			bes2600_queue_requeue(queue, hw_priv->pending_frame_id,
						true);
			priv->join_status = BES2600_JOIN_STATUS_STA;
			atomic_set(&priv->connect_in_process, 1);

			/* Due to beacon filtering it is possible that the
			 * AP's beacon is not known for the mac80211 stack.
			 * Disable filtering temporary to make sure the stack
			 * receives at least one */
			priv->disable_beacon_filter = true;

		}
		bes2600_update_filtering(priv);
	}
	up(&hw_priv->conf_lock);
	if (bss)
		cfg80211_put_bss(hw_priv->hw->wiphy, bss);
	wsm_unlock_tx(hw_priv);
}

void bes2600_join_timeout(struct work_struct *work)
{
	struct bes2600_vif *priv =
		container_of(work, struct bes2600_vif, join_timeout.work);
	bes2600_info(BES2600_DBG_STA, "[WSM] Issue unjoin command (TMO).\n");
	bes2600_pwr_clear_busy_event(priv->hw_priv, BES_PWR_LOCK_ON_JOIN);
	atomic_set(&priv->connect_in_process, 0);
	wsm_lock_tx(priv->hw_priv);
	bes2600_unjoin_work(&priv->unjoin_work);
}

void bes2600_unjoin_work(struct work_struct *work)
{
	struct bes2600_vif *priv =
		container_of(work, struct bes2600_vif, unjoin_work);
	struct bes2600_common *hw_priv = cw12xx_vifpriv_to_hwpriv(priv);

	struct wsm_reset reset = {
		.reset_statistics = true,
	};
	bool is_htcapie = false;
	int i;
	struct bes2600_vif *tmp_priv;

	del_timer_sync(&hw_priv->ba_timer);
	down(&hw_priv->conf_lock);
	if (unlikely(atomic_read(&hw_priv->scan.in_progress)
		|| atomic_read(&priv->connect_in_process))) {
		if (priv->delayed_unjoin) {
			wiphy_dbg(priv->hw->wiphy,
				"%s: Delayed unjoin "
				"is already scheduled.\n",
				__func__);
			wsm_unlock_tx(hw_priv);
		} else {
			bes2600_info(BES2600_DBG_AP, "delay unjoin work, scan:%d connect:%d\n",
				atomic_read(&hw_priv->scan.in_progress), atomic_read(&priv->connect_in_process));
			priv->delayed_unjoin = true;
		}
		up(&hw_priv->conf_lock);
		return;
	}

	if (priv->join_status &&
			priv->join_status > BES2600_JOIN_STATUS_STA) {
		wiphy_err(priv->hw->wiphy,
				"%s: Unexpected: join status: %d\n",
				__func__, priv->join_status);
		BUG_ON(1);
	}
	if (priv->join_status) {
		cancel_work_sync(&priv->update_filtering_work);
		cancel_work_sync(&priv->set_beacon_wakeup_period_work);
		memset(&priv->join_bssid[0], 0, sizeof(priv->join_bssid));
		txrx_opt_timer_exit(hw_priv);
		bes2600_pwr_clear_busy_event(priv->hw_priv, BES_PWR_LOCK_ON_PS_ACTIVE);
		bes2600_pwr_clear_ap_lp_bad_mark(hw_priv);
		priv->join_status = BES2600_JOIN_STATUS_PASSIVE;

		/* Unjoin is a reset. */
		wsm_flush_tx(hw_priv);
		WARN_ON(wsm_keep_alive_period(hw_priv, 0, priv->if_id));
		WARN_ON(wsm_reset(hw_priv, &reset, priv->if_id));
		WARN_ON(wsm_set_output_power(hw_priv,
			hw_priv->output_power * 10, priv->if_id));
		priv->join_dtim_period = 0;
		priv->cipherType = 0;
		WARN_ON(bes2600_setup_mac_pvif(priv));
		bes2600_free_event_queue(hw_priv);
		cancel_work_sync(&hw_priv->event_handler);
		cancel_delayed_work_sync(&priv->connection_loss_work);
		WARN_ON(wsm_set_block_ack_policy(hw_priv,
			0, hw_priv->ba_tid_mask, priv->if_id));
		priv->disable_beacon_filter = false;
		bes2600_update_filtering(priv);
		priv->setbssparams_done = false;
		memset(&priv->association_mode, 0,
			sizeof(priv->association_mode));
		memset(&priv->bss_params, 0, sizeof(priv->bss_params));
		memset(&priv->firmware_ps_mode, 0,
			sizeof(priv->firmware_ps_mode));
		priv->htcap = false;
		bes2600_for_each_vif(hw_priv, tmp_priv, i) {
			if ((i == (CW12XX_MAX_VIFS - 1)) || !tmp_priv)
				continue;
			if ((tmp_priv->join_status == BES2600_JOIN_STATUS_STA) && tmp_priv->htcap)
				is_htcapie = true;
		}

		if (is_htcapie) {
			hw_priv->vif0_throttle = CW12XX_HOST_VIF0_11N_THROTTLE;
			hw_priv->vif1_throttle = CW12XX_HOST_VIF1_11N_THROTTLE;
			bes2600_info(BES2600_DBG_STA, "UNJOIN HTCAP 11N %d\n",hw_priv->vif0_throttle);
		} else {
			hw_priv->vif0_throttle = CW12XX_HOST_VIF0_11BG_THROTTLE;
			hw_priv->vif1_throttle = CW12XX_HOST_VIF1_11BG_THROTTLE;
			bes2600_info(BES2600_DBG_STA, "UNJOIN 11BG %d\n",hw_priv->vif0_throttle);
		}
		bes2600_info(BES2600_DBG_STA, "[STA] Unjoin.\n");
	}

	up(&hw_priv->conf_lock);
	wsm_unlock_tx(hw_priv);
}

int bes2600_enable_listening(struct bes2600_vif *priv,
				struct ieee80211_channel *chan)
{
	/* TODO:COMBO: Channel is common to HW currently in mac80211.
	Change the code below once channel is made per VIF */
	struct bes2600_common *hw_priv = cw12xx_vifpriv_to_hwpriv(priv);
	struct wsm_start start = {
		.mode = WSM_START_MODE_P2P_DEV | (priv->if_id ? (5 << 4) : 0),
		.band = (chan->band == NL80211_BAND_5GHZ) ?
				WSM_PHY_BAND_5G : WSM_PHY_BAND_2_4G,
		.channelNumber = chan->hw_value,
		.beaconInterval = 100,
		.DTIMPeriod = 1,
		.probeDelay = 0,
		.basicRateSet = 0x0F,
	};
	if(priv->if_id != 2) {
		//WARN_ON(priv->join_status > BES2600_JOIN_STATUS_MONITOR);
		return -EOPNOTSUPP;
	}
	if (priv->join_status == BES2600_JOIN_STATUS_MONITOR)
		return 0;
	if (priv->join_status == BES2600_JOIN_STATUS_PASSIVE)
		priv->join_status = BES2600_JOIN_STATUS_MONITOR;

	WARN_ON(priv->join_status > BES2600_JOIN_STATUS_MONITOR);
	bes2600_info(BES2600_DBG_STA, "bes2600_enable_listening if_id:%d\n", priv->if_id);
	return wsm_start(hw_priv, &start, CW12XX_GENERIC_IF_ID);
}

int bes2600_disable_listening(struct bes2600_vif *priv)
{
	int ret;
	struct wsm_reset reset = {
		.reset_statistics = true,
	};
	if(priv->if_id != 2) {
		WARN_ON(priv->join_status > BES2600_JOIN_STATUS_MONITOR);
        return 0;
	}
	priv->join_status = BES2600_JOIN_STATUS_PASSIVE;

	WARN_ON(priv->join_status > BES2600_JOIN_STATUS_MONITOR);

	if (priv->hw_priv->roc_if_id == -1)
		return 0;
	bes2600_info(BES2600_DBG_STA, "bes2600_disable_listening if_id:%d\n", priv->if_id);
	ret = wsm_reset(priv->hw_priv, &reset, CW12XX_GENERIC_IF_ID);
	return ret;
}

/* TODO:COMBO:UAPSD will be supported only on one interface */
int bes2600_set_uapsd_param(struct bes2600_vif *priv,
				const struct wsm_edca_params *arg)
{
	struct bes2600_common *hw_priv = cw12xx_vifpriv_to_hwpriv(priv);
	int ret;
	u16 uapsdFlags = 0;

	/* Here's the mapping AC [queue, bit]
	VO [0,3], VI [1, 2], BE [2, 1], BK [3, 0]*/

	if (arg->params[0].uapsdEnable)
		uapsdFlags |= 1 << 3;

	if (arg->params[1].uapsdEnable)
		uapsdFlags |= 1 << 2;

	if (arg->params[2].uapsdEnable)
		uapsdFlags |= 1 << 1;

	if (arg->params[3].uapsdEnable)
		uapsdFlags |= 1;

	/* Currently pseudo U-APSD operation is not supported, so setting
	* MinAutoTriggerInterval, MaxAutoTriggerInterval and
	* AutoTriggerStep to 0 */

	priv->uapsd_info.uapsdFlags = cpu_to_le16(uapsdFlags);
	priv->uapsd_info.minAutoTriggerInterval = 0;
	priv->uapsd_info.maxAutoTriggerInterval = 0;
	priv->uapsd_info.autoTriggerStep = 0;

	ret = wsm_set_uapsd_info(hw_priv, &priv->uapsd_info,
				 priv->if_id);
	return ret;
}

void bes2600_ba_work(struct work_struct *work)
{
	struct bes2600_common *hw_priv =
		container_of(work, struct bes2600_common, ba_work);
	u8 tx_ba_tid_mask;

	/* TODO:COMBO: reenable this part of code */
/*	if (priv->join_status != BES2600_JOIN_STATUS_STA)
		return;
	if (!priv->setbssparams_done)
		return;*/

	bes2600_info(BES2600_DBG_STA, "BA work****\n");
	spin_lock_bh(&hw_priv->ba_lock);
//	tx_ba_tid_mask = hw_priv->ba_ena ? hw_priv->ba_tid_mask : 0;
	tx_ba_tid_mask = hw_priv->ba_tid_mask;
	spin_unlock_bh(&hw_priv->ba_lock);

	wsm_lock_tx(hw_priv);

	WARN_ON(wsm_set_block_ack_policy(hw_priv,
		tx_ba_tid_mask, hw_priv->ba_tid_mask, -1)); /*TODO:COMBO*/

	wsm_unlock_tx(hw_priv);
}

void bes2600_ba_timer(struct timer_list *t)
{
	bool ba_ena;
	struct bes2600_common *hw_priv = from_timer(hw_priv, t, ba_timer);

	spin_lock_bh(&hw_priv->ba_lock);
	bes2600_debug_ba(hw_priv, hw_priv->ba_cnt, hw_priv->ba_acc,
			hw_priv->ba_cnt_rx, hw_priv->ba_acc_rx);

	if (atomic_read(&hw_priv->scan.in_progress)) {
		hw_priv->ba_cnt = 0;
		hw_priv->ba_acc = 0;
		hw_priv->ba_cnt_rx = 0;
		hw_priv->ba_acc_rx = 0;
		goto skip_statistic_update;
	}

	if (hw_priv->ba_cnt >= BES2600_BLOCK_ACK_CNT &&
		(hw_priv->ba_acc / hw_priv->ba_cnt >= BES2600_BLOCK_ACK_THLD ||
		(hw_priv->ba_cnt_rx >= BES2600_BLOCK_ACK_CNT &&
		hw_priv->ba_acc_rx / hw_priv->ba_cnt_rx >=
			BES2600_BLOCK_ACK_THLD)))
		ba_ena = true;
	else
		ba_ena = false;

	hw_priv->ba_cnt = 0;
	hw_priv->ba_acc = 0;
	hw_priv->ba_cnt_rx = 0;
	hw_priv->ba_acc_rx = 0;

	if (ba_ena != hw_priv->ba_ena) {
		if (ba_ena || ++hw_priv->ba_hist >= BES2600_BLOCK_ACK_HIST) {
			hw_priv->ba_ena = ba_ena;
			hw_priv->ba_hist = 0;
		}
	} else if (hw_priv->ba_hist)
		--hw_priv->ba_hist;

skip_statistic_update:
	spin_unlock_bh(&hw_priv->ba_lock);
}

int bes2600_vif_setup(struct bes2600_vif *priv)
{
	struct bes2600_common *hw_priv = priv->hw_priv;
	int ret = 0;

	/* Setup per vif workitems and locks */
	spin_lock_init(&priv->vif_lock);
	INIT_WORK(&priv->join_work, bes2600_join_work);
	INIT_DELAYED_WORK(&priv->join_timeout, bes2600_join_timeout);
	INIT_WORK(&priv->unjoin_work, bes2600_unjoin_work);
	INIT_WORK(&priv->wep_key_work, bes2600_wep_key_work);
	INIT_WORK(&priv->offchannel_work, bes2600_offchannel_work);
	INIT_DELAYED_WORK(&priv->bss_loss_work, bes2600_bss_loss_work);
	INIT_DELAYED_WORK(&priv->connection_loss_work,
			  bes2600_connection_loss_work);
	spin_lock_init(&priv->bss_loss_lock);
	INIT_WORK(&priv->tx_failure_work, bes2600_tx_failure_work);
	spin_lock_init(&priv->ps_state_lock);
	INIT_DELAYED_WORK(&priv->set_cts_work, bes2600_set_cts_work);
	INIT_WORK(&priv->set_tim_work, bes2600_set_tim_work);
	INIT_WORK(&priv->multicast_start_work, bes2600_multicast_start_work);
	INIT_WORK(&priv->multicast_stop_work, bes2600_multicast_stop_work);
	INIT_WORK(&priv->link_id_work, bes2600_link_id_work);
	INIT_DELAYED_WORK(&priv->link_id_gc_work, bes2600_link_id_gc_work);
	INIT_WORK(&priv->update_filtering_work, bes2600_update_filtering_work);
	INIT_DELAYED_WORK(&priv->pending_offchanneltx_work,
			bes2600_pending_offchanneltx_work);
	INIT_WORK(&priv->set_beacon_wakeup_period_work,
		bes2600_set_beacon_wakeup_period_work);
        INIT_WORK(&priv->ht_info_update_work, bes2600_ht_info_update_work);
	timer_setup(&priv->mcast_timeout, bes2600_mcast_timeout, 0);

	priv->setbssparams_done = false;
	priv->power_set_true = 0;
	priv->user_power_set_true = 0;
	priv->user_pm_mode = 0;
	ret = bes2600_debug_init_priv(hw_priv, priv);
	if (WARN_ON(ret))
		goto out;

	/* Initialising the broadcast filter */
	memset(priv->broadcast_filter.MacAddr, 0xFF, ETH_ALEN);
	priv->broadcast_filter.nummacaddr = 1;
	priv->broadcast_filter.address_mode = WSM_FILTER_ADDR_MODE_A1;
	priv->broadcast_filter.filter_mode = WSM_FILTER_ACTION_FILTER_OUT;
	priv->htcap = false;

	bes2600_info(BES2600_DBG_STA, "%s: enabling priv\n", __func__);
	atomic_set(&priv->enabled, 1);

	spin_lock(&hw_priv->vif_list_lock);
	hw_priv->if_id_slot |= BIT(priv->if_id);
	hw_priv->vif_list[priv->if_id] = priv->vif;
	atomic_inc(&hw_priv->num_vifs);
	spin_unlock(&hw_priv->vif_list_lock);
	atomic_set(&priv->connect_in_process, 0);

	if (priv->if_id < 2) {
		/* default EDCA */
		WSM_EDCA_SET(&priv->edca, 0, 0x0002, 0x0003, 0x0007,
				47, 0xc8, false);
		WSM_EDCA_SET(&priv->edca, 1, 0x0002, 0x0007, 0x000f,
				94, 0xc8, false);
			WSM_EDCA_SET(&priv->edca, 2, 0x0003, 0x000f, 0x03ff,
				0, 0xc8, false);
		WSM_EDCA_SET(&priv->edca, 3, 0x0007, 0x000f, 0x03ff,
				0, 0xc8, false);
		ret = wsm_set_edca_params(hw_priv, &priv->edca, priv->if_id);
		if (WARN_ON(ret))
			goto out;

		ret = bes2600_set_uapsd_param(priv, &priv->edca);
		if (WARN_ON(ret))
			goto out;

		memset(priv->bssid, ~0, ETH_ALEN);
		priv->wep_default_key_id = -1;
		priv->cipherType = 0;
		priv->cqm_link_loss_count = 100;
		priv->cqm_beacon_loss_count = 50;

		/* Temporary configuration - beacon filter table */
		__bes2600_bf_configure(priv);
	}
out:
	return ret;
}

int bes2600_setup_mac_pvif(struct bes2600_vif *priv)
{
	int ret = 0;
	/* NOTE: There is a bug in FW: it reports signal
	* as RSSI if RSSI subscription is enabled.
	* It's not enough to set WSM_RCPI_RSSI_USE_RSSI. */
	/* NOTE2: RSSI based reports have been switched to RCPI, since
	* FW has a bug and RSSI reported values are not stable,
	* what can leads to signal level oscilations in user-end applications */
	struct wsm_rcpi_rssi_threshold threshold = {
		.rssiRcpiMode = WSM_RCPI_RSSI_THRESHOLD_ENABLE |
		WSM_RCPI_RSSI_DONT_USE_UPPER |
		WSM_RCPI_RSSI_DONT_USE_LOWER,
		.rollingAverageCount = 16,
	};

	/* Remember the decission here to make sure, we will handle
	 * the RCPI/RSSI value correctly on WSM_EVENT_RCPI_RSS */
	if (threshold.rssiRcpiMode & WSM_RCPI_RSSI_USE_RSSI)
		priv->cqm_use_rssi = true;


	/* Configure RSSI/SCPI reporting as RSSI. */
	ret = wsm_set_rcpi_rssi_threshold(priv->hw_priv, &threshold,
					priv->if_id ? 0 : 0);
	return ret;
}

void bes2600_rem_chan_timeout(struct work_struct *work)
{
	struct bes2600_common *hw_priv =
		container_of(work, struct bes2600_common, rem_chan_timeout.work);
	int ret, if_id;
	struct bes2600_vif *priv;

	if (atomic_read(&hw_priv->remain_on_channel) == 0) {
		return;
	}
	ieee80211_remain_on_channel_expired(hw_priv->hw);

	down(&hw_priv->conf_lock);
	if_id = hw_priv->roc_if_id;
	bes2600_dbg(BES2600_DBG_ROC, "ROC TO IN %d\n", if_id);
	priv = __cw12xx_hwpriv_to_vifpriv(hw_priv, if_id);
	ret = WARN_ON(__bes2600_flush(hw_priv, false, if_id));

	if (!ret) {
		wsm_unlock_tx(hw_priv);
		bes2600_disable_listening(priv);
	}
	atomic_set(&hw_priv->remain_on_channel, 0);
	hw_priv->roc_if_id = -1;

	bes2600_dbg(BES2600_DBG_ROC, "ROC TO OUT %d\n", if_id);

	up(&hw_priv->conf_lock);
	up(&hw_priv->scan.lock);
	bes2600_pwr_clear_busy_event(hw_priv, BES_PWR_LOCK_ON_ROC);
}

void bes2600_dynamic_opt_txrx_work(struct work_struct *work)
{
	struct bes2600_common *hw_priv =
		container_of(work, struct bes2600_common, dynamic_opt_txrx_work);
	struct bes2600_vif *priv = __cw12xx_hwpriv_to_vifpriv(hw_priv, 0);
	bes2600_dynamic_opt_rxtx(hw_priv,priv, 0);
	bes2600_dbg(BES2600_DBG_STA, "bes2600_dynamic_opt_txrx_work called\n");
}


const u8 *bes2600_get_ie(u8 *start, size_t len, u8 ie)
{
	u8 *end, *pos;

	pos = start;
	if (pos == NULL)
		return NULL;
	end = pos + len;

	while (pos + 1 < end) {
		if (pos + 2 + pos[1] > end)
			break;
		if (pos[0] == ie)
			return pos;
		pos += 2 + pos[1];
	}

	return NULL;
}

/**
 * bes2600_set_macaddrfilter -called when tesmode command
 * is for setting mac address filter
 *
 * @hw: the hardware
 * @data: incoming data
 *
 * Returns: 0 on success or non zero value on failure
 */
int bes2600_set_macaddrfilter(struct bes2600_common *hw_priv, struct bes2600_vif *priv, u8 *data)
{
	struct wsm_mac_addr_filter *mac_addr_filter =  NULL;
	struct wsm_mac_addr_info *addr_info = NULL;
	u8 action_mode = 0, no_of_mac_addr = 0, i = 0;
	int ret = 0;
	u16 macaddrfiltersize = 0;

	/* Retrieving Action Mode */
	action_mode = data[0];
	/* Retrieving number of address entries */
	no_of_mac_addr = data[1];

	addr_info = (struct wsm_mac_addr_info *)&data[2];

	/* Computing sizeof Mac addr filter */
	macaddrfiltersize =  sizeof(*mac_addr_filter) + \
			(no_of_mac_addr * sizeof(struct wsm_mac_addr_info));

	mac_addr_filter = kzalloc(macaddrfiltersize, GFP_KERNEL);
	if (!mac_addr_filter) {
		ret = -ENOMEM;
		goto exit_p;
	}
	mac_addr_filter->action_mode = action_mode;
	mac_addr_filter->numfilter = no_of_mac_addr;

	for (i = 0; i < no_of_mac_addr; i++) {
		mac_addr_filter->macaddrfilter[i].address_mode = \
						addr_info[i].address_mode;
		memcpy(mac_addr_filter->macaddrfilter[i].MacAddr, \
				addr_info[i].MacAddr , ETH_ALEN);
		mac_addr_filter->macaddrfilter[i].filter_mode = \
						addr_info[i].filter_mode;
	}
	ret = WARN_ON(wsm_write_mib(hw_priv, WSM_MIB_ID_MAC_ADDR_FILTER, \
					 mac_addr_filter, macaddrfiltersize, priv->if_id));

	kfree(mac_addr_filter);
exit_p:
	return ret;
}

/**
 * bes2600_set_data_filter -configure data filter in device
*
 * @hw: the hardware
 * @vif: vif
 * @data: incoming data
 * @len: incoming data length
 *
 */
void bes2600_set_data_filter(struct ieee80211_hw *hw,
			   struct ieee80211_vif *vif,
			   void *data, int len)
{
	int ret = 0;
	//struct bes2600_vif *priv = cw12xx_get_vif_from_ieee80211(vif);
	int filter_id;

	if (!data) {
		ret = -EINVAL;
		goto exit_p;
	}
	filter_id=*((enum bes2600_data_filterid*)data);

	switch (filter_id) {
	default:
		ret = -EINVAL;
		break;
	}
exit_p:

	 return ;
}

u32 bes2600_bh_get_encry_hdr_len(u32 cipherType)
{
	u32 encrypthdr = 0;

	switch (cipherType) {
	case WLAN_CIPHER_SUITE_WEP40:
	case WLAN_CIPHER_SUITE_WEP104:
		encrypthdr = WEP_ENCRYPT_HDR_SIZE;
		break;
	case WLAN_CIPHER_SUITE_TKIP:
		encrypthdr = WPA_ENCRYPT_HDR_SIZE;
		break;
	case WLAN_CIPHER_SUITE_CCMP:
		encrypthdr = WPA2_ENCRYPT_HDR_SIZE;
		break;
	case WLAN_CIPHER_SUITE_SMS4:
		encrypthdr = WAPI_ENCRYPT_HDR_SIZE;
		break;
	default:
		encrypthdr = 0;
		break;
	}

	return encrypthdr;
}

/**
 * bes2600_set_arpreply -called for creating and
 * configuring arp response template frame
 *
 * @hw: the hardware
 *
 * Returns: 0 on success or non zero value on failure
 */
int bes2600_set_arpreply(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
	struct bes2600_vif *priv = cw12xx_get_vif_from_ieee80211(vif);
	struct bes2600_common *hw_priv = (struct bes2600_common *)hw->priv;
        u32 framehdrlen, encrypthdr, encrypttailsize, framebdylen = 0;
        bool encrypt = false;
        int ret = 0;
        u8 *template_frame = NULL;
        struct ieee80211_hdr_3addr *dot11hdr = NULL;
        struct ieee80211_snap_hdr *snaphdr = NULL;
        struct arphdr *arp_hdr = NULL;

        template_frame = kzalloc(MAX_ARP_REPLY_TEMPLATE_SIZE, GFP_ATOMIC);
        if (!template_frame) {
                bes2600_err(BES2600_DBG_STA, "[STA] Template frame memory failed\n");
                ret = -ENOMEM;
                goto exit_p;
        }
        dot11hdr = (struct ieee80211_hdr_3addr *)&template_frame[4];

        framehdrlen = sizeof(*dot11hdr);
        if ((priv->vif->type == NL80211_IFTYPE_AP) && priv->vif->p2p)
                priv->cipherType = WLAN_CIPHER_SUITE_CCMP;
        switch (priv->cipherType) {

        case WLAN_CIPHER_SUITE_WEP40:
        case WLAN_CIPHER_SUITE_WEP104:
                bes2600_dbg(BES2600_DBG_STA, "[STA] WEP\n");
                encrypthdr = WEP_ENCRYPT_HDR_SIZE;
                encrypttailsize = WEP_ENCRYPT_TAIL_SIZE;
                encrypt = 1;
                break;


        case WLAN_CIPHER_SUITE_TKIP:
                bes2600_dbg(BES2600_DBG_STA, "[STA] WPA\n");
                encrypthdr = WPA_ENCRYPT_HDR_SIZE;
                encrypttailsize = WPA_ENCRYPT_TAIL_SIZE;
                encrypt = 1;
                break;

        case WLAN_CIPHER_SUITE_CCMP:
                bes2600_dbg(BES2600_DBG_STA, "[STA] WPA2\n");
                encrypthdr = WPA2_ENCRYPT_HDR_SIZE;
                encrypttailsize = WPA2_ENCRYPT_TAIL_SIZE;
                encrypt = 1;
                break;

        case WLAN_CIPHER_SUITE_SMS4:
                bes2600_dbg(BES2600_DBG_STA, "[STA] WAPI\n");
                encrypthdr = WAPI_ENCRYPT_HDR_SIZE;
                encrypttailsize = WAPI_ENCRYPT_TAIL_SIZE;
                encrypt = 1;
                break;

        default:
                encrypthdr = 0;
                encrypttailsize = 0;
                encrypt = 0;
                break;
        }

        framehdrlen += encrypthdr;

        /* Filling the 802.11 Hdr */
        dot11hdr->frame_control = cpu_to_le16(IEEE80211_FTYPE_DATA);
        if (priv->vif->type == NL80211_IFTYPE_STATION)
                dot11hdr->frame_control |= cpu_to_le16(IEEE80211_FCTL_TODS);
        else
                dot11hdr->frame_control |= cpu_to_le16(IEEE80211_FCTL_FROMDS);

        if (encrypt)
                dot11hdr->frame_control |= cpu_to_le16(IEEE80211_FCTL_WEP);

        if (priv->vif->bss_conf.qos) {
                bes2600_dbg(BES2600_DBG_STA, "[STA] QOS Enabled\n");
                dot11hdr->frame_control |= cpu_to_le16(IEEE80211_QOS_DATAGRP);
                 *(u16 *)(dot11hdr + 1) = 0x0;
                 framehdrlen += 2;
        } else {
                dot11hdr->frame_control |= cpu_to_le16(IEEE80211_STYPE_DATA);
        }

        memcpy(dot11hdr->addr1, priv->vif->bss_conf.bssid, ETH_ALEN);
        memcpy(dot11hdr->addr2, priv->vif->addr, ETH_ALEN);
        memcpy(dot11hdr->addr3, priv->vif->bss_conf.bssid, ETH_ALEN);

        /* Filling the LLC/SNAP Hdr */
        snaphdr = (struct ieee80211_snap_hdr *)((u8 *)dot11hdr + framehdrlen);
        memcpy(snaphdr, (struct ieee80211_snap_hdr *)rfc1042_header, \
                sizeof(*snaphdr));
        *(u16 *)(++snaphdr) = cpu_to_be16(ETH_P_ARP);
        /* Updating the framebdylen with snaphdr and LLC hdr size */
        framebdylen = sizeof(*snaphdr) + 2;

        /* Filling the ARP Reply Payload */
        arp_hdr = (struct arphdr *)((u8 *)dot11hdr + framehdrlen + framebdylen);
        arp_hdr->ar_hrd = cpu_to_be16(ARPHRD_ETHER);
        arp_hdr->ar_pro = cpu_to_be16(ETH_P_IP);
        arp_hdr->ar_hln = ETH_ALEN;
        arp_hdr->ar_pln = 4;
        arp_hdr->ar_op = cpu_to_be16(ARPOP_REPLY);

        /* Updating the frmbdylen with Arp Reply Hdr and Arp payload size(20) */
        framebdylen += sizeof(*arp_hdr) + 20;

        /* Updating the framebdylen with Encryption Tail Size */
        framebdylen += encrypttailsize;

        /* Filling the Template Frame Hdr */
        template_frame[0] = WSM_FRAME_TYPE_ARP_REPLY; /* Template frame type */
        template_frame[1] = 0xFF; /* Rate to be fixed */
        ((u16 *)&template_frame[2])[0] = framehdrlen + framebdylen;

        ret = WARN_ON(wsm_write_mib(hw_priv, WSM_MIB_ID_TEMPLATE_FRAME, \
                                template_frame, (framehdrlen+framebdylen+4), priv->if_id));
        kfree(template_frame);
exit_p:
        return ret;
}
