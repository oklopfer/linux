/*
 * Scan implementation for BES2600 mac80211 drivers
 *
 * Copyright (c) 2022, Bestechnic
 * Author:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/sched.h>
#include "bes2600.h"
#include "scan.h"
#include "sta.h"
#include "pm.h"
#include "epta_request.h"
#include "bes_pwr.h"

static void bes2600_scan_restart_delayed(struct bes2600_vif *priv);

static void bes2600_remove_wps_p2p_ie(struct wsm_template_frame *frame)
{
	u8 *ies;
	u32 ies_len;
	u32 ie_len;
	u32 p2p_ie_len = 0;
	u32 wps_ie_len = 0;

	ies = &frame->skb->data[sizeof(struct ieee80211_hdr_3addr)];
	ies_len = frame->skb->len - sizeof(struct ieee80211_hdr_3addr);

	while (ies_len >= 6) {
		ie_len = ies[1] + 2;
		if ((ies[0] == WLAN_EID_VENDOR_SPECIFIC)
			&& (ies[2] == 0x00 && ies[3] == 0x50 && ies[4] == 0xf2 && ies[5] == 0x04)) {
			wps_ie_len = ie_len;
			memmove(ies, ies + ie_len, ies_len);
			ies_len -= ie_len;

		}
		else if ((ies[0] == WLAN_EID_VENDOR_SPECIFIC) &&
			(ies[2] == 0x50 && ies[3] == 0x6f && ies[4] == 0x9a && ies[5] == 0x09)) {
			p2p_ie_len = ie_len;
			memmove(ies, ies + ie_len, ies_len);
			ies_len -= ie_len;
		} else {
			ies += ie_len;
			ies_len -= ie_len;
		}
	}

	if (p2p_ie_len || wps_ie_len) {
		skb_trim(frame->skb, frame->skb->len - (p2p_ie_len + wps_ie_len));
	}
}

static int bes2600_scan_get_first_active_if(struct bes2600_common *hw_priv)
{
	int i = 0;
	struct bes2600_vif *vif;

	bes2600_for_each_vif(hw_priv, vif, i) {
		if (vif->join_status > BES2600_JOIN_STATUS_PASSIVE)
			return i;
	}

	return -1;
}

static int bes2600_scan_start(struct bes2600_vif *priv, struct wsm_scan *scan)
{
	int ret, i;
	int tmo = 5000;
	struct bes2600_common *hw_priv = cw12xx_vifpriv_to_hwpriv(priv);


	if (hw_priv->scan_switch_if_id == -1 &&
		hw_priv->ht_info.channel_type > NL80211_CHAN_HT20 &&
		priv->if_id >= 0) {
		hw_priv->scan_switch_if_id = bes2600_scan_get_first_active_if(hw_priv);
		if(hw_priv->scan_switch_if_id >= 0) {
			struct wsm_switch_channel channel;
			channel.channelMode = 0 << 4;
			channel.channelSwitchCount = 0;
			channel.newChannelNumber = hw_priv->channel->hw_value;
			wsm_switch_channel(hw_priv, &channel, hw_priv->scan_switch_if_id);
			bes2600_info(BES2600_DBG_SCAN, "scan start channel type %d num %d\n", hw_priv->ht_info.channel_type, channel.newChannelNumber);
		}
	}
	for (i = 0; i < scan->numOfChannels; ++i)
		tmo += scan->ch[i].maxChannelTime + 10;
	atomic_set(&hw_priv->scan.in_progress, 1);
	atomic_set(&hw_priv->recent_scan, 1);
	queue_delayed_work(hw_priv->workqueue, &hw_priv->scan.timeout,
		tmo * HZ / 1000);
	ret = wsm_scan(hw_priv, scan, 0);
	if (unlikely(ret)) {
		atomic_set(&hw_priv->scan.in_progress, 0);
		cancel_delayed_work_sync(&hw_priv->scan.timeout);
		bes2600_scan_restart_delayed(priv);
	}
	return ret;
}

int bes2600_hw_scan(struct ieee80211_hw *hw,
		   struct ieee80211_vif *vif,
		   struct ieee80211_scan_request *hw_req)
{
	struct bes2600_common *hw_priv = hw->priv;
	struct bes2600_vif *priv = cw12xx_get_vif_from_ieee80211(vif);
	struct cfg80211_scan_request *req = &hw_req->req;
	struct wsm_template_frame frame = {
		.frame_type = WSM_FRAME_TYPE_PROBE_REQUEST,
	};
	int i;
	/* Scan when P2P_GO corrupt firmware MiniAP mode */
	if (priv->join_status == BES2600_JOIN_STATUS_AP)
		return -EOPNOTSUPP;
	if (req->n_ssids == 1 && !req->ssids[0].ssid_len)
		req->n_ssids = 0;

	wiphy_dbg(hw->wiphy, "[SCAN] Scan request for %d SSIDs.\n",
		req->n_ssids);

	if (req->n_ssids > hw->wiphy->max_scan_ssids)
		return -EINVAL;

	bes2600_pwr_set_busy_event(hw_priv, BES_PWR_LOCK_ON_SCAN);

	frame.skb = ieee80211_probereq_get(hw, priv->vif->addr, NULL, 0,
		req->ie_len);
	if (!frame.skb)
		return -ENOMEM;

	if (req->ie_len)
		skb_put_data(frame.skb, req->ie, req->ie_len);

	/* will be unlocked in bes2600_scan_work() */
	down(&hw_priv->scan.lock);
	down(&hw_priv->conf_lock);

	if (frame.skb) {
		int ret;
		//if (priv->if_id == 0)
		//	bes2600_remove_wps_p2p_ie(&frame);
		ret = wsm_set_template_frame(hw_priv, &frame, 0);
		if (ret) {
			up(&hw_priv->conf_lock);
			up(&hw_priv->scan.lock);
			dev_kfree_skb(frame.skb);
			return ret;
		}
	}

	wsm_vif_lock_tx(priv);

	BUG_ON(hw_priv->scan.req);
	hw_priv->scan.req = req;
	hw_priv->scan.n_ssids = 0;
	hw_priv->scan.status = 0;
	hw_priv->scan.begin = &req->channels[0];
	hw_priv->scan.curr = hw_priv->scan.begin;
	hw_priv->scan.end = &req->channels[req->n_channels];
	hw_priv->scan.output_power = hw_priv->output_power;
	hw_priv->scan.if_id = priv->if_id;
	/* TODO:COMBO: Populate BIT4 in scanflags to decide on which MAC
	 * address the SCAN request will be sent */
	bes2600_info(BES2600_DBG_SCAN, "%s %d if_id:%d,num_channel:%d.\n", __func__, __LINE__, priv->if_id, req->n_channels);

	for (i = 0; i < req->n_ssids; ++i) {
		struct wsm_ssid *dst =
			&hw_priv->scan.ssids[hw_priv->scan.n_ssids];
		BUG_ON(req->ssids[i].ssid_len > sizeof(dst->ssid));
		memcpy(&dst->ssid[0], req->ssids[i].ssid,
			sizeof(dst->ssid));
		dst->length = req->ssids[i].ssid_len;
		++hw_priv->scan.n_ssids;
	}

	up(&hw_priv->conf_lock);

	if (frame.skb)
		dev_kfree_skb(frame.skb);
	bwifi_change_current_status(hw_priv, BWIFI_STATUS_SCANNING);
	queue_work(hw_priv->workqueue, &hw_priv->scan.work);

	return 0;
}

void bes2600_scan_work(struct work_struct *work)
{
	struct bes2600_common *hw_priv = container_of(work,
						struct bes2600_common,
						scan.work);
	struct bes2600_vif *priv, *vif;
	struct ieee80211_channel **it;
	struct wsm_scan scan = {
		.scanType = WSM_SCAN_TYPE_FOREGROUND,
		.scanFlags = 0, /* TODO:COMBO */
		//.scanFlags = WSM_SCAN_FLAG_SPLIT_METHOD, /* TODO:COMBO */
	};
	bool first_run;
	int i;
	const u32 ProbeRequestTime  = 2;
	const u32 ChannelRemainTime = 15;
	u32 minChannelTime;
	u32 maxChannelTime;
	priv = __cw12xx_hwpriv_to_vifpriv(hw_priv, hw_priv->scan.if_id);

	/*TODO: COMBO: introduce locking so vif is not removed in meanwhile */
	if (!priv) {
		wiphy_warn(hw_priv->hw->wiphy, "[SCAN] interface removed, "
			   "ignoring scan work\n");
		return;
	}

	if (priv->if_id)
		scan.scanFlags |= WSM_FLAG_MAC_INSTANCE_1;
	else
		scan.scanFlags &= ~WSM_FLAG_MAC_INSTANCE_1;

	bes2600_for_each_vif(hw_priv, vif, i) {
		if ((i == (CW12XX_MAX_VIFS - 1)) || !vif)
			continue;
		if (vif->bss_loss_status > BES2600_BSS_LOSS_NONE)
			scan.scanFlags |= WSM_SCAN_FLAG_FORCE_BACKGROUND;
	}
	first_run = hw_priv->scan.begin == hw_priv->scan.curr &&
			hw_priv->scan.begin != hw_priv->scan.end;

	if (first_run) {
		/* Firmware gets crazy if scan request is sent
		 * when STA is joined but not yet associated.
		 * Force unjoin in this case. */
		if (cancel_delayed_work_sync(&priv->join_timeout) > 0) {
			bes2600_join_timeout(&priv->join_timeout.work);
		}
	}
	down(&hw_priv->conf_lock);
	if (first_run) {
		if (priv->join_status == BES2600_JOIN_STATUS_MONITOR) {
			/* FW bug: driver has to restart p2p-dev mode
			 * after scan */
			bes2600_disable_listening(priv);
		}
	}

	if (!hw_priv->scan.req || (hw_priv->scan.curr == hw_priv->scan.end)) {
		struct cfg80211_scan_info info = {
			.aborted = hw_priv->scan.status ? 1 : 0,
		};

		if (hw_priv->scan.output_power != hw_priv->output_power)
		/* TODO:COMBO: Change when mac80211 implementation
		 * is available for output power also */
			WARN_ON(wsm_set_output_power(hw_priv,
				hw_priv->output_power * 10,
				priv->if_id ? 0 : 0));
		if (hw_priv->scan.status < 0)
			wiphy_info(priv->hw->wiphy,
					"[SCAN] Scan failed (%d).\n",
					hw_priv->scan.status);
		else if (hw_priv->scan.req)
			wiphy_info(priv->hw->wiphy,
					"[SCAN] Scan completed.\n");
		else
			wiphy_info(priv->hw->wiphy,
					"[SCAN] Scan canceled.\n");

		if (priv->join_status == BES2600_JOIN_STATUS_STA) {
			if (hw_priv->channel->band != NL80211_BAND_2GHZ)
				bwifi_change_current_status(hw_priv, BWIFI_STATUS_GOT_IP_5G);
			else
				bwifi_change_current_status(hw_priv, BWIFI_STATUS_GOT_IP);
		} else {
			bwifi_change_current_status(hw_priv, BWIFI_STATUS_IDLE);
		}
		bes2600_info(BES2600_DBG_SCAN, "%s %d %d.", __func__, __LINE__, hw_priv->ht_info.channel_type);
		/* switch to previous channel and bw mode after scan done */
		if (hw_priv->scan_switch_if_id >= 0) {
			struct wsm_switch_channel channel;
			channel.channelMode = hw_priv->ht_info.channel_type << 4;
			channel.channelSwitchCount = 0;
			channel.newChannelNumber = hw_priv->channel->hw_value;
			wsm_switch_channel(hw_priv, &channel, hw_priv->scan_switch_if_id);
			hw_priv->scan_switch_if_id = -1;
			bes2600_info(BES2600_DBG_SCAN, "scan done channel type %d num %d\n", hw_priv->ht_info.channel_type, channel.newChannelNumber);
		}

		hw_priv->scan.req = NULL;
		bes2600_scan_restart_delayed(priv);
		wsm_unlock_tx(hw_priv);
		up(&hw_priv->conf_lock);
		bes2600_pwr_clear_busy_event(hw_priv, BES_PWR_LOCK_ON_SCAN);
		ieee80211_scan_completed(hw_priv->hw, &info);
		up(&hw_priv->scan.lock);
		return;
	} else {
		struct ieee80211_channel *first = *hw_priv->scan.curr;
		for (it = hw_priv->scan.curr + 1, i = 1;
		     it != hw_priv->scan.end &&
				i < WSM_SCAN_MAX_NUM_OF_CHANNELS;
		     ++it, ++i) {
			if ((*it)->band != first->band)
				break;
			// Doen't split scan req in case of EPTA error after scan req
			// if (((*it)->flags ^ first->flags) &
			// 		IEEE80211_CHAN_NO_IR)
			// 	break;
			// if (!(first->flags & IEEE80211_CHAN_NO_IR) &&
			// 	(*it)->max_power != first->max_power)
			// 	break;
		}
		scan.band = first->band;

		if (hw_priv->scan.req->no_cck)
			scan.maxTransmitRate = WSM_TRANSMIT_RATE_6;
		else
			scan.maxTransmitRate = WSM_TRANSMIT_RATE_1;
		/* TODO: Is it optimal? */
		scan.numOfProbeRequests =
			(first->flags & IEEE80211_CHAN_NO_IR) ? 0 : 2;
		scan.numOfSSIDs = hw_priv->scan.n_ssids;
		scan.ssids = &hw_priv->scan.ssids[0];
		scan.numOfChannels = it - hw_priv->scan.curr;
		/* TODO: Is it optimal? */
		scan.probeDelay = 100;
		/* It is not stated in WSM specification, however
		 * FW team says that driver may not use FG scan
		 * when joined. */
		if (priv->join_status == BES2600_JOIN_STATUS_STA) {
			scan.scanType = WSM_SCAN_TYPE_BACKGROUND;
			scan.scanFlags = WSM_SCAN_FLAG_FORCE_BACKGROUND;
		}
		scan.ch = kzalloc((it - hw_priv->scan.curr) *
			sizeof(struct wsm_scan_ch), GFP_KERNEL);
		if (!scan.ch) {
			hw_priv->scan.status = -ENOMEM;
			goto fail;
		}
		maxChannelTime = (scan.numOfSSIDs * scan.numOfProbeRequests *
			ProbeRequestTime) + ChannelRemainTime;
		maxChannelTime = (maxChannelTime < 35) ? 35 : maxChannelTime;

		if (scan.band == NL80211_BAND_2GHZ) {
			coex_calc_wifi_scan_time(&minChannelTime, &maxChannelTime);
		} else {
			minChannelTime = 90;
			maxChannelTime = 90;
		}

		for (i = 0; i < scan.numOfChannels; ++i) {
			scan.ch[i].number = hw_priv->scan.curr[i]->hw_value;
			scan.ch[i].minChannelTime = minChannelTime;
			scan.ch[i].maxChannelTime = maxChannelTime;
		}
		if (!(first->flags & IEEE80211_CHAN_NO_IR) &&
			hw_priv->scan.output_power != first->max_power) {
			hw_priv->scan.output_power = first->max_power;
			/* TODO:COMBO: Change after mac80211 implementation
		 	* complete */
			WARN_ON(wsm_set_output_power(hw_priv,
					hw_priv->scan.output_power * 10,
					priv->if_id ? 0 : 0));
		}
		hw_priv->scan.status = bes2600_scan_start(priv, &scan);
		kfree(scan.ch);
		if (WARN_ON(hw_priv->scan.status))
			goto fail;
		hw_priv->scan.curr = it;
	}
	up(&hw_priv->conf_lock);
	return;

fail:
	hw_priv->scan.curr = hw_priv->scan.end;
	up(&hw_priv->conf_lock);
	queue_work(hw_priv->workqueue, &hw_priv->scan.work);
	return;
}

static void bes2600_scan_restart_delayed(struct bes2600_vif *priv)
{
	struct bes2600_common *hw_priv = cw12xx_vifpriv_to_hwpriv(priv);

	if (priv->delayed_link_loss) {
		int tmo = priv->cqm_beacon_loss_count;

		if (hw_priv->scan.direct_probe)
			tmo = 0;

		priv->delayed_link_loss = 0;
		/* Restart beacon loss timer and requeue
		   BSS loss work. */
		wiphy_dbg(priv->hw->wiphy,
				"[CQM] Requeue BSS loss in %d "
				"beacons.\n", tmo);
		spin_lock(&priv->bss_loss_lock);
		priv->bss_loss_status = BES2600_BSS_LOSS_NONE;
		spin_unlock(&priv->bss_loss_lock);
		cancel_delayed_work_sync(&priv->bss_loss_work);
		queue_delayed_work(hw_priv->workqueue,
				&priv->bss_loss_work,
				tmo * HZ / 10);
	}

	/* FW bug: driver has to restart p2p-dev mode after scan. */
	if (priv->join_status == BES2600_JOIN_STATUS_MONITOR) {
		/*bes2600_enable_listening(priv);*/
		// WARN_ON(1);
		bes2600_dbg(BES2600_DBG_SCAN, "scan complete join_status is monitor");
		bes2600_update_filtering(priv);
	}

	if (priv->delayed_unjoin) {
		priv->delayed_unjoin = false;
		if (queue_work(hw_priv->workqueue, &priv->unjoin_work) <= 0)
			wsm_unlock_tx(hw_priv);
	}
}

static void bes2600_scan_complete(struct bes2600_common *hw_priv, int if_id)
{
	struct bes2600_vif *priv;
	atomic_xchg(&hw_priv->recent_scan, 0);

	if (hw_priv->scan.direct_probe) {
		down(&hw_priv->conf_lock);
		priv = __cw12xx_hwpriv_to_vifpriv(hw_priv, if_id);
		if (priv) {
			wiphy_dbg(priv->hw->wiphy, "[SCAN] Direct probe "
				  "complete.\n");
			bes2600_scan_restart_delayed(priv);
		} else {
			wiphy_dbg(priv->hw->wiphy, "[SCAN] Direct probe "
				  "complete without interface!\n");
		}
		up(&hw_priv->conf_lock);
		hw_priv->scan.direct_probe = 0;
		up(&hw_priv->scan.lock);
		wsm_unlock_tx(hw_priv);
	} else {
		bes2600_scan_work(&hw_priv->scan.work);
	}
}

void bes2600_scan_complete_cb(struct bes2600_common *hw_priv,
				struct wsm_scan_complete *arg)
{
	struct bes2600_vif *priv = cw12xx_hwpriv_to_vifpriv(hw_priv,
					hw_priv->scan.if_id);

	if (unlikely(!priv))
		return;

	if (unlikely(priv->mode == NL80211_IFTYPE_UNSPECIFIED)) {
		/* STA is stopped. */
		spin_unlock(&priv->vif_lock);
		return;
	}
	spin_unlock(&priv->vif_lock);

	// recover EPTA timer after scan wsm msg complete, in case of epta state error
	// bwifi_change_current_status(hw_priv, BWIFI_STATUS_SCANNING_COMP);
	wiphy_info(hw_priv->hw->wiphy, "bes2600_scan_complete_cb status: %u", arg->status);

	if(hw_priv->scan.status == -ETIMEDOUT)
		wiphy_warn(hw_priv->hw->wiphy,
			"Scan timeout already occured. Don't cancel work");
	if ((hw_priv->scan.status != -ETIMEDOUT) &&
		(cancel_delayed_work_sync(&hw_priv->scan.timeout) > 0)) {
		hw_priv->scan.status = 1;
		queue_delayed_work(hw_priv->workqueue,
				&hw_priv->scan.timeout, 0);
	}
}

void bes2600_scan_timeout(struct work_struct *work)
{
	struct bes2600_common *hw_priv =
		container_of(work, struct bes2600_common, scan.timeout.work);

	if (likely(atomic_xchg(&hw_priv->scan.in_progress, 0))) {
		if (hw_priv->scan.status > 0)
			hw_priv->scan.status = 0;
		else if (!hw_priv->scan.status) {
			wiphy_warn(hw_priv->hw->wiphy,
				"Timeout waiting for scan "
				"complete notification.\n");
			hw_priv->scan.status = -ETIMEDOUT;
			hw_priv->scan.curr = hw_priv->scan.end;
			WARN_ON(wsm_stop_scan(hw_priv,
						hw_priv->scan.if_id ? 1 : 0));
		}
		bes2600_scan_complete(hw_priv, hw_priv->scan.if_id);
	}
}

void bes2600_cancel_hw_scan(struct ieee80211_hw *hw, struct ieee80211_vif *vif)
{
	struct bes2600_vif *priv = cw12xx_get_vif_from_ieee80211(vif);
	struct bes2600_common *hw_priv = cw12xx_vifpriv_to_hwpriv(priv);

	if(hw_priv->scan.if_id == priv->if_id) {
		bes2600_dbg(BES2600_DBG_SCAN, "cancel hw_scan on intf:%d\n", priv->if_id);

		down(&hw_priv->conf_lock);
		hw_priv->scan.req = NULL;
		up(&hw_priv->conf_lock);

		/* cancel scan operation */
		wsm_stop_scan(hw_priv, priv->if_id);

		/* wait scan operation end */
		down(&hw_priv->scan.lock);
		up(&hw_priv->scan.lock);
	}
}

void bes2600_probe_work(struct work_struct *work)
{
	struct bes2600_common *hw_priv =
		container_of(work, struct bes2600_common, scan.probe_work.work);
	struct bes2600_vif *priv, *vif;
	u8 queueId = bes2600_queue_get_queue_id(hw_priv->pending_frame_id);
	struct bes2600_queue *queue = &hw_priv->tx_queue[queueId];
	const struct bes2600_txpriv *txpriv;
	struct wsm_tx *wsm;
	struct wsm_template_frame frame = {
		.frame_type = WSM_FRAME_TYPE_PROBE_REQUEST,
	};
	struct wsm_ssid ssids[1] = {{
		.length = 0,
	} };
	struct wsm_scan_ch ch[1] = {{
		.minChannelTime = 0,
		.maxChannelTime = 10,
	} };
	struct wsm_scan scan = {
		.scanType = WSM_SCAN_TYPE_FOREGROUND,
		.numOfProbeRequests = 2,
		.probeDelay = 0,
		.numOfChannels = 1,
		.ssids = ssids,
		.ch = ch,
	};
	u8 *ies;
	size_t ies_len;
	int ret = 1;
	int i;
	wiphy_info(hw_priv->hw->wiphy, "[SCAN] Direct probe work.\n");

	BUG_ON(queueId >= 4);
	BUG_ON(!hw_priv->channel);

	down(&hw_priv->conf_lock);
	if (unlikely(down_trylock(&hw_priv->scan.lock))) {
		/* Scan is already in progress. Requeue self. */
		schedule();
		queue_delayed_work(hw_priv->workqueue,
					&hw_priv->scan.probe_work, HZ / 10);
		up(&hw_priv->conf_lock);
		return;
	}

	if (bes2600_queue_get_skb(queue,	hw_priv->pending_frame_id,
			&frame.skb, &txpriv)) {
		up(&hw_priv->scan.lock);
		up(&hw_priv->conf_lock);
		wsm_unlock_tx(hw_priv);
		return;
	}
	priv = __cw12xx_hwpriv_to_vifpriv(hw_priv, txpriv->if_id);
	if (!priv) {
		up(&hw_priv->scan.lock);
		up(&hw_priv->conf_lock);
		return;
	}
	wsm = (struct wsm_tx *)frame.skb->data;
	scan.maxTransmitRate = wsm->maxTxRate;
	scan.band = (hw_priv->channel->band == NL80211_BAND_5GHZ) ?
		WSM_PHY_BAND_5G : WSM_PHY_BAND_2_4G;
	if (priv->join_status == BES2600_JOIN_STATUS_STA) {
		scan.scanType = WSM_SCAN_TYPE_BACKGROUND;
		scan.scanFlags = WSM_SCAN_FLAG_FORCE_BACKGROUND;
		if (priv->if_id)
			scan.scanFlags |= WSM_FLAG_MAC_INSTANCE_1;
		else
			scan.scanFlags &= ~WSM_FLAG_MAC_INSTANCE_1;
	}
	bes2600_for_each_vif(hw_priv, vif, i) {
		if (!vif)
			continue;
		if (vif->bss_loss_status > BES2600_BSS_LOSS_NONE)
			scan.scanFlags |= WSM_SCAN_FLAG_FORCE_BACKGROUND;
	}
	ch[0].number = hw_priv->channel->hw_value;

	skb_pull(frame.skb, txpriv->offset);

	ies = &frame.skb->data[sizeof(struct ieee80211_hdr_3addr)];
	ies_len = frame.skb->len - sizeof(struct ieee80211_hdr_3addr);

	if (ies_len) {
		u8 *ssidie =
			(u8 *)cfg80211_find_ie(WLAN_EID_SSID, ies, ies_len);
		if (ssidie && ssidie[1] && ssidie[1] <= sizeof(ssids[0].ssid)) {
			u8 *nextie = &ssidie[2 + ssidie[1]];
			/* Remove SSID from the IE list. It has to be provided
			 * as a separate argument in bes2600_scan_start call */

			/* Store SSID localy */
			ssids[0].length = ssidie[1];
			memcpy(ssids[0].ssid, &ssidie[2], ssids[0].length);
			scan.numOfSSIDs = 1;

			/* Remove SSID from IE list */
			ssidie[1] = 0;
			memmove(&ssidie[2], nextie, &ies[ies_len] - nextie);
			skb_trim(frame.skb, frame.skb->len - ssids[0].length);
		}
	}

	if (priv->if_id == 0)
		bes2600_remove_wps_p2p_ie(&frame);

	/* FW bug: driver has to restart p2p-dev mode after scan */
	if (priv->join_status == BES2600_JOIN_STATUS_MONITOR) {
		WARN_ON(1);
		/*bes2600_disable_listening(priv);*/
	}
	ret = WARN_ON(wsm_set_template_frame(hw_priv, &frame,
				priv->if_id));

	hw_priv->scan.direct_probe = 1;
	hw_priv->scan.if_id = priv->if_id;
	if (!ret) {
		wsm_flush_tx(hw_priv);
		ret = WARN_ON(bes2600_scan_start(priv, &scan));
	}
	up(&hw_priv->conf_lock);

	skb_push(frame.skb, txpriv->offset);
	if (!ret)
		IEEE80211_SKB_CB(frame.skb)->flags |= IEEE80211_TX_STAT_ACK;

	// TODO: find out if this is meant to be guarded by the above if statement
	BUG_ON(bes2600_queue_remove(queue, hw_priv->pending_frame_id));

	if (ret) {
		hw_priv->scan.direct_probe = 0;
		up(&hw_priv->scan.lock);
		wsm_unlock_tx(hw_priv);
	}

	return;
}
