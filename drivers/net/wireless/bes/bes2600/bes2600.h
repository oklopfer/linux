/*
 * Common private data for BES2600 drivers
 *
 * Copyright (c) 2010, Bestechnic
 * Author:
 *
 * Based on the mac80211 Prism54 code, which is
 * Copyright (c) 2006, Michael Wu <flamingice@sourmilk.net>
 *
 * Based on the islsm (softmac prism54) driver, which is:
 * Copyright 2004-2006 Jean-Baptiste Note <jbnote@gmail.com>, et al.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef BES2600_H
#define BES2600_H

#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/atomic.h>
#include <linux/usb.h>
#include <net/mac80211.h>
#define CW12XX_MAX_VIFS			(3)
#define CW12XX_GENERIC_IF_ID		(2)
#define CW12XX_HOST_VIF0_11N_THROTTLE	(63)
#define CW12XX_HOST_VIF1_11N_THROTTLE	(63)
#define CW12XX_HOST_VIF0_11BG_THROTTLE	(15)
#define CW12XX_HOST_VIF1_11BG_THROTTLE	(15)
#define CW12XX_MAX_QUEUE_SZ		(128)

#define IEEE80211_FCTL_WEP      0x4000
#define IEEE80211_QOS_DATAGRP   0x0080
#define WSM_KEY_MAX_IDX		20

#include "queue.h"
#include "wsm.h"
#include "scan.h"
#include "txrx.h"
#include "ht.h"
#include "pm.h"
#include "fwio.h"
#include "bes2600_log.h"
#include "bes_pwr.h"
#include "tx_loop.h"

/* extern */ struct sbus_ops;
/* extern */ struct task_struct;
/* extern */ struct bes2600_debug_priv;
/* extern */ struct bes2600_debug_common;
/* extern */ struct firmware;

#define txrx_printk(...)

#define BES2600_MAX_CTRL_FRAME_LEN	(0x1000)

#define CW1250_MAX_STA_IN_AP_MODE	(14)
#define WLAN_LINK_ID_MAX		(CW1250_MAX_STA_IN_AP_MODE + 3)

#define BES2600_MAX_STA_IN_AP_MODE	(5)
#define BES2600_MAX_REQUEUE_ATTEMPTS	(5)
#define BES2600_LINK_ID_UNMAPPED		(15)

#define BES2600_MAX_TID			(8)

#define BES2600_TX_BLOCK_ACK_ENABLED_FOR_ALL_TID         (0x3F)
#define BES2600_RX_BLOCK_ACK_ENABLED_FOR_ALL_TID         (0x3F)
#define BES2600_RX_BLOCK_ACK_ENABLED_FOR_BE_TID \
	(BES2600_TX_BLOCK_ACK_ENABLED_FOR_ALL_TID & 0x01)
#define BES2600_TX_BLOCK_ACK_DISABLED_FOR_ALL_TID	(0)
#define BES2600_RX_BLOCK_ACK_DISABLED_FOR_ALL_TID	(0)

#define BES2600_BLOCK_ACK_CNT		(30)
#define BES2600_BLOCK_ACK_THLD		(800)
#define BES2600_BLOCK_ACK_HIST		(3)
#define BES2600_BLOCK_ACK_INTERVAL	(1 * HZ / BES2600_BLOCK_ACK_HIST)
#define CW12XX_ALL_IFS			(-1)

#define IEEE80211_FCTL_WEP      0x4000
#define IEEE80211_QOS_DATAGRP   0x0080

#define BSS_LOSS_CK_THR		1
#define BSS_LOSS_CK_INV		2000
#define BSS_LOSS_CFM_THR	1
#define BSS_LOSS_CFM_INV	200

/* Please keep order */
enum bes2600_join_status {
	BES2600_JOIN_STATUS_PASSIVE = 0,
	BES2600_JOIN_STATUS_MONITOR,
	BES2600_JOIN_STATUS_STA,
	BES2600_JOIN_STATUS_AP,
};

enum bes2600_link_status {
	BES2600_LINK_OFF,
	BES2600_LINK_RESERVE,
	BES2600_LINK_SOFT,
	BES2600_LINK_HARD,
};

enum bes2600_bss_loss_status {
	BES2600_BSS_LOSS_NONE,
	BES2600_BSS_LOSS_CHECKING,
	BES2600_BSS_LOSS_CONFIRMING,
	BES2600_BSS_LOSS_CONFIRMED,
};

struct bes2600_link_entry {
	unsigned long			timestamp;
	enum bes2600_link_status	status;
	u8				mac[ETH_ALEN];
	u8				buffered[BES2600_MAX_TID];
	struct sk_buff_head		rx_queue;
};

struct bes2600_common {
	struct bes2600_debug_common	*debug;
	struct bes2600_queue		tx_queue[4];
	struct bes2600_queue_stats	tx_queue_stats;

	struct ieee80211_hw		*hw;
	struct mac_address		addresses[CW12XX_MAX_VIFS];

	/*Will be a pointer to a list of VIFs - Dynamically allocated */
	struct ieee80211_vif		*vif_list[CW12XX_MAX_VIFS];
	atomic_t			num_vifs;
	atomic_t			netdevice_start;
	spinlock_t			vif_list_lock;
	u32				if_id_slot;
	struct device			*pdev;
	struct workqueue_struct		*workqueue;

	struct semaphore		conf_lock;

	const struct sbus_ops		*sbus_ops;
	struct sbus_priv		*sbus_priv;

	/* HW/FW type (HIF_...) */
	int				hw_type;
	int				hw_revision;
	int				fw_revision;

	/* firmware/hardware info */
	unsigned int tx_hdr_len;

	/* Radio data */
	int output_power;
	int noise;

	/* calibration, output power limit and rssi<->dBm conversation data */

	/* BBP/MAC state */
	const struct firmware		*sdd;
	struct ieee80211_rate		*rates;
	struct ieee80211_rate		*mcs_rates;
	u8 mac_addr[ETH_ALEN];
	/*TODO:COMBO: To be made per VIFF after mac80211 support */
	struct ieee80211_channel	*channel;
	int				channel_switch_in_progress;
	wait_queue_head_t		channel_switch_done;
	u8				long_frame_max_tx_count;
	u8				short_frame_max_tx_count;
	/* TODO:COMBO: According to Hong aggregation will happen per VIFF.
	* Keeping in common structure for the time being. Will be moved to VIFF
	* after the mechanism is clear */
	u8				ba_tid_mask;
	int				ba_acc; /*TODO: Same as above */
	int				ba_cnt; /*TODO: Same as above */
	int				ba_cnt_rx; /*TODO: Same as above */
	int				ba_acc_rx; /*TODO: Same as above */
	int				ba_hist; /*TODO: Same as above */
	struct timer_list		ba_timer;/*TODO: Same as above */
	spinlock_t			ba_lock; /*TODO: Same as above */
	bool				ba_ena; /*TODO: Same as above */
	struct work_struct              ba_work; /*TODO: Same as above */
	bool				is_BT_Present;
	bool				is_go_thru_go_neg;
	u8				conf_listen_interval;

	/* BH */
	atomic_t			bh_rx;
	atomic_t			bh_tx;
	atomic_t			bh_term;
	atomic_t			bh_suspend;

	struct workqueue_struct         *bh_workqueue;
	struct work_struct              bh_work;

	int				bh_error;
	wait_queue_head_t		bh_wq;
	wait_queue_head_t		bh_evt_wq;
	int				buf_id_tx;	/* byte */
	int				buf_id_rx;	/* byte */
	int				wsm_rx_seq[2];	/* idx 0: Normal tx/rx, idx 1: special cmd */
	int				wsm_tx_seq[2];	/* idx 0: Normal tx/rx, idx 1: special cmd */
	int				wsm_tx_pending[2];	/* idx 0: Normal tx/rx, idx 1: special cmd */
	struct timer_list		mcu_mon_timer;
	struct timer_list		lmac_mon_timer;
	int				hw_bufs_used;
	int				hw_bufs_used_vif[CW12XX_MAX_VIFS];
	struct sk_buff			*skb_cache;
	/* Keep bes2600 awake (WUP = 1) 1 second after each scan to avoid
	 * FW issue with sleeping/waking up. */
	atomic_t			recent_scan;

	/* WSM */
	struct wsm_caps			wsm_caps;
	struct semaphore		wsm_cmd_sema;
	struct wsm_buf			wsm_cmd_buf;
	struct wsm_cmd			wsm_cmd;
	wait_queue_head_t		wsm_cmd_wq;
	wait_queue_head_t		wsm_startup_done;
	struct wsm_cbc			wsm_cbc;
	atomic_t			tx_lock;
	u32				pending_frame_id;

	/* WSM debug */
	int				wsm_enable_wsm_dumps;
	u32				wsm_dump_max_size;

	/* Scan status */
	struct bes2600_scan scan;

	/* TX/RX */
	unsigned long		rx_timestamp;

	/* Scan Timestamp */
	unsigned long		scan_timestamp;

	/* WSM events */
	spinlock_t		event_queue_lock;
	struct list_head	event_queue;
	struct work_struct	event_handler;

	/* TX rate policy cache */
	struct tx_policy_cache tx_policy_cache;
	struct work_struct tx_policy_upload_work;

	/* cryptographic engine information */

	/* bit field of glowing LEDs */
	u16 softled_state;

	/* statistics */
	struct ieee80211_low_level_stats stats;

	struct bes2600_ht_info		ht_info;
	int				tx_burst_idx;

	struct ieee80211_iface_limit		if_limits1[2];
	struct ieee80211_iface_limit		if_limits2[2];
	struct ieee80211_iface_limit		if_limits3[2];
	struct ieee80211_iface_combination	if_combs[3];

	struct semaphore		wsm_oper_lock;
	struct delayed_work		rem_chan_timeout;
	MIB_TXRX_OPT_PARAM		txrx_opt_param;
	u32 					rtsvalue;
	spinlock_t				rtsvalue_lock;
	struct timer_list		txrx_opt_timer;
	struct work_struct		dynamic_opt_txrx_work;
	atomic_t			remain_on_channel;
	int				roc_if_id;
	u64				roc_cookie;
	wait_queue_head_t		offchannel_wq;
	u16				offchannel_done;
	u16				prev_channel;
	int				if_id_selected;
	u32				key_map;
	struct wsm_add_key		keys[WSM_KEY_MAX_INDEX + 1];
	u8      connected_sta_cnt;
	u16     vif0_throttle;
	u16     vif1_throttle;
	int     scan_switch_if_id;

	/* member for coexistence */
	struct work_struct coex_work;
	struct list_head coex_event_list;
	spinlock_t coex_event_lock;

	/* member for low power */
	struct bes2600_pwr_t bes_power;

	/* member for tx loop */
	struct bes2600_tx_loop tx_loop;
};

/* Virtual Interface State. One copy per VIF */
struct bes2600_vif {
	atomic_t			enabled;
	spinlock_t			vif_lock;
	int				if_id;
	/*TODO: Split into Common and VIF parts */
	struct bes2600_debug_priv	*debug;
	/* BBP/MAC state */
	u8 bssid[ETH_ALEN];
	struct wsm_edca_params		edca;
	struct wsm_tx_queue_params	tx_queue_params;
	struct wsm_association_mode	association_mode;
	struct wsm_set_bss_params	bss_params;
	struct wsm_set_pm		powersave_mode;
	struct wsm_set_pm		firmware_ps_mode;
	int				power_set_true;
	int				user_power_set_true;
	u8				user_pm_mode;
	int				cqm_rssi_thold;
	unsigned			cqm_rssi_hyst;
	unsigned			cqm_tx_failure_thold;
	unsigned			cqm_tx_failure_count;
	unsigned			cmq_tx_success_count;
	bool				cqm_use_rssi;
	int				cqm_link_loss_count;
	int				cqm_beacon_loss_count;
	int				mode;
	bool				enable_beacon;
	int				beacon_int;
	size_t				ssid_length;
	u8				ssid[IEEE80211_MAX_SSID_LEN];
	bool				listening;
	struct wsm_rx_filter		rx_filter;
	struct wsm_beacon_filter_table	bf_table;
	struct wsm_beacon_filter_control bf_control;
	struct wsm_multicast_filter	multicast_filter;
	bool				has_multicast_subscription;
	struct wsm_broadcast_addr_filter	broadcast_filter;
	bool				disable_beacon_filter;
	struct wsm_arp_ipv4_filter      filter4;
	struct work_struct		update_filtering_work;
	struct work_struct		set_beacon_wakeup_period_work;
	struct bes2600_pm_state_vif	pm_state_vif;
	/*TODO: Add support in mac80211 for psmode info per VIF */
	struct wsm_p2p_ps_modeinfo	p2p_ps_modeinfo;
	struct wsm_uapsd_info		uapsd_info;
	bool				setbssparams_done;
	u32				listen_interval;
	u32				erp_info;
	bool				powersave_enabled;

	/* WSM Join */
	enum bes2600_join_status	join_status;
	u8			join_bssid[ETH_ALEN];
	struct work_struct	join_work;
	struct delayed_work	join_timeout;
	struct work_struct	unjoin_work;
	struct work_struct	offchannel_work;
	int			join_dtim_period;
	bool			delayed_unjoin;
	atomic_t		connect_in_process;

	/* Security */
	s8			wep_default_key_id;
	struct work_struct	wep_key_work;
        unsigned long           rx_timestamp;
        u32                     cipherType;


	/* AP powersave */
	u32			link_id_map;
	u32			max_sta_ap_mode;
	u32			link_id_after_dtim;
	u32			link_id_uapsd;
	u32			link_id_max;
	u32			wsm_key_max_idx;
	struct bes2600_link_entry link_id_db[CW1250_MAX_STA_IN_AP_MODE];
	struct work_struct	link_id_work;
	struct delayed_work	link_id_gc_work;
	u32			sta_asleep_mask;
	u32			pspoll_mask;
	bool			aid0_bit_set;
	spinlock_t		ps_state_lock;
	bool			buffered_multicasts;
	bool			tx_multicast;
	struct work_struct	set_tim_work;
	struct delayed_work	set_cts_work;
	struct work_struct	multicast_start_work;
	struct work_struct	multicast_stop_work;
	struct timer_list	mcast_timeout;

	/* CQM Implementation */
	struct delayed_work	bss_loss_work;
	struct delayed_work	connection_loss_work;
	struct work_struct	tx_failure_work;
	int			delayed_link_loss;
	spinlock_t		bss_loss_lock;
	int			bss_loss_status;
	int			bss_loss_confirm_id;

	struct ieee80211_vif	*vif;
	struct bes2600_common	*hw_priv;
	struct ieee80211_hw	*hw;

	/* ROC implementation */
	struct delayed_work		pending_offchanneltx_work;
	bool			htcap;
        u16                     ht_info;
        struct work_struct      ht_info_update_work;
	bool pmf;

    u32 hw_value;
    /* dot11CountersTable */
    u32 dot11TransmittedFragmentCount;
    u32 dot11MulticastTransmittedFrameCount;
    u32 dot11FailedCount;
    u32 dot11RetryCount;
    u32 dot11MultipleRetryCount;
    u32 dot11FrameDuplicateCount;
    u32 dot11ReceivedFragmentCount;
    u32 dot11RxReorderLeakCount;
    u32 dot11ReceivedBytes;
    u32 dot11ReceivedDataBytes;
    u32 dot11MulticastReceivedFrameCount;
    u32 dot11TransmittedFrameCount;
    u32 dot11TransmittedBytes;
    u32 dot11TransmittedDataBytes;
    u32 dot11Txbps;
    u32 dot11Rxbps;
};
struct bes2600_sta_priv {
	int link_id;
	struct bes2600_vif *priv;
};
enum bes2600_data_filterid {
	IPV4ADDR_FILTER_ID = 0,
};

static inline
struct bes2600_common *cw12xx_vifpriv_to_hwpriv(struct bes2600_vif *priv)
{
	return priv->hw_priv;
}


static inline
struct bes2600_vif *cw12xx_get_vif_from_ieee80211(struct ieee80211_vif *vif)
{
	return  (struct bes2600_vif *)vif->drv_priv;
}

static inline
struct bes2600_vif *cw12xx_hwpriv_to_vifpriv(struct bes2600_common *hw_priv,
						int if_id)
{
	struct bes2600_vif *vif;

	if (WARN_ON((-1 == if_id) || (if_id > CW12XX_MAX_VIFS)))
		return NULL;
	/* TODO:COMBO: During scanning frames can be received
	 * on interface ID 3 */
	spin_lock(&hw_priv->vif_list_lock);
	if (!hw_priv->vif_list[if_id]) {
		spin_unlock(&hw_priv->vif_list_lock);
		return NULL;
	}

	vif = cw12xx_get_vif_from_ieee80211(hw_priv->vif_list[if_id]);
	WARN_ON(!vif);
	if (vif)
		spin_lock(&vif->vif_lock);
	spin_unlock(&hw_priv->vif_list_lock);
	return vif;
}

static inline
struct bes2600_vif *__cw12xx_hwpriv_to_vifpriv(struct bes2600_common *hw_priv,
					      int if_id)
{
	WARN_ON((-1 == if_id) || (if_id > CW12XX_MAX_VIFS));
	/* TODO:COMBO: During scanning frames can be received
	 * on interface ID 3 */
	if (!hw_priv->vif_list[if_id]) {
		return NULL;
	}

	return cw12xx_get_vif_from_ieee80211(hw_priv->vif_list[if_id]);
}

static inline
struct bes2600_vif *cw12xx_get_activevif(struct bes2600_common *hw_priv)
{
	return cw12xx_hwpriv_to_vifpriv(hw_priv, ffs(hw_priv->if_id_slot)-1);
}

static inline bool is_hardware_cw1250(struct bes2600_common *hw_priv)
{
	return (hw_priv->hw_revision == BES2600_HW_REV_CUT20);
}

static inline bool is_hardware_cw1260(struct bes2600_common *hw_priv)
{
	return (hw_priv->hw_revision == BES2600_HW_REV_CUT10);
}

static inline int cw12xx_get_nr_hw_ifaces(struct bes2600_common *hw_priv)
{
	switch(hw_priv->hw_revision) {
		case BES2600_HW_REV_CUT10:
		case BES2600_HW_REV_CUT11:
		case BES2600_HW_REV_CUT20:
		case BES2600_HW_REV_CUT22:
			return 1;
		case CW1250_HW_REV_CUT10:
			return 3;
		default:
			return 1;
	}
}

/* interfaces for the drivers */
int bes2600_core_probe(const struct sbus_ops *sbus_ops,
		      struct sbus_priv *sbus,
		      struct device *pdev,
		      struct bes2600_common **pself);
void bes2600_core_release(struct bes2600_common *self);

static inline void bes2600_tx_queues_lock(struct bes2600_common *hw_priv)
{
	int i;
	for (i = 0; i < 4; ++i)
		bes2600_queue_lock(&hw_priv->tx_queue[i]);
}

static inline void bes2600_tx_queues_unlock(struct bes2600_common *hw_priv)
{
	int i;
	for (i = 0; i < 4; ++i)
		bes2600_queue_unlock(&hw_priv->tx_queue[i]);
}

/* Datastructure for LLC-SNAP HDR */
#define P80211_OUI_LEN  3

struct ieee80211_snap_hdr {
        u8    dsap;   /* always 0xAA */
        u8    ssap;   /* always 0xAA */
        u8    ctrl;   /* always 0x03 */
        u8    oui[P80211_OUI_LEN];    /* organizational universal id */
} __packed;

#define bes2600_for_each_vif(_hw_priv, _priv, _i)			\
for (									\
	_i = 0;								\
	(_i < CW12XX_MAX_VIFS) && \
	(_priv = hw_priv->vif_list[_i] ? 				\
	cw12xx_get_vif_from_ieee80211(hw_priv->vif_list[_i]) : NULL);	\
	_i++								\
)

#endif /* BES2600_H */
