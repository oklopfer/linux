/*
 * Mac80211 driver for BES2600 device
 *
 * Copyright (c) 2022, Bestechnic
 * Author:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <net/mac80211.h>
#include <linux/kthread.h>
#include <uapi/linux/ip.h>
#include <uapi/linux/tcp.h>
#include <uapi/linux/udp.h>

#include "bes2600.h"
#include "bh.h"
#include "hwio.h"
#include "wsm.h"
#include "sbus.h"
#include "debug.h"
#include "epta_coex.h"
#include "bes_chardev.h"
#include "txrx_opt.h"
#include "sta.h"

static int bes2600_bh(void *arg);

extern void sdio_work_debug(struct sbus_priv *self);

/* TODO: Verify these numbers with WSM specification. */
#define DOWNLOAD_BLOCK_SIZE_WR	(0x1000 - 4)
/* an SPI message cannot be bigger than (2"12-1)*2 bytes
 * "*2" to cvt to bytes */
#define MAX_SZ_RD_WR_BUFFERS	(DOWNLOAD_BLOCK_SIZE_WR*2)
#define PIGGYBACK_CTRL_REG	(2)
#define EFFECTIVE_BUF_SIZE	(MAX_SZ_RD_WR_BUFFERS - PIGGYBACK_CTRL_REG)

/* Suspend state privates */
enum bes2600_bh_pm_state {
	BES2600_BH_RESUMED = 0,
	BES2600_BH_SUSPEND,
	BES2600_BH_SUSPENDED,
	BES2600_BH_RESUME,
};

typedef int (*bes2600_wsm_handler)(struct bes2600_common *hw_priv,
	u8 *data, size_t size);

static void bes2600_bh_work(struct work_struct *work)
{
	struct bes2600_common *priv =
	container_of(work, struct bes2600_common, bh_work);
	bes2600_bh(priv);
}

int bes2600_register_bh(struct bes2600_common *hw_priv)
{
	int err = 0;
	/* Realtime workqueue */
	hw_priv->bh_workqueue = alloc_workqueue("bes2600_bh",
				WQ_MEM_RECLAIM | WQ_HIGHPRI
				| WQ_CPU_INTENSIVE, 1);

	if (!hw_priv->bh_workqueue)
		return -ENOMEM;

	INIT_WORK(&hw_priv->bh_work, bes2600_bh_work);

	bes2600_info(BES2600_DBG_BH, "[BH] register.\n");

	coex_init_mode(hw_priv, 0);
	atomic_set(&hw_priv->bh_rx, 0);
	atomic_set(&hw_priv->bh_tx, 0);
	atomic_set(&hw_priv->bh_term, 0);
	atomic_set(&hw_priv->bh_suspend, BES2600_BH_RESUMED);
	hw_priv->buf_id_tx = 0;
	hw_priv->buf_id_rx = 0;
	init_waitqueue_head(&hw_priv->bh_wq);
	init_waitqueue_head(&hw_priv->bh_evt_wq);

	err = !queue_work(hw_priv->bh_workqueue, &hw_priv->bh_work);
	WARN_ON(err);
	return err;
}

void bes2600_unregister_bh(struct bes2600_common *hw_priv)
{
	coex_deinit_mode(hw_priv);

	atomic_add(1, &hw_priv->bh_term);
	wake_up(&hw_priv->bh_wq);

	flush_workqueue(hw_priv->bh_workqueue);

	destroy_workqueue(hw_priv->bh_workqueue);
	hw_priv->bh_workqueue = NULL;

	bes2600_info(BES2600_DBG_BH, "[BH] unregistered.\n");
}

void bes2600_irq_handler(struct bes2600_common *hw_priv)
{
	bes2600_dbg(BES2600_DBG_BH, "[BH] irq.\n");
	if(!hw_priv) {
		bes2600_warn(BES2600_DBG_BH, "%s hw private data is null", __func__);
		return;
	}

	if (hw_priv->bh_error) {
		bes2600_err(BES2600_DBG_BH, "%s bh error", __func__);
		return;
	}

	if (atomic_add_return(1, &hw_priv->bh_rx) == 1)
		wake_up(&hw_priv->bh_wq);
}
EXPORT_SYMBOL(bes2600_irq_handler);

void bes2600_bh_wakeup(struct bes2600_common *hw_priv)
{
	bes2600_dbg(BES2600_DBG_BH,  "[BH] wakeup.\n");
	if (WARN_ON(hw_priv->bh_error))
		return;

	if (atomic_add_return(1, &hw_priv->bh_tx) == 1)
		wake_up(&hw_priv->bh_wq);
}
EXPORT_SYMBOL(bes2600_bh_wakeup);

int bes2600_bh_suspend(struct bes2600_common *hw_priv)
{
	bes2600_dbg(BES2600_DBG_BH, "[BH] suspend.\n");
	if (hw_priv->bh_error) {
		wiphy_warn(hw_priv->hw->wiphy, "BH error -- can't suspend\n");
		return -EINVAL;
	}

	atomic_set(&hw_priv->bh_suspend, BES2600_BH_SUSPEND);
	wake_up(&hw_priv->bh_wq);
	return wait_event_timeout(hw_priv->bh_evt_wq, hw_priv->bh_error ||
		(BES2600_BH_SUSPENDED == atomic_read(&hw_priv->bh_suspend)),
		 1 * HZ) ? 0 : -ETIMEDOUT;
}
EXPORT_SYMBOL(bes2600_bh_suspend);

int bes2600_bh_resume(struct bes2600_common *hw_priv)
{
	int ret;

	bes2600_dbg(BES2600_DBG_BH, "[BH] resume.\n");
	if (hw_priv->bh_error) {
		wiphy_warn(hw_priv->hw->wiphy, "BH error -- can't resume\n");
		return -EINVAL;
	}

	atomic_set(&hw_priv->bh_suspend, BES2600_BH_RESUME);
	wake_up(&hw_priv->bh_wq);
	ret = wait_event_timeout(hw_priv->bh_evt_wq, hw_priv->bh_error ||
		(BES2600_BH_RESUMED == atomic_read(&hw_priv->bh_suspend)),
		1 * HZ) ? 0 : -ETIMEDOUT;

	return ret;
}
EXPORT_SYMBOL(bes2600_bh_resume);

static inline void wsm_alloc_tx_buffer(struct bes2600_common *hw_priv)
{
	++hw_priv->hw_bufs_used;
}

int wsm_release_tx_buffer(struct bes2600_common *hw_priv, int count)
{
	int ret = 0;
	int hw_bufs_used = hw_priv->hw_bufs_used;

	hw_priv->hw_bufs_used -= count;

	if (WARN_ON(hw_priv->hw_bufs_used < 0))
		ret = -1;
	/* Tx data patch stops when all but one hw buffers are used.
	   So, re-start tx path in case we find hw_bufs_used equals
	   numInputChBufs - 1.
	 */
	else if (hw_bufs_used >= (hw_priv->wsm_caps.numInpChBufs - 1))
		ret = 1;
	if (!hw_priv->hw_bufs_used) {
		bes2600_pwr_clear_busy_event(hw_priv, BES_PWR_LOCK_ON_LMAC_RSP);
		wake_up(&hw_priv->bh_evt_wq);
	}
	return ret;
}
EXPORT_SYMBOL(wsm_release_tx_buffer);

int wsm_release_vif_tx_buffer(struct bes2600_common *hw_priv, int if_id,
				int count)
{
	int ret = 0;

	hw_priv->hw_bufs_used_vif[if_id] -= count;

	if (!hw_priv->hw_bufs_used_vif[if_id])
		wake_up(&hw_priv->bh_evt_wq);

	if (WARN_ON(hw_priv->hw_bufs_used_vif[if_id] < 0))
		ret = -1;
	return ret;
}

/* Must be called from BH thraed. */
void bes2600_enable_powersave(struct bes2600_vif *priv,
			     bool enable)
{
	bes2600_dbg(BES2600_DBG_BH, "[BH] Powerave is %s.\n",
			enable ? "enabled" : "disabled");
	priv->powersave_enabled = enable;
}

extern int bes2600_bh_read_ctrl_reg(struct bes2600_common *priv, u32 *ctrl_reg);

static void bes2600_bh_parse_ipv4_data(struct iphdr *ip)
{
	u8 *tmp_ptr = (u8 *)ip;

	bes2600_info(BES2600_DBG_BH, "IP Addr src:0x%08x dst:0x%08x\n", 
		__be32_to_cpu(ip->saddr), __be32_to_cpu(ip->daddr));

	if (ip->protocol == IPPROTO_TCP) {
		struct tcphdr *tcp = (struct tcphdr *)(tmp_ptr + ip->ihl * 4);
		bes2600_info(BES2600_DBG_BH, "TCP Port src:%d dst:%d\n",
			__be16_to_cpu(tcp->source), __be16_to_cpu(tcp->dest));
	} else if (ip->protocol == IPPROTO_UDP) {
		struct udphdr *udp = (struct udphdr *)(tmp_ptr + ip->ihl * 4);
		bes2600_info(BES2600_DBG_BH, "UDP Port src:%d dst:%d\n",
			__be16_to_cpu(udp->source), __be16_to_cpu(udp->dest));
	}
}

static void bes2600_bh_parse_data_pkt(struct bes2600_common *hw_priv, struct sk_buff *skb)
{
	struct wsm_hdr *wsm = (struct wsm_hdr *)skb->data;
	u16 wsm_id = __le16_to_cpu(wsm->id) & 0xFFF;
	int if_id = (wsm_id >> 6) & 0x0F;
	u8 *data_ptr = (u8 *)&wsm[1];
	struct ieee80211_hdr *i80211_ptr = (struct ieee80211_hdr *)(data_ptr + 28 /* radio header */);
	__le16 fctl = *(__le16 *)i80211_ptr;
	struct bes2600_vif *priv = cw12xx_get_vif_from_ieee80211(hw_priv->vif_list[if_id]);
	u32 encry_hdr_len = bes2600_bh_get_encry_hdr_len(priv->cipherType);
	u32 i80211_len = ieee80211_hdrlen(fctl);
	u8 *tmp_ptr = (u8 *)i80211_ptr;
	u16 *eth_type_ptr = (u16 *)(tmp_ptr + i80211_len + encry_hdr_len + ETH_ALEN);
	u16 eth_type = __be16_to_cpu(*eth_type_ptr);

	bes2600_info(BES2600_DBG_BH, "Host was waked by data:\n");
	bes2600_info(BES2600_DBG_BH, "RA:%pM\n", ieee80211_get_DA(i80211_ptr));
	bes2600_info(BES2600_DBG_BH, "ETH_TYPE:0x%04x\n", eth_type);

	if (eth_type == ETH_P_IP) {
		struct iphdr *ip = (struct iphdr *)&eth_type_ptr[1];
		bes2600_info(BES2600_DBG_BH, "IPVersion:%d\n", ip->version);
		bes2600_info(BES2600_DBG_BH, "IPProtol:%d\n", ip->protocol);

		if (ip->version == 4) {
			bes2600_bh_parse_ipv4_data(ip);
		}
	}
}

static void bes2600_bh_parse_wakeup_event(struct bes2600_common *hw_priv, struct sk_buff *skb)
{
	struct wsm_hdr *wsm = (struct wsm_hdr *)skb->data;
	u16 wsm_id = __le16_to_cpu(wsm->id) & 0xFFF;

	if (hw_priv->sbus_ops->wakeup_source &&
	    hw_priv->sbus_ops->wakeup_source(hw_priv->sbus_priv)) {
		if (wsm_id == 0x0804) {
			u8 *data_ptr = (u8 *)&wsm[1];
			u8 *i80211_ptr = data_ptr + 28/* radio header */;
			__le16 fctl = *(__le16 *)i80211_ptr;

			if (ieee80211_is_mgmt(fctl)) {
				u16 type = (fctl & cpu_to_le16(IEEE80211_FCTL_FTYPE)) >> 2;
				u16 stype = (fctl & cpu_to_le16(IEEE80211_FCTL_STYPE)) >> 4;
				bes2600_info(BES2600_DBG_BH, "Host was waked by mgmt, type:%d(%d)\n", type, stype);
			} else if(ieee80211_is_data(fctl)){
				bes2600_bh_parse_data_pkt(hw_priv, skb);
			} else {
				bes2600_info(BES2600_DBG_BH, "Host was waked by unexpected frame, fctl:0x%04x\n", fctl);
			}
		} else {
			bes2600_info(BES2600_DBG_BH, "Host was waked by event:0x%04x.\n", wsm_id);
		}
	}
}

static int bes2600_bh_rx_helper(struct bes2600_common *priv, int *tx)
{
	struct sk_buff *skb = NULL;
	struct wsm_hdr *wsm;
	size_t wsm_len;
	u16 wsm_id;
	u8 wsm_seq;
	int rx = 0;
	u32 confirm_label = 0x0; /* wsm to mcu cmd cnfirm label */

	skb = (struct sk_buff *)priv->sbus_ops->pipe_read(priv->sbus_priv);
	if (!skb)
		return 0;
	rx = 1; // always consider rx pipe not empty

	wsm = (struct wsm_hdr *)skb->data;
	wsm_len = __le16_to_cpu(wsm->len);
	if (WARN_ON(wsm_len > skb->len)) {
		bes2600_err(BES2600_DBG_BH, "wsm_len err %d %d\n", (int)wsm_len, (int)skb->len);
		goto err;
	}

	if (priv->wsm_enable_wsm_dumps)
		print_hex_dump_bytes("<-- ",
				     DUMP_PREFIX_NONE,
				     skb->data, wsm_len);

	wsm_id  = __le16_to_cpu(wsm->id) & 0xFFF;
	wsm_seq = (__le16_to_cpu(wsm->id) >> 13) & 7;
	bes2600_dbg(BES2600_DBG_BH, "bes2600_bh_rx_helper wsm_id:0x%04x seq:%d\n", wsm_id, wsm_seq);

	skb_trim(skb, wsm_len);

	if (wsm_id == 0x0800) {
		wsm_handle_exception(priv,
				     &skb->data[sizeof(*wsm)],
				     wsm_len - sizeof(*wsm));
		bes2600_err(BES2600_DBG_BH, "wsm exception.!\n");
		goto err;
	} else if ((wsm_seq != priv->wsm_rx_seq[WSM_TXRX_SEQ_IDX(wsm_id)])) {
		bes2600_err(BES2600_DBG_BH, "seq error! %u. %u. 0x%x.", wsm_seq, priv->wsm_rx_seq[WSM_TXRX_SEQ_IDX(wsm_id)], wsm_id);
		goto err;
	}

	bes2600_bh_parse_wakeup_event(priv, skb);

	priv->wsm_rx_seq[WSM_TXRX_SEQ_IDX(wsm_id)] = (wsm_seq + 1) & 7;

	if (IS_DRIVER_TO_MCU_CMD(wsm_id))
		confirm_label = __le32_to_cpu(((struct wsm_mcu_hdr *)wsm)->handle_label);

	if (WSM_CONFIRM_CONDITION(wsm_id, confirm_label)) {
		int rc = wsm_release_tx_buffer(priv, 1);
		bes2600_bh_dec_pending_count(priv, WSM_TXRX_SEQ_IDX(wsm->id));

		if (WARN_ON(rc < 0))
			return rc;
		else if (rc > 0)
			*tx = 1;
	}

	/* bes2600_wsm_rx takes care on SKB livetime */
	//if (WARN_ON(wsm_handle_rx(priv, wsm_id, wsm, &skb)))
	if ((wsm_handle_rx(priv, wsm_id, wsm, &skb))) {
		bes2600_err(BES2600_DBG_BH, "wsm_handle_rx fail\n");
		goto err;
	}

	if (skb) {
		dev_kfree_skb(skb);
		skb = NULL;
	}
	return rx;

err:
	bes2600_err(BES2600_DBG_BH, "bes2600_bh_rx_helper err\n");
	if (skb) {
		dev_kfree_skb(skb);
		skb = NULL;
	}
	return -1;
}

static int bes2600_bh_tx_helper(struct bes2600_common *hw_priv,
			       int *pending_tx,
			       int *tx_burst)
{
	size_t tx_len;
	u8 *data;
	int ret;
	struct wsm_hdr *wsm;
	int vif_selected;

	wsm_alloc_tx_buffer(hw_priv);
	ret = wsm_get_tx(hw_priv, &data, &tx_len, tx_burst, &vif_selected);
	if (ret <= 0) {
		wsm_release_tx_buffer(hw_priv, 1);
		if (WARN_ON(ret < 0)) {
			bes2600_err(BES2600_DBG_BH, "bh get tx failed.\n");
			return ret; /* Error */
		}
		return 0; /* No work */
	}

	wsm = (struct wsm_hdr *)data;
	BUG_ON(tx_len < sizeof(*wsm));
	BUG_ON(__le16_to_cpu(wsm->len) != tx_len);
	tx_len += 4;

	atomic_add(1, &hw_priv->bh_tx);

	tx_len = hw_priv->sbus_ops->align_size(
		hw_priv->sbus_priv, tx_len);

	/* Check if not exceeding BES2600 capabilities */
	if (WARN_ON_ONCE(tx_len > EFFECTIVE_BUF_SIZE))
		bes2600_err(BES2600_DBG_BH,  "Write aligned len: %zu\n", tx_len);

	wsm->id &= __cpu_to_le16(0xffff ^ WSM_TX_SEQ(WSM_TX_SEQ_MAX));
	wsm->id |= __cpu_to_le16(WSM_TX_SEQ(hw_priv->wsm_tx_seq[WSM_TXRX_SEQ_IDX(wsm->id)]));

	//bes2600_dbg(BES2600_DBG_BH, "usb send buff len:%u.priv->hw_bufs_used:%d.\n", tx_len, priv->hw_bufs_used);
	bes2600_dbg(BES2600_DBG_BH, "%s id:0x%04x seq:%d\n", __func__,
		wsm->id, hw_priv->wsm_tx_seq[WSM_TXRX_SEQ_IDX(wsm->id)]);

	if (WARN_ON(hw_priv->sbus_ops->pipe_send(hw_priv->sbus_priv, 1, tx_len, data))) {
		bes2600_err(BES2600_DBG_BH,  "tx blew up, len %zu\n", tx_len);
		wsm_release_tx_buffer(hw_priv, 1);
		return -1; /* Error */
	}

	if (vif_selected != -1)
		hw_priv->hw_bufs_used_vif[vif_selected] ++;

	if (hw_priv->wsm_enable_wsm_dumps)
		print_hex_dump_bytes("--> ",
				     DUMP_PREFIX_NONE,
				     data,
				     __le16_to_cpu(wsm->len));

	wsm_txed(hw_priv, data);

	hw_priv->wsm_tx_seq[WSM_TXRX_SEQ_IDX(wsm->id)] =
		(hw_priv->wsm_tx_seq[WSM_TXRX_SEQ_IDX(wsm->id)] + 1) & WSM_TX_SEQ_MAX;
	bes2600_bh_inc_pending_count(hw_priv, WSM_TXRX_SEQ_IDX(wsm->id));

	if (*tx_burst > 1) {
		bes2600_debug_tx_burst(hw_priv);
		return 1; /* Work remains */
	}

	return 0;
}

static inline bool
ieee80211_is_tcp_pkt(struct sk_buff *skb)
{

	if (!skb) {
		return false;
	}

	if (skb->protocol == cpu_to_be16(ETH_P_IP)) {
		struct iphdr *iph = (struct iphdr *)skb_network_header(skb);
		if (iph->protocol == IPPROTO_TCP) { // TCP
			bes2600_dbg(BES2600_DBG_BH, "################ %s line =%d.\n",__func__,__LINE__);
			return true;
		}
	}
	return false;
}


static int bes2600_need_retry_type(struct sk_buff *skb, int status)
{
	int ret = 0;
	if (!skb) {
		bes2600_info(BES2600_DBG_BH, "################ %s line =%d.\n",__func__,__LINE__);
		return -1;
	}

	if (skb->protocol == cpu_to_be16(ETH_P_IP)) {
		if (ieee80211_is_tcp_pkt(skb)) {
			ret = 1;
		}
	}
	if (status !=  WSM_STATUS_RETRY_EXCEEDED)
		ret = 0;
	return ret;
}

int bes2600_bh_sw_process(struct bes2600_common *hw_priv,
			 struct wsm_tx_confirm *tx_confirm)
{
	struct bes2600_txpriv *txpriv;
	struct sk_buff *skb = NULL;
	unsigned long timestamp = 0;
	struct bes2600_queue *queue;
	u8 queue_id, queue_gen;

	long delta_time;
	if (!tx_confirm) {
		bes2600_err(BES2600_DBG_BH,  "%s tx_confirm is NULL\n", __func__);
		return 0;
	}
	queue_id = bes2600_queue_get_queue_id(tx_confirm->packetID);
	queue = &hw_priv->tx_queue[queue_id];

	if (!queue) {
		bes2600_err(BES2600_DBG_BH,  "%s queue is NULL\n", __func__);
		return 0;
	}

	/* don't retry if the connection is already disconnected */
	queue_gen = bes2600_queue_get_generation(tx_confirm->packetID);
	if(queue_gen != queue->generation)
		return -1;

	bes2600_queue_get_skb_and_timestamp(queue, tx_confirm->packetID,
						&skb, &txpriv, &timestamp);
	if (skb == NULL) {
		bes2600_err(BES2600_DBG_BH,  "%s skb is NULL\n", __func__);
		return -1;
	}
	if (timestamp > jiffies)
		delta_time = jiffies + ((unsigned long)0xffffffff - timestamp);
	else
		delta_time =  jiffies - timestamp;
	bes2600_add_tx_delta_time(delta_time);
	bes2600_add_tx_ac_delta_time(queue_id, delta_time);

	if (bes2600_need_retry_type(skb, tx_confirm->status) == 0)
		return -1;

	if (delta_time > 1000)
		return -1;

	if (txpriv->retry_count < CW1200_MAX_SW_RETRY_CNT ) {
		struct bes2600_vif *priv =
		__cw12xx_hwpriv_to_vifpriv(hw_priv, txpriv->if_id);
		txpriv->retry_count++;

		bes2600_tx_status(priv,skb);

		bes2600_pwr_set_busy_event_with_timeout_async(
			hw_priv, BES_PWR_LOCK_ON_TX, BES_PWR_EVENT_TX_TIMEOUT);

		bes2600_sw_retry_requeue(hw_priv, queue, tx_confirm->packetID, true);
		return 0;
	} else {
		txpriv->retry_count = 0;
	}

	return -1;
}

void bes2600_bh_inc_pending_count(struct bes2600_common *hw_priv, int idx)
{
	struct timer_list *timer = (idx == 0) ? &hw_priv->lmac_mon_timer
	                                      : &hw_priv->mcu_mon_timer;

	if(hw_priv->wsm_tx_pending[idx]++ == 0) {
		bes2600_dbg(BES2600_DBG_BH,  "start timer in tx, idx:%d\n", idx);
		mod_timer(timer, jiffies + 3 * HZ);
	}
}

void bes2600_bh_dec_pending_count(struct bes2600_common *hw_priv, int idx)
{
	struct timer_list *timer = (idx == 0) ? &hw_priv->lmac_mon_timer
	                                      : &hw_priv->mcu_mon_timer;

	if(hw_priv->wsm_tx_pending[idx] == 0) {
		bes2600_err(BES2600_DBG_TXRX, "tx pending count error, idx:%d\n", idx);
		return;
	}

	if(--hw_priv->wsm_tx_pending[idx] == 0) {
		del_timer_sync(timer);
	} else {
		mod_timer(timer, jiffies + 3 * HZ);
	}
}

void bes2600_bh_mcu_active_monitor(struct timer_list* t)
{
	struct bes2600_common *hw_priv = from_timer(hw_priv, t, mcu_mon_timer);

	bes2600_err(BES2600_DBG_TXRX, "link break between mcu and host, hw_buf_used:%d pending:%d\n", 
				hw_priv->hw_bufs_used, hw_priv->wsm_tx_pending[1]);
	bes2600_chrdev_wifi_force_close(hw_priv, true);
}

void bes2600_bh_lmac_active_monitor(struct timer_list* t)
{
	struct bes2600_common *hw_priv = from_timer(hw_priv, t, lmac_mon_timer);

	bes2600_err(BES2600_DBG_TXRX, "link break between lmac and host, hw_buf_used:%d pending:%d\n", 
				hw_priv->hw_bufs_used, hw_priv->wsm_tx_pending[0]);
	bes2600_chrdev_wifi_force_close(hw_priv, true);
}

#define BH_RX_CONT_LIMIT	3
#define BH_TX_CONT_LIMIT	20
static int bes2600_bh(void *arg)
{
	struct bes2600_common *hw_priv = arg;
	int rx, tx, term, suspend;
	int tx_allowed;
	int pending_tx = 0;
	int tx_burst;
	long status;
	int ret;

	int tx_cont = 0;
	int rx_cont = 0;

	for (;;) {
		rx_cont = 0;
		tx_cont = 0;

		if (!hw_priv->hw_bufs_used &&
		    !bes2600_pwr_device_is_idle(hw_priv) &&
		    !atomic_read(&hw_priv->recent_scan)) {
			status = 5 * HZ;
		} else if (hw_priv->hw_bufs_used) {
			/* Interrupt loss detection */
			status = 5 * HZ;
		} else {
			status = MAX_SCHEDULE_TIMEOUT;
		}

		status = wait_event_interruptible_timeout(hw_priv->bh_wq, ({
				rx = atomic_xchg(&hw_priv->bh_rx, 0);
				tx = atomic_xchg(&hw_priv->bh_tx, 0);
				term = atomic_xchg(&hw_priv->bh_term, 0);
				suspend = pending_tx ?
					0 : atomic_read(&hw_priv->bh_suspend);
				(rx || tx || term || suspend || hw_priv->bh_error);
			}), status);

		//bes2600_err(BES2600_DBG_BH,  "[BH] - rx: %d, tx: %d, term: %d, bh_err: %d, suspend: %d, bufused: %d, status: %ld\n",
				//rx, tx, term, suspend, hw_priv->bh_error, hw_priv->hw_bufs_used, status);

		/* Did an error occur? */
		if ((status < 0 && status != -ERESTARTSYS) ||
		    term || hw_priv->bh_error) {
			break;
		}
		if (!status) {  /* wait_event timed out */
			unsigned long timestamp = jiffies;
			long timeout;
			int pending = 0;
			int i;
			/* Check to see if we have any outstanding frames */
			if (hw_priv->hw_bufs_used && (!rx || !tx)) {
				bes2600_err(BES2600_DBG_BH,  "usedbuf:%u. rx:%u. tx:%u.\n", hw_priv->hw_bufs_used, rx, tx);
				sdio_work_debug(hw_priv->sbus_priv);
				bes2600_err(BES2600_DBG_BH,  "Missed interrupt? (%d frames outstanding)\n",
					   hw_priv->hw_bufs_used);
				rx = 1;

				/* Get a timestamp of "oldest" frame */
				for (i = 0; i < 4; ++i)
					pending += bes2600_queue_get_xmit_timestamp(
						&hw_priv->tx_queue[i],
						&timestamp, i,
						hw_priv->pending_frame_id);

				/* Check if frame transmission is timed out.
				 * Add an extra second with respect to possible
				 * interrupt loss.
				 */
				timeout = timestamp +
					WSM_CMD_LAST_CHANCE_TIMEOUT +
					1 * HZ  -
					jiffies;

				/* And terminate BH thread if the frame is "stuck" */
				if (pending && timeout < 0) {
					wiphy_warn(hw_priv->hw->wiphy,
						   "Timeout waiting for TX confirm (%d/%d pending, %ld vs %lu).\n",
						   hw_priv->hw_bufs_used, pending,
						   timestamp, jiffies);
				}

				bes2600_chrdev_wifi_force_close(hw_priv, false);
			}
			goto rx;
		} else if (suspend) {
			bes2600_dbg(BES2600_DBG_BH,  "[BH] Device suspend.\n");

			atomic_set(&hw_priv->bh_suspend, BES2600_BH_SUSPENDED);
			wake_up(&hw_priv->bh_evt_wq);
			status = wait_event_interruptible(hw_priv->bh_wq,
							  BES2600_BH_RESUME == atomic_read(&hw_priv->bh_suspend));
			if (status < 0) {
				wiphy_err(hw_priv->hw->wiphy,
					  "Failed to wait for resume: %ld.\n",
					  status);
				break;
			}
			bes2600_dbg(BES2600_DBG_BH,  "[BH] Device resume.\n");
			atomic_set(&hw_priv->bh_suspend, BES2600_BH_RESUMED);
			wake_up(&hw_priv->bh_evt_wq);
			atomic_add(1, &hw_priv->bh_rx);
			goto done;
		}

	rx:
		tx += pending_tx;
		pending_tx = 0;
		ret = bes2600_bh_rx_helper(hw_priv, &tx);
		if (ret < 0) {
			bes2600_err(BES2600_DBG_BH, "bes2600_bh_rx_helper fail\n");
			sdio_work_debug(hw_priv->sbus_priv);
			// break; // rx error
			bes2600_chrdev_wifi_force_close(hw_priv, false);
		}
		else if (ret == 1) {
			rx = 1; // continue rx
			rx_cont++;
		}
		else
			rx = 0; // wait for a new rx event
		if (rx && (rx_cont < BH_RX_CONT_LIMIT))
			goto rx;
		rx_cont = 0;
	tx:
		if (1) {

			tx = 0;

			BUG_ON(hw_priv->hw_bufs_used > hw_priv->wsm_caps.numInpChBufs);
			tx_burst = hw_priv->wsm_caps.numInpChBufs - hw_priv->hw_bufs_used;
			tx_allowed = tx_burst > 0;

			if (!tx_allowed) {
				/* Buffers full.  Ensure we process tx
				 * after we handle rx..
				 */
				pending_tx = tx;
				goto done_rx;
			}
			ret = bes2600_bh_tx_helper(hw_priv, &pending_tx, &tx_burst);
			if (ret < 0) {
				bes2600_err(BES2600_DBG_BH, "bes2600_bh_tx_helper fail\n");
				sdio_work_debug(hw_priv->sbus_priv);
				break;
			}
			if (ret > 0) {
				/* More to transmit */
				tx_cont++;
				tx = ret;
			}

			if (tx && tx_cont < BH_TX_CONT_LIMIT)
				goto tx;
			tx_cont = 0;
		}

	done_rx:
		if (hw_priv->bh_error)
			break;
		//if (ctrl_reg & ST90TDS_CONT_NEXT_LEN_MASK)
		if (rx)
			goto rx;
		if (tx)
			goto tx;

	done:
		/* Re-enable device interrupts */
		//hw_priv->sbus_ops->lock(hw_priv->sbus_priv);
		//__bes2600_irq_enable(1);
		//hw_priv->sbus_ops->unlock(hw_priv->sbus_priv);
		asm volatile ("nop");
	}

	/* Explicitly disable device interrupts */
	hw_priv->sbus_ops->lock(hw_priv->sbus_priv);
	__bes2600_irq_enable(0);
	hw_priv->sbus_ops->unlock(hw_priv->sbus_priv);

	if (!term) {
		bes2600_err(BES2600_DBG_BH,  "[BH] Fatal error, exiting.\n");
		sdio_work_debug(hw_priv->sbus_priv);
		hw_priv->bh_error = 1;
		/* TODO: schedule_work(recovery) */
	}
	return 0;
}
