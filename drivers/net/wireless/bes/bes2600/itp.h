/*
 * ITP code for BES2600 mac80211 driver
 *
 * Copyright (c) 2011, Bestechnic
 * Author:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef BES2600_ITP_H_INCLUDED
#define BES2600_ITP_H_INCLUDED

struct cw200_common;
struct wsm_tx_confirm;
struct dentry;

static inline int
bes2600_itp_init(struct bes2600_common *priv)
{
	return 0;
}

static inline void bes2600_itp_release(struct bes2600_common *priv)
{
}

static inline bool bes2600_is_itp(struct bes2600_common *priv)
{
	return false;
}

static inline bool bes2600_itp_rxed(struct bes2600_common *priv,
		struct sk_buff *skb)
{
	return false;
}


static inline void bes2600_itp_consume_txed(struct bes2600_common *priv)
{
}

static inline void bes2600_itp_wake_up_tx(struct bes2600_common *priv)
{
}

static inline int bes2600_itp_get_tx(struct bes2600_common *priv, u8 **data,
		size_t *tx_len, int *burst)
{
	return 0;
}

static inline bool bes2600_itp_tx_running(struct bes2600_common *priv)
{
	return false;
}

#endif /* BES2600_ITP_H_INCLUDED */
