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
#include <linux/init.h>
#include <linux/firmware.h>
#include <linux/etherdevice.h>
#include <linux/vmalloc.h>
#include <linux/random.h>
#include <linux/sched.h>
#include <net/mac80211.h>
#include <net/cfg80211.h>
#include <linux/ctype.h>
#include <linux/rtnetlink.h>
#include <net/netlink.h>
#include <linux/netlink.h>
#include "bes2600_cfgvendor.h"

void bes2600_reg_notifier(struct wiphy *wiphy,
                              struct regulatory_request *request)
{
	const struct ieee80211_regdomain *tmp = NULL;

	/* If wiphy->regd is not cleared, reopening the SoftAP after the sta disconnects from an AP will fail */
	if(request->initiator == NL80211_REGDOM_SET_BY_CORE &&
	   !(wiphy->regulatory_flags & REGULATORY_CUSTOM_REG)) {
		tmp = rtnl_dereference(wiphy->regd);
		if(tmp) {
			rcu_assign_pointer(wiphy->regd, NULL);
			kfree_rcu((struct ieee80211_regdomain *)tmp, rcu_head);
			bes2600_info(BES2600_DBG_ANDROID, "clear regdom when sta disconnects from an ap.\n");
		}
	}
}

static int bes2600_set_country_code(struct wiphy *wiphy,
	struct wireless_dev *wdev, const void  *data, int len)
{
#define CNTRY_BUF_SZ	4	/* Country string is 3 bytes + NUL */
	int rem, type;
	char country_code[CNTRY_BUF_SZ] = {0};
	const struct nlattr *iter;

	nla_for_each_attr(iter, data, len, rem) {
		type = nla_type(iter);
		switch (type) {
			case ANDR_WIFI_ATTRIBUTE_COUNTRY:
				memcpy(country_code, nla_data(iter),
					MIN(nla_len(iter), CNTRY_BUF_SZ));
				break;
			default:
				return -EINVAL;
		}
	}

	/* check whether the country is valid or not */
	if(!isalpha(country_code[0]) ||
	   !isalpha(country_code[1])) {
		return -EINVAL;
	}

	/* notify cfg80211 to update database */
	return regulatory_hint(wiphy, country_code);
}

static const struct wiphy_vendor_command bes2600_own_commands[] = {
	{
		{
		.vendor_id = OUI_GOOGLE,
		.subcmd = WIFI_SUBCMD_SET_COUNTRY_CODE
		},
		.flags = WIPHY_VENDOR_CMD_NEED_WDEV | WIPHY_VENDOR_CMD_NEED_NETDEV,
		.doit = bes2600_set_country_code,
		.policy = VENDOR_CMD_RAW_DATA,

	},
};

static const struct  nl80211_vendor_cmd_info bes2600_own_events[] = {
	{ OUI_GOOGLE, GSCAN_EVENT_SIGNIFICANT_CHANGE_RESULTS },
	{ OUI_GOOGLE, GSCAN_EVENT_HOTLIST_RESULTS_FOUND },
	{ OUI_GOOGLE, GSCAN_EVENT_SCAN_RESULTS_AVAILABLE },
	{ OUI_GOOGLE, GSCAN_EVENT_FULL_SCAN_RESULTS },
	{ OUI_GOOGLE, RTT_EVENT_COMPLETE },
	{ OUI_GOOGLE, GOOGLE_RSSI_MONITOR_EVENT },
	{ OUI_GOOGLE, GSCAN_EVENT_COMPLETE_SCAN },
	{ OUI_GOOGLE, GSCAN_EVENT_HOTLIST_RESULTS_LOST }

};


int bes2600_set_vendor_command(struct wiphy *wiphy)
{

	wiphy->vendor_commands = bes2600_own_commands;
	wiphy->n_vendor_commands = ARRAY_SIZE(bes2600_own_commands);
	wiphy->vendor_events	= bes2600_own_events;
	wiphy->n_vendor_events	= ARRAY_SIZE(bes2600_own_events);

	return 0;
}

int bes2600_vendor_command_detach(struct wiphy *wiphy)
{

	wiphy->vendor_commands  = NULL;
	wiphy->vendor_events    = NULL;
	wiphy->n_vendor_commands = 0;
	wiphy->n_vendor_events  = 0;

	return 0;
}




