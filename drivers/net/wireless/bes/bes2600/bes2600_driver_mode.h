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
#ifndef _BES2600_DRIVER_MODE_CMD_
#define _BES2600_DRIVER_MODE_CMD_

#ifdef CONFIG_FW_LOADER
#define BES2600_LOAD_BOOT_NAME		"bes2600/best2002_fw_boot_sdio.bin"
#define BES2600_LOAD_FW_NAME		"bes2600/best2002_fw_sdio.bin"
#define BES2600_LOAD_NOSIGNAL_FW_NAME	"bes2600/best2002_fw_sdio_nosignal.bin"
#define BES2600_LOAD_BTRF_FW_NAME	"bes2600/best2002_fw_sdio_btrf.bin"
#else
#define BES2600_LOAD_BOOT_NAME		"/lib/firmware/bes2600/best2002_fw_boot_sdio.bin"
#define BES2600_LOAD_FW_NAME		"/lib/firmware/bes2600/best2002_fw_sdio.bin"
#define BES2600_LOAD_NOSIGNAL_FW_NAME	"/lib/firmware/bes2600/best2002_fw_sdio_nosignal.bin"
#define BES2600_LOAD_BTRF_FW_NAME	"/lib/firmware/bes2600/best2002_fw_sdio_btrf.bin"
#endif

#endif /* _BES2600_DRIVER_MODE_CMD_ */
