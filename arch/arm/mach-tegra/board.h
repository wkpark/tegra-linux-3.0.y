/*
 * arch/arm/mach-tegra/board.h
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * Author:
 *	Colin Cross <ccross@google.com>
 *	Erik Gilling <konkers@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __MACH_TEGRA_BOARD_H
#define __MACH_TEGRA_BOARD_H

#include <linux/types.h>

struct tegra_suspend_platform_data;

void __init tegra_init_early(void);
void __init tegra_mc_init(void);
void __init tegra_common_init(void);
void __init tegra_map_common_io(void);
void __init tegra_init_irq(void);
void __init tegra_init_clock(void);
void __init tegra_init_suspend(struct tegra_suspend_platform_data *plat);

#ifdef CONFIG_CPU_IDLE
void __init tegra_init_idle(struct tegra_suspend_platform_data *plat);
#else
#define tegra_init_idle(plat) (0)
#endif

#ifdef CONFIG_CPU_FREQ
int tegra_start_dvfsd(void);
#else
#define tegra_start_dvfsd() (0)
#endif

#define TEGRA_ALL_REVS (~0ul)
bool tegra_chip_compare(u32 chip, u32 major_rev, u32 minor_rev);

#define tegra_is_ap20_a03() tegra_chip_compare(0x20, 0x1, 0x3)

bool tegra_is_ap20_a03p(void);

extern struct sys_timer tegra_timer;

#ifdef CONFIG_MACH_VENTANA
extern int __init ventana_setup_wifi(void);
int ventana_wifi_status_register(void (*callback)(int card_present,
	void  *dev_id), void *dev_id);
#endif


#ifdef CONFIG_MACH_STAR
#ifdef CONFIG_STAR_HIDDEN_RESET
#define RAM_CONSOLE_RESERVED_SIZE 2
#define RAM_RESERVED_SIZE 3*512*SZ_1K
#else
#define RAM_CONSOLE_RESERVED_SIZE 1
#define RAM_RESERVED_SIZE 100*SZ_1K
#endif
#define DEFAULT_CARVEOUT_SIZE 128
#define STAR_DEFAULT_RAM_CONSOLE_BASE ((512 - DEFAULT_CARVEOUT_SIZE - RAM_CONSOLE_RESERVED_SIZE)*SZ_1M)
#ifdef CONFIG_MACH_STAR_TMUS
#define STAR_RAM_CONSOLE_SIZE	(128*SZ_1K) 	
#else
#define STAR_RAM_CONSOLE_SIZE	(512*SZ_1K)
#endif
#endif /* CONFIG_MACH_STAR */

#endif
