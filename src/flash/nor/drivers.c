// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2009 Zachary T Welch <zw@superlucidity.net>             *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include "imp.h"

extern const struct flash_driver at32f403xx_flash;
extern const struct flash_driver at32f413xx_flash;
extern const struct flash_driver at32f415xx_flash;
extern const struct flash_driver at32f403axx_flash;
extern const struct flash_driver at32f407xx_flash;
extern const struct flash_driver at32wb415xx_flash;

extern const struct flash_driver at32f421xx_flash;
extern const struct flash_driver at32f425xx_flash;
extern const struct flash_driver at32f435xx_flash;
extern const struct flash_driver at32f437xx_flash;
extern const struct flash_driver at32f423xx_flash;
extern const struct flash_driver at32f4xx_flash;
extern const struct flash_driver at32qspi_flash;


/**
 * The list of built-in flash drivers.
 * @todo Make this dynamically extendable with loadable modules.
 */
static const struct flash_driver * const flash_drivers[] = {
	&aduc702x_flash,
	&aducm360_flash,
	&ambiqmicro_flash,
	&at91sam3_flash,
	&at91sam4_flash,
	&at91sam4l_flash,
	&at91sam7_flash,
	&at91samd_flash,
	&ath79_flash,
	&atsame5_flash,
	&atsamv_flash,
	&avr_flash,
	&bluenrgx_flash,
	&cc3220sf_flash,
	&cc26xx_flash,
	&cfi_flash,
	&dsp5680xx_flash,
	&efm32_flash,
	&em357_flash,
	&esirisc_flash,
	&faux_flash,
	&fm3_flash,
	&fm4_flash,
	&fespi_flash,
	&jtagspi_flash,
	&kinetis_flash,
	&kinetis_ke_flash,
	&lpc2000_flash,
	&lpc288x_flash,
	&lpc2900_flash,
	&lpcspifi_flash,
	&max32xxx_flash,
	&mdr_flash,
	&mrvlqspi_flash,
	&msp432_flash,
	&niietcm4_flash,
	&npcx_flash,
	&nrf5_flash,
	&nrf51_flash,
	&numicro_flash,
	&ocl_flash,
	&pic32mx_flash,
	&psoc4_flash,
	&psoc5lp_flash,
	&psoc5lp_eeprom_flash,
	&psoc5lp_nvl_flash,
	&psoc6_flash,
	&qn908x_flash,
	&renesas_rpchf_flash,
	&rp2040_flash,
	&sh_qspi_flash,
	&sim3x_flash,
	&stellaris_flash,
	&stm32f1x_flash,
	&stm32f2x_flash,
	&stm32lx_flash,
	&stm32l4x_flash,
	&stm32h7x_flash,
	&stmsmi_flash,
	&stmqspi_flash,
	&str7x_flash,
	&str9x_flash,
	&str9xpec_flash,
	&swm050_flash,
	&tms470_flash,
	&virtual_flash,
	&xcf_flash,
	&xmc1xxx_flash,
	&xmc4xxx_flash,
	&w600_flash,

	&at32f403xx_flash,
	&at32f413xx_flash,
	&at32f415xx_flash,
	&at32f403axx_flash,
	&at32f407xx_flash,
	&at32wb415xx_flash,
	&at32f421xx_flash,
	&at32f425xx_flash,
	&at32f435xx_flash,
	&at32f437xx_flash,
	&at32f423xx_flash,
	&at32f4xx_flash,
	&at32qspi_flash,
	&rsl10_flash,
	NULL,
};

const struct flash_driver *flash_driver_find_by_name(const char *name)
{
	for (unsigned i = 0; flash_drivers[i]; i++) {
		if (strcmp(name, flash_drivers[i]->name) == 0)
			return flash_drivers[i];
	}
	return NULL;
}
