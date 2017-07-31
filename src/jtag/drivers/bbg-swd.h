/***************************************************************************
 *   Copyright (C) 2016  Flying Stone Technology                           *
 *   Author: NIIBE Yutaka <gniibe@fsij.org>                                *
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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifndef BBG_SWD_H
#define BBG_SWD_H

#include <jtag/swd.h>

struct bbg_swd_interface {
	/* low level callbacks (for bbg_swd)
	 */
	int (*read)(void);
	void (*write)(int tck, int tms, int tdi);
	void (*reset)(int trst, int srst);
	void (*blink)(int on);
	int (*swdio_read)(void);
	void (*swdio_drive)(bool on);
};

const struct swd_driver bbg_swd_swd;

int bbg_swd_execute_queue(void);

extern struct jtag_interface bbg_swd_interface;
int bbg_swd_swd_switch_seq(enum swd_special_seq seq);

#endif /* BBG_SWD_H */
