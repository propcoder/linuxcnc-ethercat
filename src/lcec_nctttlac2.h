//
//    Copyright (C) 2018 Sascha Ittner <sascha.ittner@modusoft.de>
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
//
#ifndef _LCEC_NCTTTLAC2_H_
#define _LCEC_NCTTTLAC2_H_

#include "lcec.h"


#define LCEC_NCTTTLAC2_VID LCEC_NCT_VID
#define LCEC_NCTTTLAC2_PID 0x00000032
// Revision number: 0x00000002

#define LCEC_NCTTTLAC2_CHANS 2
#define LCEC_NCTTTLAC2_PDOS 22

#define LCEC_NCTTTLAC2_STATUS_LATCH_ACK (1 << 3)
#define LCEC_NCTTTLAC2_STATUS_ERR_CLEAR_ACK (1 << 4)
#define LCEC_NCTTTLAC2_CONTROL_ERROR_CLEAR (1 << 9)
#define LCEC_NCTTTLAC2_CONTROL_LATCH_REQUEST (1 << 11)
#define LCEC_NCTTTLAC2_ERROR_ENCODER (1 << 1)

#define LCEC_NCTTTLAC2_VELCMD_MAXVAL 0x1fffff
#define LCEC_NCTTTLAC2_VELCMD_MAX_VOLTAGE 10.0f

int lcec_nctttlac2_init(int comp_id, struct lcec_slave *slave, ec_pdo_entry_reg_t *pdo_entry_regs);

#endif

