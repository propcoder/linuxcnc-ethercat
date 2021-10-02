//
//    Copyright (C) 2011 Sascha Ittner <sascha.ittner@modusoft.de>
//                  2021 Marius Alksnys <marius.alksnys@gmail.com>
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

#include "lcec.h"
#include "lcec_ncti16.h"

typedef struct {
  hal_bit_t *in;
  hal_bit_t *in_not;
} lcec_ncti16_chan_t;

typedef struct {
  lcec_ncti16_chan_t chans[LCEC_NCTI16_CHANS];
  unsigned int pdo_os;
} lcec_ncti16_data_t;

static const lcec_pindesc_t slave_pins[] = {
  { HAL_BIT, HAL_OUT, offsetof(lcec_ncti16_chan_t, in), "%s.%s.%s.din-%d" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_ncti16_chan_t, in_not), "%s.%s.%s.din-%d-not" },
  { HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL }
};

void lcec_ncti16_read(struct lcec_slave *slave, long period);

int lcec_ncti16_init(int comp_id, struct lcec_slave *slave, ec_pdo_entry_reg_t *pdo_entry_regs) {
  lcec_master_t *master = slave->master;
  lcec_ncti16_data_t *hal_data;
  lcec_ncti16_chan_t *ch;
  int i;
  int err;

  // initialize callbacks
  slave->proc_read = lcec_ncti16_read;

  // alloc hal memory
  if ((hal_data = hal_malloc(sizeof(lcec_ncti16_data_t))) == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "hal_malloc() for slave %s.%s failed\n", master->name, slave->name);
    return -EIO;
  }
  memset(hal_data, 0, sizeof(lcec_ncti16_data_t));
  slave->hal_data = hal_data;

  // initialize PDO entry
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3001, 0x01, &hal_data->pdo_os, NULL);
  
  // initialize and export pins
  for (i=0; i<LCEC_NCTI16_CHANS; i++) {
    if ((err = lcec_pin_newf_list(&hal_data->chans[i], slave_pins, LCEC_MODULE_NAME, master->name, slave->name, i)) != 0) {
      return err;
    }
  }

  return 0;
}

void lcec_ncti16_read(struct lcec_slave *slave, long period) {
  lcec_master_t *master = slave->master;
  lcec_ncti16_data_t *hal_data = (lcec_ncti16_data_t *) slave->hal_data;
  uint8_t *pd = master->process_data;
  lcec_ncti16_chan_t *chan;
  int i;
  uint32_t s;
  bool b;

  // wait for slave to be operational
  if (!slave->state.operational) {
    return;
  }

  s = EC_READ_U32(&pd[hal_data->pdo_os]);
  // set inputs
  for (i=0, chan=&hal_data->chans[0]; i<LCEC_NCTI16_CHANS; i++, chan++, s>>=1) {
    b = s & 1;
    *(chan->in) = b;
    *(chan->in_not) = !b;
  }
}
