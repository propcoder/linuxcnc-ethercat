//
//    Copyright (C) 2019 Sascha Ittner <sascha.ittner@modusoft.de>
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
#include "lcec_pana6b.h"

#include "lcec_class_enc.h"

#define PANA6B_PULSES_PER_REV_DEFLT (1 << 23)
#define PANA6B_FAULT_AUTORESET_DELAY_NS 100000000LL

typedef struct {
  hal_float_t *pos_cmd;
  hal_float_t *pos_ferr;
  hal_bit_t *fault;
  hal_bit_t *fault_reset;
  hal_bit_t *enable;
  hal_u32_t *error_code;

  hal_bit_t *din_not;
  hal_bit_t *din_pot;
  hal_bit_t *din_home;
  hal_bit_t *din_vicl;
  hal_bit_t *din_ret;
  hal_bit_t *din_ext1;
  hal_bit_t *din_ext2;
  hal_bit_t *din_mon3;
  hal_bit_t *din_mon4;
  hal_bit_t *din_stop;
  hal_bit_t *din_inp;
  hal_bit_t *din_retst;

  hal_bit_t *stat_switchon_ready;
  hal_bit_t *stat_switched_on;
  hal_bit_t *stat_op_enabled;
  hal_bit_t *stat_fault;
  hal_bit_t *stat_volt_enabled;
  hal_bit_t *stat_quick_stop;
  hal_bit_t *stat_switchon_disabled;
  hal_bit_t *stat_warning;
  hal_bit_t *stat_remote;

  hal_u32_t *modes_display;

  hal_float_t pos_scale;
  hal_u32_t pprev;
  hal_bit_t auto_fault_reset;

  lcec_class_enc_data_t enc;
  
  hal_float_t pos_scale_old;
  double pos_scale_rcpt;

  unsigned int error_pdo_os;
  unsigned int status_pdo_os;
  unsigned int modes_display_pdo_os;
  unsigned int curr_pos_pdo_os;
  unsigned int curr_ferr_pdo_os;
  unsigned int latch_stat_pdo_os;
  unsigned int latch_pos1_pdo_os;
  unsigned int din_pdo_os;
  unsigned int control_pdo_os;
  unsigned int modes_pdo_os;
  unsigned int target_pos_pdo_os;
  unsigned int latch_fnk_os;

  hal_bit_t enable_old;
  long long auto_fault_reset_delay;

} lcec_pana6b_data_t;

static const lcec_pindesc_t slave_pins[] = {
  { HAL_FLOAT, HAL_IN, offsetof(lcec_pana6b_data_t, pos_cmd), "%s.%s.%s.pos-cmd" },
  { HAL_FLOAT, HAL_OUT, offsetof(lcec_pana6b_data_t, pos_ferr), "%s.%s.%s.pos-ferr" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_pana6b_data_t, fault), "%s.%s.%s.fault" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_pana6b_data_t, fault_reset), "%s.%s.%s.fault-reset" },
  { HAL_BIT, HAL_IN, offsetof(lcec_pana6b_data_t, enable), "%s.%s.%s.enable" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_pana6b_data_t, stat_switchon_ready), "%s.%s.%s.stat-switchon-ready" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_pana6b_data_t, stat_switched_on), "%s.%s.%s.stat-switched-on" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_pana6b_data_t, stat_op_enabled), "%s.%s.%s.stat-op-enabled" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_pana6b_data_t, stat_fault), "%s.%s.%s.stat-fault" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_pana6b_data_t, stat_volt_enabled), "%s.%s.%s.stat-volt-enabled" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_pana6b_data_t, stat_quick_stop), "%s.%s.%s.stat-quick-stop" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_pana6b_data_t, stat_switchon_disabled), "%s.%s.%s.stat-switchon-disabled" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_pana6b_data_t, stat_warning), "%s.%s.%s.stat-warning" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_pana6b_data_t, stat_remote), "%s.%s.%s.stat-remote" },
  { HAL_U32, HAL_OUT, offsetof(lcec_pana6b_data_t, modes_display), "%s.%s.%s.modes-display" },
  { HAL_U32, HAL_OUT, offsetof(lcec_pana6b_data_t, error_code), "%s.%s.%s.error-code" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_pana6b_data_t, din_not), "%s.%s.%s.din-not" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_pana6b_data_t, din_pot), "%s.%s.%s.din-pot" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_pana6b_data_t, din_home), "%s.%s.%s.din-home" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_pana6b_data_t, din_vicl), "%s.%s.%s.din-vicl" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_pana6b_data_t, din_ret), "%s.%s.%s.din-ret" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_pana6b_data_t, din_ext1), "%s.%s.%s.din-ext1" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_pana6b_data_t, din_ext2), "%s.%s.%s.din-ext2" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_pana6b_data_t, din_mon3), "%s.%s.%s.din-mon3" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_pana6b_data_t, din_mon4), "%s.%s.%s.din-mon4" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_pana6b_data_t, din_stop), "%s.%s.%s.din-stop" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_pana6b_data_t, din_inp), "%s.%s.%s.din-inp" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_pana6b_data_t, din_retst), "%s.%s.%s.din-retst" },
  { HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL }
};

static const lcec_pindesc_t slave_params[] = {
  { HAL_FLOAT, HAL_RW, offsetof(lcec_pana6b_data_t, pos_scale), "%s.%s.%s.pos-scale" },
  { HAL_U32, HAL_RW, offsetof(lcec_pana6b_data_t, pprev), "%s.%s.%s.pulses-per-rev" },
  { HAL_BIT, HAL_RW, offsetof(lcec_pana6b_data_t, auto_fault_reset), "%s.%s.%s.auto-fault-reset" },
  { HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL }
};

static ec_pdo_entry_info_t lcec_pana6b_in[] = {
   {0x603F, 0x00, 16}, // Error Code (U16)
   {0x6041, 0x00, 16}, // Status Word (U16)
   {0x6061, 0x00,  8}, // Modes of Operation Display (U8)
   {0x6064, 0x00, 32}, // Current Position (S32)
   {0x60B9, 0x00, 16}, // Latch Status (U16)
   {0x60BA, 0x00, 32}, // Latch Pos1 (S32)
   {0x60F4, 0x00, 32}, // Current Following Error (S32)
   {0x60FD, 0x00, 32}  // Digital Inputs (U32)
};

static ec_pdo_entry_info_t lcec_pana6b_out[] = {
   {0x6040, 0x00, 16}, // Control Word (U16)
   {0x6060, 0x00,  8}, // Modes of Operation (U8)
   {0x607A, 0x00, 32}, // Target Position (S32)
   {0x60B8, 0x00, 16}, // Latch Function (U16)
};

static ec_pdo_info_t lcec_pana6b_pdos_out[] = {
    {0x1600,  4, lcec_pana6b_out}
};

static ec_pdo_info_t lcec_pana6b_pdos_in[] = {
    {0x1a00, 8, lcec_pana6b_in}
};

static ec_sync_info_t lcec_pana6b_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL},
    {1, EC_DIR_INPUT,  0, NULL},
    {2, EC_DIR_OUTPUT, 1, lcec_pana6b_pdos_out, EC_WD_ENABLE},
    {3, EC_DIR_INPUT,  1, lcec_pana6b_pdos_in},
    {0xff}
};

void lcec_pana6b_check_scales(lcec_pana6b_data_t *hal_data);

void lcec_pana6b_read(struct lcec_slave *slave, long period);
void lcec_pana6b_write(struct lcec_slave *slave, long period);

int lcec_pana6b_init(int comp_id, struct lcec_slave *slave, ec_pdo_entry_reg_t *pdo_entry_regs) {
  lcec_master_t *master = slave->master;
  lcec_pana6b_data_t *hal_data;
  int err;

  // initialize callbacks
  slave->proc_read = lcec_pana6b_read;
  slave->proc_write = lcec_pana6b_write;

  // alloc hal memory
  if ((hal_data = hal_malloc(sizeof(lcec_pana6b_data_t))) == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "hal_malloc() for slave %s.%s failed\n", master->name, slave->name);
    return -EIO;
  }
  memset(hal_data, 0, sizeof(lcec_pana6b_data_t));
  slave->hal_data = hal_data;

  // set to cyclic synchronous position mode
  if (ecrt_slave_config_sdo8(slave->config, 0x6060, 0x00, 8) != 0) {
    rtapi_print_msg (RTAPI_MSG_ERR, LCEC_MSG_PFX "fail to configure slave %s.%s sdo cyclic synchronous position mode\n", master->name, slave->name);
  }

  // initialize sync info
  slave->sync_info = lcec_pana6b_syncs;

  // initialize POD entries
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x603F, 0x00, &hal_data->error_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6041, 0x00, &hal_data->status_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6061, 0x00, &hal_data->modes_display_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6064, 0x00, &hal_data->curr_pos_pdo_os, NULL);
  
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x60F4, 0x00, &hal_data->curr_ferr_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x60B9, 0x00, &hal_data->latch_stat_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x60BA, 0x00, &hal_data->latch_pos1_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x60FD, 0x00, &hal_data->din_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6040, 0x00, &hal_data->control_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x6060, 0x00, &hal_data->modes_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x607A, 0x00, &hal_data->target_pos_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x60B8, 0x00, &hal_data->latch_fnk_os, NULL);

  // export pins
  if ((err = lcec_pin_newf_list(hal_data, slave_pins, LCEC_MODULE_NAME, master->name, slave->name)) != 0) {
    return err;
  }

  // export parameters
  if ((err = lcec_param_newf_list(hal_data, slave_params, LCEC_MODULE_NAME, master->name, slave->name)) != 0) {
    return err;
  }
  
    // init subclasses
  if ((err = class_enc_init(slave, &hal_data->enc, 32, "enc")) != 0) {
    return err;
  }

  // initialize variables
  hal_data->pos_scale = 1.0;
  hal_data->pos_scale_old = hal_data->pos_scale + 1.0;
  hal_data->pos_scale_rcpt = 1.0;
  hal_data->pprev = PANA6B_PULSES_PER_REV_DEFLT;
  hal_data->auto_fault_reset = 1;

  return 0;
}

void lcec_pana6b_check_scales(lcec_pana6b_data_t *hal_data) {
  // check for change in scale value
  if (hal_data->pos_scale != hal_data->pos_scale_old) {

    // scale value has changed, test and update it
    if ((hal_data->pos_scale > -1e-20) && (hal_data->pos_scale < 1e-20)) {
      // value too small, divide by zero is a bad thing
      hal_data->pos_scale = 1.0;
    }

    // save new scale to detect future changes
    hal_data->pos_scale_old = hal_data->pos_scale;

    // we actually want the reciprocal
    hal_data->pos_scale_rcpt = 1.0 / hal_data->pos_scale;
  }
}

void lcec_pana6b_read(struct lcec_slave *slave, long period) {
  lcec_master_t *master = slave->master;
  lcec_pana6b_data_t *hal_data = (lcec_pana6b_data_t *) slave->hal_data;
  uint8_t *pd = master->process_data;
  uint16_t status;
  uint32_t din;
  uint32_t pos_cnt;

  // check for change in scale value
  lcec_pana6b_check_scales(hal_data);

  if(!slave->state.operational) {
    *(hal_data->fault) = true;
    *(hal_data->error_code) = 0x100;
    return;
  }
  
  // read status
  status = EC_READ_U16(&pd[hal_data->status_pdo_os]);
  *(hal_data->stat_switchon_ready)    = (status >> 0) & 1;
  *(hal_data->stat_switched_on)       = (status >> 1) & 1;
  *(hal_data->stat_op_enabled)        = (status >> 2) & 1;
  *(hal_data->stat_fault)             = (status >> 3) & 1;
  *(hal_data->stat_volt_enabled)      = (status >> 4) & 1;
  *(hal_data->stat_quick_stop)        = (status >> 5) & 1;
  *(hal_data->stat_switchon_disabled) = (status >> 6) & 1;
  *(hal_data->stat_warning)           = (status >> 7) & 1;
  *(hal_data->stat_remote)            = (status >> 9) & 1;
  
  *(hal_data->modes_display) = EC_READ_U8(&pd[hal_data->modes_display_pdo_os]);

  // read digital inputs
  din = EC_READ_U32(&pd[hal_data->din_pdo_os]);
  *(hal_data->din_not)   = (din >> 0) & 1;
  *(hal_data->din_pot)   = (din >> 1) & 1;
  *(hal_data->din_home)  = (din >> 2) & 1;
  *(hal_data->din_vicl)  = (din >> 17) & 1;
  *(hal_data->din_ret)   = (din >> 18) & 1;
  *(hal_data->din_ext1)  = (din >> 19) & 1;
  *(hal_data->din_ext2)  = (din >> 20) & 1;
  *(hal_data->din_mon3)  = (din >> 21) & 1;
  *(hal_data->din_mon4)  = (din >> 22) & 1;
  *(hal_data->din_stop)  = (din >> 23) & 1;
  *(hal_data->din_inp)   = (din >> 24) & 1;
  *(hal_data->din_retst) = (din >> 25) & 1;

  // read position feedback
  pos_cnt = EC_READ_S32(&pd[hal_data->curr_pos_pdo_os]);
  class_enc_update(&hal_data->enc, hal_data->pprev, hal_data->pos_scale, pos_cnt, 0, 0);

  // read following error
   *(hal_data->pos_ferr) = ((double) EC_READ_S32(&pd[hal_data->curr_ferr_pdo_os])) * hal_data->pos_scale / hal_data->pprev;

  // read error code
  *(hal_data->error_code) = EC_READ_U16(&pd[hal_data->error_pdo_os]);

  // update fault output
  if (hal_data->auto_fault_reset_delay > 0) {
    hal_data->auto_fault_reset_delay -= period;
    *(hal_data->fault) = 0;
  } else {
    *(hal_data->fault) = *(hal_data->stat_fault) && *(hal_data->enable);
  }
}

void lcec_pana6b_write(struct lcec_slave *slave, long period) {
  lcec_master_t *master = slave->master;
  lcec_pana6b_data_t *hal_data = (lcec_pana6b_data_t *) slave->hal_data;
  uint8_t *pd = master->process_data;
  int enable_edge;
  uint16_t control;
  int64_t pos64;

  // detect enable edge
  enable_edge = *(hal_data->enable) && !hal_data->enable_old;
  hal_data->enable_old = *(hal_data->enable);

  // write control register
  control = 0;
  if (*(hal_data->stat_fault)) {
    if (*(hal_data->fault_reset)) {
      control |= (1 << 7); // fault reset
    }
    if (hal_data->auto_fault_reset && enable_edge) {
      hal_data->auto_fault_reset_delay = PANA6B_FAULT_AUTORESET_DELAY_NS;
      control |= (1 << 7); // fault reset
    }
  } else {
    if (*(hal_data->enable)) {
      control |= (1 << 2); // disable quick stop
      control |= (1 << 1); // enable voltage
      if (*(hal_data->stat_switchon_ready)) {
        control |= (1 << 0); // switch on
        if (*(hal_data->stat_switched_on)) {
          control |= (1 << 3); // enable op
          control |= (1 << 9); // change on set-point
        }
      }
    }
  }
  EC_WRITE_U16(&pd[hal_data->control_pdo_os], control);
  EC_WRITE_U8(&pd[hal_data->modes_pdo_os], (1 << 3));

  // write position command
  EC_WRITE_S32(&pd[hal_data->target_pos_pdo_os], (int64_t)(*(hal_data->pos_cmd) * (hal_data->pos_scale_rcpt * hal_data->pprev)));
}
