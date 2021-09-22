//
//    Copyright (C) 2018 Sascha Ittner <sascha.ittner@modusoft.de>
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
#include "lcec_nctttlac2.h"

typedef struct {
  hal_bit_t *fault;
  
  hal_float_t *vel_cmd;
  hal_bit_t *index_enable;
  hal_u32_t *index_pulse_position;
  hal_bit_t *fault_reset;
  
  hal_s32_t *raw_count;
  hal_s32_t *count;
  hal_float_t *pos_scale, *vel_scale;
  hal_float_t *enc_pos;
  hal_u32_t *enc_resolution;
  hal_s32_t *enc_max_speed;
  hal_u32_t *message_code;
  hal_float_t *message_data;
  
  unsigned int vel_cmd_pdo_os;
  unsigned int control_pdo_os;
  unsigned int index_pulse_position_pdo_os;
  unsigned int enc_resolution_pdo_os;
  unsigned int enc_max_speed_pdo_os;
  
  unsigned int enc_count_pdo_os;
  unsigned int enc_latch_pdo_os;
  unsigned int status_pdo_os;
  unsigned int error_pdo_os;
  unsigned int message_code_pdo_os;
  unsigned int message_data_pdo_os;
  
  int do_init;
  int32_t last_count;
  double scale_old, scale_rcp, vel_cmd_multiplier, vel_scale_old;
  
} lcec_nctttlac2_chan_t;

typedef struct {
  lcec_nctttlac2_chan_t chans[LCEC_NCTTTLAC2_CHANS];
  int last_operational;
} lcec_nctttlac2_data_t;

static const lcec_pindesc_t slave_pins[] = {
  
  { HAL_S32, HAL_OUT, offsetof(lcec_nctttlac2_chan_t, count), "%s.%s.%s.%d.count" },
  { HAL_FLOAT, HAL_OUT, offsetof(lcec_nctttlac2_chan_t, enc_pos), "%s.%s.%s.%d.enc-pos" },
  { HAL_FLOAT, HAL_IO, offsetof(lcec_nctttlac2_chan_t, pos_scale), "%s.%s.%s.%d.enc-pos-scale" },
  { HAL_FLOAT, HAL_IO, offsetof(lcec_nctttlac2_chan_t, vel_scale), "%s.%s.%s.%d.vel-scale" },
  { HAL_FLOAT, HAL_IN, offsetof(lcec_nctttlac2_chan_t, vel_cmd), "%s.%s.%s.%d.vel-cmd" },
  { HAL_BIT, HAL_IN, offsetof(lcec_nctttlac2_chan_t, index_enable), "%s.%s.%s.%d.index-enable" },
  { HAL_U32, HAL_IO, offsetof(lcec_nctttlac2_chan_t, index_pulse_position), "%s.%s.%s.%d.index-pulse-position" },
  { HAL_BIT, HAL_IN, offsetof(lcec_nctttlac2_chan_t, fault_reset), "%s.%s.%s.%d.fault-reset" },
  { HAL_U32, HAL_IN, offsetof(lcec_nctttlac2_chan_t, message_code), "%s.%s.%s.%d.message-code" },
  { HAL_FLOAT, HAL_IN, offsetof(lcec_nctttlac2_chan_t, message_data), "%s.%s.%s.%d.message-data" },
  { HAL_U32, HAL_IN, offsetof(lcec_nctttlac2_chan_t, enc_resolution), "%s.%s.%s.%d.enc-resolution" },
  { HAL_S32, HAL_IN, offsetof(lcec_nctttlac2_chan_t, enc_max_speed), "%s.%s.%s.%d.enc-max-speed" },
  
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctttlac2_chan_t, fault), "%s.%s.%s.%d.fault" },
  { HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL }
};

static ec_pdo_entry_info_t lcec_nctttlac2_in_0[] = {
   {0x3001, 0x01, 32}, // ActualPosition
   {0x3001, 0x02, 32}, // ReferencePosition
   {0x3001, 0x03, 32}, // StatusDword
   {0x3001, 0x04, 32}, // ErrorDword
   {0x3001, 0x05, 32}, // MessageCode
   {0x3001, 0x06, 32}, // MessageData
   {0x3001, 0x07, 16}, // TimeStamp
};

static ec_pdo_entry_info_t lcec_nctttlac2_in_1[] = {
   {0x3002, 0x01, 32}, // ActualPosition
   {0x3002, 0x02, 32}, // ReferencePosition
   {0x3002, 0x03, 32}, // StatusDword
   {0x3002, 0x04, 32}, // ErrorDword
   {0x3002, 0x05, 32}, // MessageCode
   {0x3002, 0x06, 32}, // MessageData
   {0x3002, 0x07, 16}, // TimeStamp
};

static ec_pdo_entry_info_t lcec_nctttlac2_out_0[] = {
   {0x3101, 0x01, 32}, // SpeedSetValue
   {0x3101, 0x02, 32}, // ControlDword
   {0x3101, 0x03, 16}, // RefPulsePosition
   {0x3101, 0x04, 16}, // EncoderResolution
   {0x3101, 0x05, 16}, // EncoderMaxSpeed
};

static ec_pdo_entry_info_t lcec_nctttlac2_out_1[] = {
   {0x3102, 0x01, 32}, // SpeedSetValue
   {0x3102, 0x02, 32}, // ControlDword
   {0x3102, 0x03, 16}, // RefPulsePosition
   {0x3102, 0x04, 16}, // EncoderResolution
   {0x3102, 0x05, 16}, // EncoderMaxSpeed
};

static ec_pdo_info_t lcec_nctttlac2_pdos_out[] = {
    {0x1600,  5, lcec_nctttlac2_out_0},
    {0x1601,  5, lcec_nctttlac2_out_1}
};

static ec_pdo_info_t lcec_nctttlac2_pdos_in[] = {
    {0x1A01,  7, lcec_nctttlac2_in_0},
    {0x1A02,  7, lcec_nctttlac2_in_1}
};

static ec_sync_info_t lcec_nctttlac2_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL},
    {1, EC_DIR_INPUT,  0, NULL},
    {2, EC_DIR_OUTPUT, 2, lcec_nctttlac2_pdos_out},
    {3, EC_DIR_INPUT,  2, lcec_nctttlac2_pdos_in},
    {0xff}
};

void lcec_nctttlac2_check_scales(lcec_nctttlac2_chan_t *hal_data);

void lcec_nctttlac2_read(struct lcec_slave *slave, long period);
void lcec_nctttlac2_write(struct lcec_slave *slave, long period);

int lcec_nctttlac2_init(int comp_id, struct lcec_slave *slave, ec_pdo_entry_reg_t *pdo_entry_regs) {
  lcec_master_t *master = slave->master;
  lcec_nctttlac2_data_t *hal_data;
  int i;
  lcec_nctttlac2_chan_t *chan;
  int err;
  char pfx[HAL_NAME_LEN];

  // initialize callbacks
  slave->proc_read = lcec_nctttlac2_read;
  slave->proc_write = lcec_nctttlac2_write;

  // alloc hal memory
  if ((hal_data = hal_malloc(sizeof(lcec_nctttlac2_data_t))) == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "hal_malloc() for slave %s.%s failed\n", master->name, slave->name);
    return -EIO;
  }
  memset(hal_data, 0, sizeof(lcec_nctttlac2_data_t));
  slave->hal_data = hal_data;

  // initialize sync info
  slave->sync_info = lcec_nctttlac2_syncs;

  // initialize global data
  
  // initialize pins
  for (i=0; i<LCEC_NCTTTLAC2_CHANS; i++) {
    chan = &hal_data->chans[i];

    // initialize PDO entries
    LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3101 + i, 0x01, &chan->vel_cmd_pdo_os, NULL);
    LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3101 + i, 0x02, &chan->control_pdo_os, NULL);
    LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3101 + i, 0x03, &chan->index_pulse_position_pdo_os, NULL);
    LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3101 + i, 0x04, &chan->enc_resolution_pdo_os, NULL);
    LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3101 + i, 0x05, &chan->enc_max_speed_pdo_os, NULL);
    
    LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3001 + i, 0x01, &chan->enc_count_pdo_os, NULL);
    LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3001 + i, 0x02, &chan->enc_latch_pdo_os, NULL);
    LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3001 + i, 0x03, &chan->status_pdo_os, NULL);
    LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3001 + i, 0x04, &chan->error_pdo_os, NULL);
    LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3001 + i, 0x05, &chan->message_code_pdo_os, NULL);
    LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3001 + i, 0x06, &chan->message_data_pdo_os, NULL);

    // export pins
    if ((err = lcec_pin_newf_list(chan, slave_pins, LCEC_MODULE_NAME, master->name, slave->name, i)) != 0) {
      return err;
    }

    // initialize pins
    *(chan->pos_scale) = 1.0;

    // initialize variables
    chan->do_init = 1;
    chan->last_count = 0;
    chan->scale_old = *(chan->pos_scale) + 1.0;
    chan->vel_scale_old = *(chan->vel_scale) + 1.0;
    *(chan->fault_reset) = 1;
    *(chan->enc_resolution) = 1000;
    *(chan->enc_max_speed) = 6000;
  }
  return 0;
}

void lcec_nctttlac2_check_scales(lcec_nctttlac2_chan_t *hal_data) {
  // check for change in pos_scale value
  if (*(hal_data->pos_scale) != hal_data->scale_old) {
    // pos_scale value has changed, test and update it
    if ((*(hal_data->pos_scale) < 1e-20) && (*(hal_data->pos_scale) > -1e-20)) {
      // value too small, divide by zero is a bad thing
      *(hal_data->pos_scale) = 1.0;
    }
    // save new pos_scale to detect future changes
    hal_data->scale_old = *(hal_data->pos_scale);
    // we actually want the reciprocal
    hal_data->scale_rcp = 1.0 / *(hal_data->pos_scale);
  }
  if (*(hal_data->vel_scale) != hal_data->vel_scale_old) {
    // vel_scale value has changed, test and update it
    if ((*(hal_data->vel_scale) < 1e-20) && (*(hal_data->vel_scale) > -1e-20)) {
      // value too small, divide by zero is a bad thing
      *(hal_data->vel_scale) = 1.0;
    }
    // save new vel_scale to detect future changes
    hal_data->vel_scale_old = *(hal_data->vel_scale);
    // we actually want final multiplier
    hal_data->vel_cmd_multiplier = (double)LCEC_NCTTTLAC2_VELCMD_MAXVAL / *(hal_data->vel_scale);
  }
}

void lcec_nctttlac2_read(struct lcec_slave *slave, long period) {
  lcec_master_t *master = slave->master;
  lcec_nctttlac2_data_t *hal_data = (lcec_nctttlac2_data_t *) slave->hal_data;
  uint8_t *pd = master->process_data;
  uint32_t raw_status;
  int i;
  lcec_nctttlac2_chan_t *chan;
  int32_t raw_count, raw_delta;

  // wait for slave to be operational
  if (!slave->state.operational) {
    hal_data->last_operational = 0;
    for (i=0; i<LCEC_NCTTTLAC2_CHANS; i++) {
      chan = &hal_data->chans[i];
      *(chan->fault) = 1;
    } 
    return;
  }
  
  // check inputs
  for (i=0; i<LCEC_NCTTTLAC2_CHANS; i++) {
    chan = &hal_data->chans[i];

    lcec_nctttlac2_check_scales(chan);
    
    // get bit states
    raw_status = EC_READ_U32(&pd[chan->status_pdo_os]);
    // check fault
    if(*(chan->fault) && (raw_status & LCEC_NCTTTLAC2_STATUS_ERR_CLEAR_ACK)) {
      *(chan->fault) = 0;
    }
    
    // read raw values
    raw_count = EC_READ_S32(&pd[chan->enc_count_pdo_os]);
    *(chan->fault) |= EC_READ_U32(&pd[chan->error_pdo_os]) & LCEC_NCTTTLAC2_ERROR_ENCODER;
    
    // check for operational change of slave
    if (!hal_data->last_operational) {
      chan->last_count = raw_count;
    }

    // handle initialization
    if (chan->do_init) {
      chan->do_init = 0;
      chan->last_count = raw_count;
      *(chan->count) = raw_count;
      *(chan->fault_reset) = 0;
    }

    // handle index
    if (raw_status & LCEC_NCTTTLAC2_STATUS_LATCH_ACK) { // latch valid
      chan->last_count = EC_READ_S32(&pd[chan->enc_latch_pdo_os]);;
      *(chan->count) = 0;
      *(chan->index_enable) = 0;
    }

    // compute net counts
    *(chan->count) += raw_count - chan->last_count;
    chan->last_count = raw_count;

    // scale count to make floating point position
    *(chan->enc_pos) = *(chan->count) * chan->scale_rcp;
  }
  hal_data->last_operational = 1;
}

void lcec_nctttlac2_write(struct lcec_slave *slave, long period) {
  lcec_master_t *master = slave->master;
  lcec_nctttlac2_data_t *hal_data = (lcec_nctttlac2_data_t *) slave->hal_data;
  uint8_t *pd = master->process_data;
  int i;
  lcec_nctttlac2_chan_t *chan;
  uint16_t raw_ctrl;
  double vel_cmd_raw;
  
  // write outputs
  for (i=0; i<LCEC_NCTTTLAC2_CHANS; i++) {
    chan = &hal_data->chans[i];
    
    // build control word
    raw_ctrl = 0;
    if (*(chan->index_enable)) {
      raw_ctrl |= LCEC_NCTTTLAC2_CONTROL_LATCH_REQUEST;
    }
    if(*(chan->fault) && *(chan->fault_reset)) {
      raw_ctrl |= LCEC_NCTTTLAC2_CONTROL_ERROR_CLEAR;
    }
    
    // set output data
    EC_WRITE_U16(&pd[chan->control_pdo_os], raw_ctrl);
    
    // set pulse position
    *(chan->index_pulse_position) &= 0b11;
    EC_WRITE_U16(&pd[chan->index_pulse_position_pdo_os], *(chan->index_pulse_position));
    
    // form CAN message
    EC_WRITE_U32(&pd[chan->message_code_pdo_os], *(chan->message_code));
    EC_WRITE_REAL(&pd[chan->message_data_pdo_os], *(chan->message_data));
    
    // set encoder and tacho parameters
    EC_WRITE_U16(&pd[chan->enc_resolution_pdo_os], *(chan->enc_resolution));
    EC_WRITE_S16(&pd[chan->enc_max_speed_pdo_os], *(chan->enc_max_speed));
    
    // set velocity command
    vel_cmd_raw = *(chan->vel_cmd) * chan->vel_cmd_multiplier;
    if (vel_cmd_raw > (double)LCEC_NCTTTLAC2_VELCMD_MAXVAL) {
      vel_cmd_raw = (double)LCEC_NCTTTLAC2_VELCMD_MAXVAL;
    } else if (vel_cmd_raw < (double)-LCEC_NCTTTLAC2_VELCMD_MAXVAL) {
      vel_cmd_raw = (double)-LCEC_NCTTTLAC2_VELCMD_MAXVAL;
    }
    EC_WRITE_S32(&pd[chan->vel_cmd_pdo_os], (int32_t)vel_cmd_raw);
  }
}
