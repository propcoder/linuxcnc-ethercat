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
#include "lcec_nctmk1x.h"

typedef union {
    struct {
      uint8_t
        start           : 1,
        stop            : 1,
        mst_lock        : 1,
                        : 1,
        spindle_forward : 1,
        spindle_reverse : 1,
        spindle_stop    : 1;
      union {
        uint8_t jog_btns;
        struct {
          uint8_t
          x_minus     : 1,
          a_plus      : 1,
          x_plus      : 1,
          y_minus     : 1,
          y_plus      : 1,
          z_minus     : 1,
          a_minus     : 1,
          z_plus      : 1;
        } jog;
      };
      uint8_t
        ref_point       : 1,
        modes           : 6,
//       union {
//         uint8_t modes : 6;
//         struct {
//           uint8_t
//           handwheel  : 1,
//           jog_incr   : 1,
//           jog_cont   : 1,
//                      : 1,
//           mdi        : 1,
//           aut        : 1;
//         } mode;
//       };
//       uint8_t
        edit            : 1,
        program_test    : 1,
        motion_enable   : 1,
        dry_run         : 1,
        block_restart   : 1,
        block_return    : 1,
        optional_stop   : 1,
        block_delete    : 1,
        step            : 1,
        incr_btns       : 4,
        s_over_plus     : 1,
        s_over_none     : 1,
        s_over_minus    : 1,
        jog_rapid       : 1,
        machine_on      : 1,
        fork_ccw        : 1,
        tool_release    : 1,
        orient          : 1,
        lube            : 1,
        fork_cw         : 1,
        tool_clamp      : 1,
        flood           : 1,
                        : 6,
        reset           : 1,
        clear_faults    : 1,
                        : 6,
        key_switch      : 1;
    } f;
    uint64_t all;
} keys_leds_t;

enum Modes_e {
  M_HW    = 1,
  M_INCR  = 2,
  M_JOG   = 4,
  M_MDI  = 16,
  M_AUTO = 32,
};

typedef struct {
  hal_bit_t *enable;
  hal_bit_t *gpin;
  hal_bit_t *gpout;
  hal_bit_t *jog_minus;
  hal_bit_t *jog_plus;
  hal_bit_t *incr_minus;
  hal_bit_t *incr_plus;
  
  unsigned int move_pdo_os;
  unsigned int enable_pdo_os;
  unsigned int incr_pdo_os;
  unsigned int gpout_pdo_os;
  unsigned int gpin_pdo_os;
} lcec_nctmk1x_axis_t;

typedef struct {
  lcec_nctmk1x_axis_t axes[LCEC_NCTMK1X_CHANS];
  hal_s32_t *hw_counts, *f_over_counts, *s_over_counts, *s_over_min, *s_over_none, *s_over_max, *s_over_increment, *jog_mask;
  hal_bit_t
    *nc_ready,
    *start,
    *running,
    *stop,
    *stopped,
    *mst_lock,
    *spindle_forward,
    *spindle_runs_forward,
    *spindle_reverse,
    *spindle_runs_reverse,
    *spindle_stop,
    *ref_point,
    *handwheel_mode,
    *incr_mode,
    *jog_mode,
    *mdi_mode, *is_mdi,
    *auto_mode, *is_auto,
    *edit,
    *program_test,
    *motion_enable, *motion_enabled, *motion_inhibit,
    *dry_run,
    *block_restart,
    *block_return,
    *optional_stop_is_on, *optional_stop_on, *optional_stop_off,
    *block_delete_is_on, *block_delete_on, *block_delete_off,
    *step,
    *machine_on, *machine_is_on,
    *fork_ccw, *fork_is_ccw, *fork_cw, *fork_is_cw, 
    *tool_release, *tool_released, *tool_clamp, *tool_clamped,
    *orient, *oriented,
    *lube, *lube_is_on,
    *flood_on, *flood_off, *flood_is_on,
    *reset, *clear_faults, *key_switch;
  hal_float_t *increment, *normal_jog_speed, *rapid_jog_speed, *jog_speed;
  
  hal_bit_t do_init;
  keys_leds_t k_old, leds;
  
  unsigned int leds_pdo_os;
  unsigned int nc_ready_pdo_os;
  unsigned int btns_pdo_os;
  unsigned int override_pdo_os;
  unsigned int mainspindle_pdo_os;
  unsigned int hwmove_pdo_os;
  unsigned int beep_pdo_os;
  unsigned int ibtnserno_pdo_os;
} lcec_nctmk1x_data_t;

static const lcec_pindesc_t slave_pins[] = {
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, nc_ready), "%s.%s.%s.nc-ready" },
  { HAL_S32, HAL_OUT, offsetof(lcec_nctmk1x_data_t, hw_counts), "%s.%s.%s.hw-counts" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, start), "%s.%s.%s.start" },
  { HAL_BIT, HAL_IN, offsetof(lcec_nctmk1x_data_t, running), "%s.%s.%s.running" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, stop), "%s.%s.%s.stop" },
  { HAL_BIT, HAL_IN, offsetof(lcec_nctmk1x_data_t, stopped), "%s.%s.%s.stopped" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, mst_lock), "%s.%s.%s.mst_lock" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, spindle_forward), "%s.%s.%s.spindle-forward" },
  { HAL_BIT, HAL_IN, offsetof(lcec_nctmk1x_data_t, spindle_runs_forward), "%s.%s.%s.spindle-runs-forward" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, spindle_reverse), "%s.%s.%s.spindle-reverse" },
  { HAL_BIT, HAL_IN, offsetof(lcec_nctmk1x_data_t, spindle_runs_reverse), "%s.%s.%s.spindle-runs-reverse" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, spindle_stop), "%s.%s.%s.spindle-stop" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, ref_point), "%s.%s.%s.ref-point" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, handwheel_mode), "%s.%s.%s.handwheel-mode" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, incr_mode), "%s.%s.%s.jog-mode-incr" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, jog_mode), "%s.%s.%s.jog-mode-cont" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, mdi_mode), "%s.%s.%s.mdi-mode" },
  { HAL_BIT, HAL_IN, offsetof(lcec_nctmk1x_data_t, is_mdi), "%s.%s.%s.is-mdi" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, auto_mode), "%s.%s.%s.auto-mode" },
  { HAL_BIT, HAL_IN, offsetof(lcec_nctmk1x_data_t, is_auto), "%s.%s.%s.is-auto" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, edit), "%s.%s.%s.edit" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, program_test), "%s.%s.%s.program-test" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, motion_enable), "%s.%s.%s.motion-enable" },
  { HAL_BIT, HAL_IN, offsetof(lcec_nctmk1x_data_t, motion_enabled), "%s.%s.%s.motion-enabled" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, motion_inhibit), "%s.%s.%s.motion-inhibit" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, dry_run), "%s.%s.%s.dry-run" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, block_restart), "%s.%s.%s.block-restart" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, block_return), "%s.%s.%s.block-return" },
  { HAL_BIT, HAL_IN, offsetof(lcec_nctmk1x_data_t, optional_stop_is_on), "%s.%s.%s.optional-stop-is-on" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, optional_stop_on), "%s.%s.%s.optional-stop-on" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, optional_stop_off), "%s.%s.%s.optional-stop-off" },
  { HAL_BIT, HAL_IN, offsetof(lcec_nctmk1x_data_t, block_delete_is_on), "%s.%s.%s.block-delete-is-on" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, block_delete_on), "%s.%s.%s.block-delete-on" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, block_delete_off), "%s.%s.%s.block-delete-off" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, step), "%s.%s.%s.step" },
  { HAL_FLOAT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, increment), "%s.%s.%s.increment" },
  { HAL_S32, HAL_OUT, offsetof(lcec_nctmk1x_data_t, f_over_counts), "%s.%s.%s.f-over-counts" },
  { HAL_S32, HAL_IN, offsetof(lcec_nctmk1x_data_t, s_over_increment), "%s.%s.%s.s-over-increment" },
  { HAL_S32, HAL_OUT, offsetof(lcec_nctmk1x_data_t, s_over_counts), "%s.%s.%s.s-over-counts" },
  { HAL_S32, HAL_IN, offsetof(lcec_nctmk1x_data_t, s_over_min), "%s.%s.%s.s-over-min" },
  { HAL_S32, HAL_IN, offsetof(lcec_nctmk1x_data_t, s_over_none), "%s.%s.%s.s-over-none" },
  { HAL_S32, HAL_IN, offsetof(lcec_nctmk1x_data_t, s_over_max), "%s.%s.%s.s-over-max" },
  { HAL_U32, HAL_IO, offsetof(lcec_nctmk1x_data_t, jog_mask), "%s.%s.%s.jog-mask" },
  { HAL_FLOAT, HAL_IN, offsetof(lcec_nctmk1x_data_t, normal_jog_speed), "%s.%s.%s.normal-jog-speed" },
  { HAL_FLOAT, HAL_IN, offsetof(lcec_nctmk1x_data_t, rapid_jog_speed), "%s.%s.%s.rapid-jog-speed" },
  { HAL_FLOAT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, jog_speed), "%s.%s.%s.jog-speed" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, machine_on), "%s.%s.%s.machine-on" },
  { HAL_BIT, HAL_IN, offsetof(lcec_nctmk1x_data_t, machine_is_on), "%s.%s.%s.machine-is-on" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, fork_ccw), "%s.%s.%s.fork-ccw" },
  { HAL_BIT, HAL_IN, offsetof(lcec_nctmk1x_data_t, fork_is_ccw), "%s.%s.%s.fork-is-ccw" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, fork_cw), "%s.%s.%s.fork-cw" },
  { HAL_BIT, HAL_IN, offsetof(lcec_nctmk1x_data_t, fork_is_cw), "%s.%s.%s.fork-is-cw" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, tool_release), "%s.%s.%s.tool-release" },
  { HAL_BIT, HAL_IN, offsetof(lcec_nctmk1x_data_t, tool_released), "%s.%s.%s.tool-released" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, tool_clamp), "%s.%s.%s.tool-clamp" },
  { HAL_BIT, HAL_IN, offsetof(lcec_nctmk1x_data_t, tool_clamped), "%s.%s.%s.tool-clamped" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, orient), "%s.%s.%s.orient" },
  { HAL_BIT, HAL_IN, offsetof(lcec_nctmk1x_data_t, oriented), "%s.%s.%s.oriented" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, lube), "%s.%s.%s.lube" },
  { HAL_BIT, HAL_IN, offsetof(lcec_nctmk1x_data_t, lube_is_on), "%s.%s.%s.lube-is-on" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, flood_on), "%s.%s.%s.flood-on" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, flood_off), "%s.%s.%s.flood-off" },
  { HAL_BIT, HAL_IN, offsetof(lcec_nctmk1x_data_t, flood_is_on), "%s.%s.%s.flood-is-on" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, reset), "%s.%s.%s.reset" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, clear_faults), "%s.%s.%s.clear-faults" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_data_t, key_switch), "%s.%s.%s.key-switch" },
  { HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL }
};

static const lcec_pindesc_t axis_pins[] = {
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_axis_t, enable), "%s.%s.%s.%d.enable" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_axis_t, gpin), "%s.%s.%s.%d.gpin" },
  { HAL_BIT, HAL_IN, offsetof(lcec_nctmk1x_axis_t, gpout), "%s.%s.%s.%d.gpout" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_axis_t, jog_minus), "%s.%s.%s.%d.jog-minus" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_axis_t, jog_plus), "%s.%s.%s.%d.jog-plus" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_axis_t, incr_minus), "%s.%s.%s.%d.incr-minus" },
  { HAL_BIT, HAL_OUT, offsetof(lcec_nctmk1x_axis_t, incr_plus), "%s.%s.%s.%d.incr-plus" },
  { HAL_TYPE_UNSPECIFIED, HAL_DIR_UNSPECIFIED, -1, NULL }
};

static ec_pdo_entry_info_t lcec_nctmk1x_out_0[] = {
   {0x3101, 0x01, 64}, // kb.leds
   {0x3102, 0x02,  8}, // ncready
};

static ec_pdo_entry_info_t lcec_nctmk1x_in_0[] = {
   {0x3001, 0x01, 64}, // kb.keys
   {0x3002, 0x02,  8}, // F override
   {0x3003, 0x03,  8}, // MainSpindle
   {0x3004, 0x04,  8}, // HwMove
   {0x3005, 0x05,  8}, // HandwheelMove0, not functioning
   {0x3006, 0x06,  8}, // HandwheelMove1, not functioning
   {0x3007, 0x07,  8}, // HandwheelMove2, not functioning
   {0x3008, 0x08,  8}, // HandwheelMove3
   {0x3009, 0x09,  8}, // HandwheelAxis0, not functioning
   {0x3010, 0x10,  8}, // HandwheelAxis1, not functioning
   {0x3011, 0x11,  8}, // HandwheelAxis2, not functioning
   {0x3012, 0x12,  8}, // HandwheelAxis3
   {0x3013, 0x13,  8}, // HandwheelIncr0, not functioning
   {0x3014, 0x14,  8}, // HandwheelIncr1, not functioning
   {0x3015, 0x15,  8}, // HandwheelIncr2, not functioning
   {0x3016, 0x16,  8}, // HandwheelIncr3
   {0x3017, 0x17,  8}, // Beep
   {0x3018, 0x18, 64}, // iButtonSerialNumber
};

static ec_pdo_entry_info_t lcec_nctmk1x_out_1[] = {
   {0x3201, 0x01,  8}, // GpOut1
   {0x3202, 0x02,  8}, // GpOut2
   {0x3203, 0x03,  8}, // GpOut3
   {0x3204, 0x04,  8}, // GpOut4
};

static ec_pdo_entry_info_t lcec_nctmk1x_in_1[] = {
   {0x3301, 0x01,  8}, // GpIn1
   {0x3302, 0x02,  8}, // GpIn2
   {0x3303, 0x03,  8}, // GpIn3
   {0x3304, 0x04,  8}, // GpIn4
};

static ec_pdo_info_t lcec_nctmk1x_pdos_out[] = {
    {0x1600,  2, lcec_nctmk1x_out_0},
    {0x1601,  4, lcec_nctmk1x_out_1}
};

static ec_pdo_info_t lcec_nctmk1x_pdos_in[] = {
    {0x1A01,  18, lcec_nctmk1x_in_0},
    {0x1A02,  4, lcec_nctmk1x_in_1}
};

static ec_sync_info_t lcec_nctmk1x_syncs[] = {
    {0, EC_DIR_OUTPUT, 1, lcec_nctmk1x_pdos_out, EC_WD_ENABLE},
    {1, EC_DIR_INPUT,  1, lcec_nctmk1x_pdos_in, EC_WD_DISABLE},
    {2, EC_DIR_INPUT,  1, lcec_nctmk1x_pdos_out + 1, EC_WD_DISABLE},
    {3, EC_DIR_INPUT,  1, lcec_nctmk1x_pdos_in + 1, EC_WD_DISABLE},
    {0xff}
};

void lcec_nctmk1x_read(struct lcec_slave *slave, long period);
void lcec_nctmk1x_write(struct lcec_slave *slave, long period);

int lcec_nctmk1x_init(int comp_id, struct lcec_slave *slave, ec_pdo_entry_reg_t *pdo_entry_regs) {
  lcec_master_t *master = slave->master;
  lcec_nctmk1x_data_t *hal_data;
  int i;
  lcec_nctmk1x_axis_t *axis;
  int err;
  char pfx[HAL_NAME_LEN];

  // initialize callbacks
  slave->proc_read = lcec_nctmk1x_read;
  slave->proc_write = lcec_nctmk1x_write;

  // alloc hal memory
  if ((hal_data = hal_malloc(sizeof(lcec_nctmk1x_data_t))) == NULL) {
    rtapi_print_msg(RTAPI_MSG_ERR, LCEC_MSG_PFX "hal_malloc() for slave %s.%s failed\n", master->name, slave->name);
    return -EIO;
  }
  memset(hal_data, 0, sizeof(lcec_nctmk1x_data_t));
  slave->hal_data = hal_data;

  // initialize sync info
  slave->sync_info = lcec_nctmk1x_syncs;

  // initialize global data
  hal_data->do_init = true;
  
  // initialize PDO entries
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3101, 0x01, &hal_data->leds_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3102, 0x02, &hal_data->nc_ready_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3001, 0x01, &hal_data->btns_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3002, 0x02, &hal_data->override_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3003, 0x03, &hal_data->mainspindle_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3004, 0x04, &hal_data->hwmove_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3005, 0x05, &hal_data->axes[0].move_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3006, 0x06, &hal_data->axes[1].move_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3007, 0x07, &hal_data->axes[2].move_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3008, 0x08, &hal_data->axes[3].move_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3009, 0x09, &hal_data->axes[0].enable_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3010, 0x10, &hal_data->axes[1].enable_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3011, 0x11, &hal_data->axes[2].enable_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3012, 0x12, &hal_data->axes[3].enable_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3013, 0x13, &hal_data->axes[0].incr_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3014, 0x14, &hal_data->axes[1].incr_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3015, 0x15, &hal_data->axes[2].incr_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3016, 0x16, &hal_data->axes[3].incr_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3017, 0x17, &hal_data->beep_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3018, 0x18, &hal_data->ibtnserno_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3201, 0x01, &hal_data->axes[0].gpout_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3202, 0x02, &hal_data->axes[1].gpout_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3203, 0x03, &hal_data->axes[2].gpout_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3204, 0x04, &hal_data->axes[3].gpout_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3301, 0x01, &hal_data->axes[0].gpin_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3302, 0x02, &hal_data->axes[1].gpin_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3303, 0x03, &hal_data->axes[2].gpin_pdo_os, NULL);
  LCEC_PDO_INIT(pdo_entry_regs, slave->index, slave->vid, slave->pid, 0x3304, 0x04, &hal_data->axes[3].gpin_pdo_os, NULL);
  
  for (i=0; i<LCEC_NCTMK1X_CHANS; i++) {
    axis = &hal_data->axes[i];
    // export pins
    if ((err = lcec_pin_newf_list(axis, axis_pins, LCEC_MODULE_NAME, master->name, slave->name, i)) != 0) return err;
  }
  
  // export pins
  if ((err = lcec_pin_newf_list(hal_data, slave_pins, LCEC_MODULE_NAME, master->name, slave->name)) != 0) return err;

  
  // initialize pins
  *hal_data->normal_jog_speed = 1200;
  *hal_data->jog_mask = 255;
  *hal_data->rapid_jog_speed = 6000;
  *hal_data->increment = 1.0f;
  *hal_data->motion_enable = true;
  return 0;
}

void lcec_nctmk1x_read(struct lcec_slave *slave, long period) {
  lcec_master_t *master = slave->master;
  if (!slave->state.operational) return; // wait for slave to be operational
  lcec_nctmk1x_data_t *hal_data = (lcec_nctmk1x_data_t *) slave->hal_data;
  uint8_t b, *pd = master->process_data;
  keys_leds_t k;
  int i;
  lcec_nctmk1x_axis_t *axis;
  
  // handle initialization
//   if (hal_data->do_init) {
//     hal_data->do_init = false;
//     *hal_data->s_over_counts = *hal_data->s_over_none;
//   }
  *hal_data->nc_ready = EC_READ_BIT(&pd[hal_data->nc_ready_pdo_os], 0);
  // Handwheel increment
  b = 0b111 & EC_READ_U8(&pd[hal_data->axes[3].incr_pdo_os]);
  if(*hal_data->handwheel_mode)
    *hal_data->hw_counts += EC_READ_S8(&pd[hal_data->hwmove_pdo_os]) * (b==4 ? 100 : (b==2 ? 10 : (b==1 ? 1 : 0)));
  // Axis selector
  b = EC_READ_U8(&pd[hal_data->axes[3].enable_pdo_os]);
  for (i=0; i<LCEC_NCTMK1X_AXES; i++) *hal_data->axes[i].enable = (b == (1 << i));
  // Feed override
  *hal_data->f_over_counts = EC_READ_U8(&pd[hal_data->override_pdo_os]);
  if(*hal_data->f_over_counts > 30) *hal_data->f_over_counts = 30;
  // Spindle override
  *hal_data->s_over_counts = EC_READ_U8(&pd[hal_data->mainspindle_pdo_os]);
  if(*hal_data->s_over_counts > 10) *hal_data->s_over_counts = 10;
  *hal_data->s_over_counts += 5;
  // Keys
  k.all = EC_READ_U64(&pd[hal_data->btns_pdo_os]);
  if(k.all != hal_data->k_old.all) {
    *hal_data->start = k.f.start;
    *hal_data->stop = k.f.stop;
    *hal_data->mst_lock = k.f.mst_lock;
    *hal_data->spindle_stop      = k.f.spindle_stop;
    if(!*hal_data->spindle_stop) {
      *hal_data->spindle_forward   = k.f.spindle_forward;
      *hal_data->spindle_reverse   = k.f.spindle_reverse;
    }
    // Modes
    if(k.f.modes) {
      *hal_data->auto_mode      = false;
      *hal_data->mdi_mode       = false;
      *hal_data->jog_mode       = false;
      *hal_data->incr_mode      = false;
      *hal_data->handwheel_mode = false;
      switch(k.f.modes) {
        case M_AUTO:
          *hal_data->auto_mode      = true;
          break;
        case M_MDI:
          *hal_data->mdi_mode       = true;
          break;
        case M_JOG:
          *hal_data->jog_mode       = true;
          break;
        case M_INCR:
          *hal_data->incr_mode      = true;
          break;
        case M_HW:
          *hal_data->handwheel_mode = true;
      }
    }
    *hal_data->edit              = k.f.edit;
    k.f.jog_btns &= *hal_data->jog_mask;
    if(*hal_data->incr_mode) {
      *hal_data->axes[0].jog_minus  = 0;
      *hal_data->axes[3].jog_plus   = 0;
      *hal_data->axes[0].jog_plus   = 0;
      *hal_data->axes[1].jog_minus  = 0;
      *hal_data->axes[1].jog_plus   = 0;
      *hal_data->axes[2].jog_minus  = 0;
      *hal_data->axes[3].jog_minus  = 0;
      *hal_data->axes[2].jog_plus   = 0;
      *hal_data->axes[0].incr_minus = k.f.jog.x_minus;
      *hal_data->axes[3].incr_plus  = k.f.jog.a_plus;
      *hal_data->axes[0].incr_plus  = k.f.jog.x_plus;
      *hal_data->axes[1].incr_minus = k.f.jog.y_minus;
      *hal_data->axes[1].incr_plus  = k.f.jog.y_plus;
      *hal_data->axes[2].incr_minus = k.f.jog.z_minus;
      *hal_data->axes[3].incr_minus = k.f.jog.a_minus;
      *hal_data->axes[2].incr_plus  = k.f.jog.z_plus;
    }
    else {
      *hal_data->axes[0].incr_minus = 0;
      *hal_data->axes[3].incr_plus  = 0;
      *hal_data->axes[0].incr_plus  = 0;
      *hal_data->axes[1].incr_minus = 0;
      *hal_data->axes[1].incr_plus  = 0;
      *hal_data->axes[2].incr_minus = 0;
      *hal_data->axes[3].incr_minus = 0;
      *hal_data->axes[2].incr_plus  = 0;
      *hal_data->axes[0].jog_minus  = k.f.jog.x_minus;
      *hal_data->axes[3].jog_plus   = k.f.jog.a_plus;
      *hal_data->axes[0].jog_plus   = k.f.jog.x_plus;
      *hal_data->axes[1].jog_minus  = k.f.jog.y_minus;
      *hal_data->axes[1].jog_plus   = k.f.jog.y_plus;
      *hal_data->axes[2].jog_minus  = k.f.jog.z_minus;
      *hal_data->axes[3].jog_minus  = k.f.jog.a_minus;
      *hal_data->axes[2].jog_plus   = k.f.jog.z_plus;
    }
    *hal_data->ref_point         = k.f.ref_point;
    *hal_data->program_test      = k.f.program_test;
    *hal_data->motion_enable    ^= k.f.motion_enable;
    *hal_data->motion_inhibit    = !*hal_data->motion_enable;
    *hal_data->dry_run           = k.f.dry_run;
    *hal_data->block_restart     = k.f.block_restart;
    *hal_data->block_return      = k.f.block_return;
    b = k.f.optional_stop;
    *hal_data->optional_stop_on = !*hal_data->optional_stop_is_on && b;
    *hal_data->optional_stop_off = *hal_data->optional_stop_is_on && b;
    b = k.f.block_delete;
    *hal_data->block_delete_on = !*hal_data->block_delete_is_on && b;
    *hal_data->block_delete_off = *hal_data->block_delete_is_on && b;
    *hal_data->step = k.f.step;
    uint8_t incr_btns = k.f.incr_btns;
    hal_data->leds.f.incr_btns = incr_btns;
    if(incr_btns & 1) *hal_data->increment = 0.001f;
    else if(incr_btns & 2) *hal_data->increment = 0.01f;
    else if(incr_btns & 4) *hal_data->increment = 0.1f;
    else if(incr_btns & 8) *hal_data->increment = 1.0f;
   
// Software solution for s_override    
//     if(k.f.s_over_none) *hal_data->s_over_counts = *hal_data->s_over_none;
//     else {
//       if(k.f.s_over_minus) {
//           *hal_data->s_over_counts -= *hal_data->s_over_increment;
//           if(*hal_data->s_over_counts < *hal_data->s_over_min) *hal_data->s_over_counts = *hal_data->s_over_min;
//       }
//       if(k.f.s_over_plus) {
//           *hal_data->s_over_counts += *hal_data->s_over_increment;
//           if(*hal_data->s_over_counts > *hal_data->s_over_max) *hal_data->s_over_counts = *hal_data->s_over_max;
//       }
//     }
    
    *hal_data->jog_mask &= 255;
    *hal_data->jog_speed    = k.f.jog_rapid ? *hal_data->rapid_jog_speed : *hal_data->normal_jog_speed;
    *hal_data->machine_on   = k.f.machine_on;
    *hal_data->fork_ccw     = k.f.fork_ccw;
    *hal_data->tool_release = k.f.tool_release;
    *hal_data->orient       ^= k.f.orient;
    *hal_data->lube         = k.f.lube;
    *hal_data->fork_cw      = k.f.fork_cw;
    *hal_data->tool_clamp   = k.f.tool_clamp;
    b = k.f.flood;
    *hal_data->flood_on = !*hal_data->flood_is_on && b;
    *hal_data->flood_off = *hal_data->flood_is_on && b;
    *hal_data->reset        = k.f.reset;
    *hal_data->clear_faults = k.f.clear_faults;
    *hal_data->key_switch = k.f.key_switch;
    hal_data->k_old.all = k.all;
  }
}

void lcec_nctmk1x_write(struct lcec_slave *slave, long period) {
  lcec_master_t *master = slave->master;
  lcec_nctmk1x_data_t *hal_data = (lcec_nctmk1x_data_t *) slave->hal_data;
  uint8_t *pd = master->process_data, u;
  int i;
  lcec_nctmk1x_axis_t *axis;
  
  // write outputs
//   for (i=0; i<LCEC_NCTMK1X_CHANS; i++) {
//     axis = &hal_data->axes[i];
//     if(*hal_data->hw_counts >= 0 && *hal_data->hw_counts < 32) *hal_data->leds0 = 1 << *hal_data->hw_counts;
//     else *hal_data->leds0 = 0;
//     if(*hal_data->hw_counts >= 32 && *hal_data->hw_counts < 64) *hal_data->leds1 = 1 << (*hal_data->hw_counts - 32);
//     else *hal_data->leds1 = 0;

  hal_data->leds.f.start           = *hal_data->running;
  hal_data->leds.f.stop            = *hal_data->stopped;
//   hal_data->leds.f.mst_lock        = mst_lock;
  hal_data->leds.f.spindle_forward = *hal_data->spindle_runs_forward;
  hal_data->leds.f.spindle_reverse = *hal_data->spindle_runs_reverse;
  hal_data->leds.f.spindle_stop    = !hal_data->leds.f.spindle_forward && !hal_data->leds.f.spindle_reverse;
  hal_data->leds.f.jog_btns        = (*hal_data->incr_mode || *hal_data->jog_mode) ? *hal_data->jog_mask : 0;
//   hal_data->leds.f.ref_point       = ref_point;
  hal_data->leds.f.modes =
    *hal_data->handwheel_mode ? M_HW   : 0 |
    *hal_data->incr_mode      ? M_INCR : 0 |
    *hal_data->jog_mode       ? M_JOG  : 0 |
    *hal_data->is_mdi         ? M_MDI  : 0 |
    *hal_data->is_auto        ? M_AUTO : 0;
//   hal_data->leds.f.edit            = *hal_data->edit;
//   hal_data->leds.f.program_test    = *hal_data->program_test;
  hal_data->leds.f.motion_enable   = *hal_data->motion_enable;
//   hal_data->leds.f.dry_run         = *hal_data->dry_run;
//   hal_data->leds.f.block_restart   = *hal_data->block_restart;
//   hal_data->leds.f.block_return    = *hal_data->block_return;
  hal_data->leds.f.optional_stop   = *hal_data->optional_stop_is_on;
  hal_data->leds.f.block_delete    = *hal_data->block_delete_is_on;
  hal_data->leds.f.step            = *hal_data->running;
  if(*hal_data->incr_mode) {
    i = round(*hal_data->increment * 1000);
    hal_data->leds.f.incr_btns       = (i == 1000) ? 8 : ((i == 100) ? 4 : ((i == 10) ? 2 : 1));
  }
  else hal_data->leds.f.incr_btns = 0;
// // Software spindle override solution
//   hal_data->leds.f.s_over_plus     = *hal_data->s_over_counts > *hal_data->s_over_none;
//   hal_data->leds.f.s_over_none     = *hal_data->s_over_counts == *hal_data->s_over_none;
//   hal_data->leds.f.s_over_minus    = *hal_data->s_over_counts < *hal_data->s_over_none;
  // Hardware spindle override solution
  hal_data->leds.f.s_over_plus     = *hal_data->s_over_counts > 10;
  hal_data->leds.f.s_over_none     = *hal_data->s_over_counts == 10;
  hal_data->leds.f.s_over_minus    = *hal_data->s_over_counts < 10;
  hal_data->leds.f.jog_rapid       = *hal_data->jog_mode && hal_data->leds.f.jog_btns;
  hal_data->leds.f.machine_on      = *hal_data->machine_is_on;
  hal_data->leds.f.fork_ccw        = *hal_data->fork_is_ccw;
  hal_data->leds.f.tool_release    = *hal_data->tool_released;
  hal_data->leds.f.orient          = *hal_data->oriented;
  hal_data->leds.f.lube            = *hal_data->lube_is_on;
  hal_data->leds.f.fork_cw         = *hal_data->fork_is_cw;
  hal_data->leds.f.tool_clamp      = *hal_data->tool_clamped;
  hal_data->leds.f.flood           = *hal_data->flood_is_on;
  hal_data->leds.f.reset           = *hal_data->reset;
  hal_data->leds.f.clear_faults    = *hal_data->clear_faults;
 
  EC_WRITE_U64(&pd[hal_data->leds_pdo_os], hal_data->leds.all);
}
