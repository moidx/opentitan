// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Register Package auto-generated by `reggen` containing data structure

package gpio_reg_pkg;

  // Param list
  parameter int NumIOs = 32;
  parameter int NumInpPeriodCounters = 8;
  parameter int NumAlerts = 1;

  // Address widths within the block
  parameter int BlockAw = 8;

  // Number of registers for every interface
  parameter int NumRegs = 34;

  ////////////////////////////
  // Typedefs for registers //
  ////////////////////////////

  typedef struct packed {
    logic [31:0] q;
  } gpio_reg2hw_intr_state_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } gpio_reg2hw_intr_enable_reg_t;

  typedef struct packed {
    logic [31:0] q;
    logic        qe;
  } gpio_reg2hw_intr_test_reg_t;

  typedef struct packed {
    logic        q;
    logic        qe;
  } gpio_reg2hw_alert_test_reg_t;

  typedef struct packed {
    logic [31:0] q;
    logic        qe;
  } gpio_reg2hw_direct_out_reg_t;

  typedef struct packed {
    struct packed {
      logic [15:0] q;
      logic        qe;
    } mask;
    struct packed {
      logic [15:0] q;
      logic        qe;
    } data;
  } gpio_reg2hw_masked_out_lower_reg_t;

  typedef struct packed {
    struct packed {
      logic [15:0] q;
      logic        qe;
    } mask;
    struct packed {
      logic [15:0] q;
      logic        qe;
    } data;
  } gpio_reg2hw_masked_out_upper_reg_t;

  typedef struct packed {
    logic [31:0] q;
    logic        qe;
  } gpio_reg2hw_direct_oe_reg_t;

  typedef struct packed {
    struct packed {
      logic [15:0] q;
      logic        qe;
    } mask;
    struct packed {
      logic [15:0] q;
      logic        qe;
    } data;
  } gpio_reg2hw_masked_oe_lower_reg_t;

  typedef struct packed {
    struct packed {
      logic [15:0] q;
      logic        qe;
    } mask;
    struct packed {
      logic [15:0] q;
      logic        qe;
    } data;
  } gpio_reg2hw_masked_oe_upper_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } gpio_reg2hw_intr_ctrl_en_rising_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } gpio_reg2hw_intr_ctrl_en_falling_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } gpio_reg2hw_intr_ctrl_en_lvlhigh_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } gpio_reg2hw_intr_ctrl_en_lvllow_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } gpio_reg2hw_ctrl_en_input_filter_reg_t;

  typedef struct packed {
    logic        q;
  } gpio_reg2hw_hw_straps_data_in_valid_reg_t;

  typedef struct packed {
    logic [31:0] q;
  } gpio_reg2hw_hw_straps_data_in_reg_t;

  typedef struct packed {
    struct packed {
      logic [7:0]  q;
    } prescaler;
    struct packed {
      logic [7:0]  q;
    } input_select;
    struct packed {
      logic        q;
    } polarity;
    struct packed {
      logic        q;
    } continuous_mode;
    struct packed {
      logic        q;
    } enable;
  } gpio_reg2hw_inp_prd_cnt_ctrl_mreg_t;

  typedef struct packed {
    logic [31:0] d;
    logic        de;
  } gpio_hw2reg_intr_state_reg_t;

  typedef struct packed {
    logic [31:0] d;
    logic        de;
  } gpio_hw2reg_data_in_reg_t;

  typedef struct packed {
    logic [31:0] d;
  } gpio_hw2reg_direct_out_reg_t;

  typedef struct packed {
    struct packed {
      logic [15:0] d;
    } data;
    struct packed {
      logic [15:0] d;
    } mask;
  } gpio_hw2reg_masked_out_lower_reg_t;

  typedef struct packed {
    struct packed {
      logic [15:0] d;
    } data;
    struct packed {
      logic [15:0] d;
    } mask;
  } gpio_hw2reg_masked_out_upper_reg_t;

  typedef struct packed {
    logic [31:0] d;
  } gpio_hw2reg_direct_oe_reg_t;

  typedef struct packed {
    struct packed {
      logic [15:0] d;
    } data;
    struct packed {
      logic [15:0] d;
    } mask;
  } gpio_hw2reg_masked_oe_lower_reg_t;

  typedef struct packed {
    struct packed {
      logic [15:0] d;
    } data;
    struct packed {
      logic [15:0] d;
    } mask;
  } gpio_hw2reg_masked_oe_upper_reg_t;

  typedef struct packed {
    logic        d;
    logic        de;
  } gpio_hw2reg_hw_straps_data_in_valid_reg_t;

  typedef struct packed {
    logic [31:0] d;
    logic        de;
  } gpio_hw2reg_hw_straps_data_in_reg_t;

  typedef struct packed {
    struct packed {
      logic        d;
      logic        de;
    } enable;
    struct packed {
      logic        d;
      logic        de;
    } continuous_mode;
    struct packed {
      logic        d;
      logic        de;
    } polarity;
    struct packed {
      logic [7:0]  d;
      logic        de;
    } input_select;
    struct packed {
      logic [7:0]  d;
      logic        de;
    } prescaler;
  } gpio_hw2reg_inp_prd_cnt_ctrl_mreg_t;

  typedef struct packed {
    logic [31:0] d;
    logic        de;
  } gpio_hw2reg_inp_prd_cnt_val_mreg_t;

  // Register -> HW type
  typedef struct packed {
    gpio_reg2hw_intr_state_reg_t intr_state; // [645:614]
    gpio_reg2hw_intr_enable_reg_t intr_enable; // [613:582]
    gpio_reg2hw_intr_test_reg_t intr_test; // [581:549]
    gpio_reg2hw_alert_test_reg_t alert_test; // [548:547]
    gpio_reg2hw_direct_out_reg_t direct_out; // [546:514]
    gpio_reg2hw_masked_out_lower_reg_t masked_out_lower; // [513:480]
    gpio_reg2hw_masked_out_upper_reg_t masked_out_upper; // [479:446]
    gpio_reg2hw_direct_oe_reg_t direct_oe; // [445:413]
    gpio_reg2hw_masked_oe_lower_reg_t masked_oe_lower; // [412:379]
    gpio_reg2hw_masked_oe_upper_reg_t masked_oe_upper; // [378:345]
    gpio_reg2hw_intr_ctrl_en_rising_reg_t intr_ctrl_en_rising; // [344:313]
    gpio_reg2hw_intr_ctrl_en_falling_reg_t intr_ctrl_en_falling; // [312:281]
    gpio_reg2hw_intr_ctrl_en_lvlhigh_reg_t intr_ctrl_en_lvlhigh; // [280:249]
    gpio_reg2hw_intr_ctrl_en_lvllow_reg_t intr_ctrl_en_lvllow; // [248:217]
    gpio_reg2hw_ctrl_en_input_filter_reg_t ctrl_en_input_filter; // [216:185]
    gpio_reg2hw_hw_straps_data_in_valid_reg_t hw_straps_data_in_valid; // [184:184]
    gpio_reg2hw_hw_straps_data_in_reg_t hw_straps_data_in; // [183:152]
    gpio_reg2hw_inp_prd_cnt_ctrl_mreg_t [7:0] inp_prd_cnt_ctrl; // [151:0]
  } gpio_reg2hw_t;

  // HW -> register type
  typedef struct packed {
    gpio_hw2reg_intr_state_reg_t intr_state; // [748:716]
    gpio_hw2reg_data_in_reg_t data_in; // [715:683]
    gpio_hw2reg_direct_out_reg_t direct_out; // [682:651]
    gpio_hw2reg_masked_out_lower_reg_t masked_out_lower; // [650:619]
    gpio_hw2reg_masked_out_upper_reg_t masked_out_upper; // [618:587]
    gpio_hw2reg_direct_oe_reg_t direct_oe; // [586:555]
    gpio_hw2reg_masked_oe_lower_reg_t masked_oe_lower; // [554:523]
    gpio_hw2reg_masked_oe_upper_reg_t masked_oe_upper; // [522:491]
    gpio_hw2reg_hw_straps_data_in_valid_reg_t hw_straps_data_in_valid; // [490:489]
    gpio_hw2reg_hw_straps_data_in_reg_t hw_straps_data_in; // [488:456]
    gpio_hw2reg_inp_prd_cnt_ctrl_mreg_t [7:0] inp_prd_cnt_ctrl; // [455:264]
    gpio_hw2reg_inp_prd_cnt_val_mreg_t [7:0] inp_prd_cnt_val; // [263:0]
  } gpio_hw2reg_t;

  // Register offsets
  parameter logic [BlockAw-1:0] GPIO_INTR_STATE_OFFSET = 8'h 0;
  parameter logic [BlockAw-1:0] GPIO_INTR_ENABLE_OFFSET = 8'h 4;
  parameter logic [BlockAw-1:0] GPIO_INTR_TEST_OFFSET = 8'h 8;
  parameter logic [BlockAw-1:0] GPIO_ALERT_TEST_OFFSET = 8'h c;
  parameter logic [BlockAw-1:0] GPIO_DATA_IN_OFFSET = 8'h 10;
  parameter logic [BlockAw-1:0] GPIO_DIRECT_OUT_OFFSET = 8'h 14;
  parameter logic [BlockAw-1:0] GPIO_MASKED_OUT_LOWER_OFFSET = 8'h 18;
  parameter logic [BlockAw-1:0] GPIO_MASKED_OUT_UPPER_OFFSET = 8'h 1c;
  parameter logic [BlockAw-1:0] GPIO_DIRECT_OE_OFFSET = 8'h 20;
  parameter logic [BlockAw-1:0] GPIO_MASKED_OE_LOWER_OFFSET = 8'h 24;
  parameter logic [BlockAw-1:0] GPIO_MASKED_OE_UPPER_OFFSET = 8'h 28;
  parameter logic [BlockAw-1:0] GPIO_INTR_CTRL_EN_RISING_OFFSET = 8'h 2c;
  parameter logic [BlockAw-1:0] GPIO_INTR_CTRL_EN_FALLING_OFFSET = 8'h 30;
  parameter logic [BlockAw-1:0] GPIO_INTR_CTRL_EN_LVLHIGH_OFFSET = 8'h 34;
  parameter logic [BlockAw-1:0] GPIO_INTR_CTRL_EN_LVLLOW_OFFSET = 8'h 38;
  parameter logic [BlockAw-1:0] GPIO_CTRL_EN_INPUT_FILTER_OFFSET = 8'h 3c;
  parameter logic [BlockAw-1:0] GPIO_HW_STRAPS_DATA_IN_VALID_OFFSET = 8'h 40;
  parameter logic [BlockAw-1:0] GPIO_HW_STRAPS_DATA_IN_OFFSET = 8'h 44;
  parameter logic [BlockAw-1:0] GPIO_INP_PRD_CNT_CTRL_0_OFFSET = 8'h 48;
  parameter logic [BlockAw-1:0] GPIO_INP_PRD_CNT_CTRL_1_OFFSET = 8'h 4c;
  parameter logic [BlockAw-1:0] GPIO_INP_PRD_CNT_CTRL_2_OFFSET = 8'h 50;
  parameter logic [BlockAw-1:0] GPIO_INP_PRD_CNT_CTRL_3_OFFSET = 8'h 54;
  parameter logic [BlockAw-1:0] GPIO_INP_PRD_CNT_CTRL_4_OFFSET = 8'h 58;
  parameter logic [BlockAw-1:0] GPIO_INP_PRD_CNT_CTRL_5_OFFSET = 8'h 5c;
  parameter logic [BlockAw-1:0] GPIO_INP_PRD_CNT_CTRL_6_OFFSET = 8'h 60;
  parameter logic [BlockAw-1:0] GPIO_INP_PRD_CNT_CTRL_7_OFFSET = 8'h 64;
  parameter logic [BlockAw-1:0] GPIO_INP_PRD_CNT_VAL_0_OFFSET = 8'h 68;
  parameter logic [BlockAw-1:0] GPIO_INP_PRD_CNT_VAL_1_OFFSET = 8'h 6c;
  parameter logic [BlockAw-1:0] GPIO_INP_PRD_CNT_VAL_2_OFFSET = 8'h 70;
  parameter logic [BlockAw-1:0] GPIO_INP_PRD_CNT_VAL_3_OFFSET = 8'h 74;
  parameter logic [BlockAw-1:0] GPIO_INP_PRD_CNT_VAL_4_OFFSET = 8'h 78;
  parameter logic [BlockAw-1:0] GPIO_INP_PRD_CNT_VAL_5_OFFSET = 8'h 7c;
  parameter logic [BlockAw-1:0] GPIO_INP_PRD_CNT_VAL_6_OFFSET = 8'h 80;
  parameter logic [BlockAw-1:0] GPIO_INP_PRD_CNT_VAL_7_OFFSET = 8'h 84;

  // Reset values for hwext registers and their fields
  parameter logic [31:0] GPIO_INTR_TEST_RESVAL = 32'h 0;
  parameter logic [31:0] GPIO_INTR_TEST_GPIO_RESVAL = 32'h 0;
  parameter logic [0:0] GPIO_ALERT_TEST_RESVAL = 1'h 0;
  parameter logic [0:0] GPIO_ALERT_TEST_FATAL_FAULT_RESVAL = 1'h 0;
  parameter logic [31:0] GPIO_DIRECT_OUT_RESVAL = 32'h 0;
  parameter logic [31:0] GPIO_MASKED_OUT_LOWER_RESVAL = 32'h 0;
  parameter logic [31:0] GPIO_MASKED_OUT_UPPER_RESVAL = 32'h 0;
  parameter logic [31:0] GPIO_DIRECT_OE_RESVAL = 32'h 0;
  parameter logic [31:0] GPIO_MASKED_OE_LOWER_RESVAL = 32'h 0;
  parameter logic [31:0] GPIO_MASKED_OE_UPPER_RESVAL = 32'h 0;

  // Register index
  typedef enum int {
    GPIO_INTR_STATE,
    GPIO_INTR_ENABLE,
    GPIO_INTR_TEST,
    GPIO_ALERT_TEST,
    GPIO_DATA_IN,
    GPIO_DIRECT_OUT,
    GPIO_MASKED_OUT_LOWER,
    GPIO_MASKED_OUT_UPPER,
    GPIO_DIRECT_OE,
    GPIO_MASKED_OE_LOWER,
    GPIO_MASKED_OE_UPPER,
    GPIO_INTR_CTRL_EN_RISING,
    GPIO_INTR_CTRL_EN_FALLING,
    GPIO_INTR_CTRL_EN_LVLHIGH,
    GPIO_INTR_CTRL_EN_LVLLOW,
    GPIO_CTRL_EN_INPUT_FILTER,
    GPIO_HW_STRAPS_DATA_IN_VALID,
    GPIO_HW_STRAPS_DATA_IN,
    GPIO_INP_PRD_CNT_CTRL_0,
    GPIO_INP_PRD_CNT_CTRL_1,
    GPIO_INP_PRD_CNT_CTRL_2,
    GPIO_INP_PRD_CNT_CTRL_3,
    GPIO_INP_PRD_CNT_CTRL_4,
    GPIO_INP_PRD_CNT_CTRL_5,
    GPIO_INP_PRD_CNT_CTRL_6,
    GPIO_INP_PRD_CNT_CTRL_7,
    GPIO_INP_PRD_CNT_VAL_0,
    GPIO_INP_PRD_CNT_VAL_1,
    GPIO_INP_PRD_CNT_VAL_2,
    GPIO_INP_PRD_CNT_VAL_3,
    GPIO_INP_PRD_CNT_VAL_4,
    GPIO_INP_PRD_CNT_VAL_5,
    GPIO_INP_PRD_CNT_VAL_6,
    GPIO_INP_PRD_CNT_VAL_7
  } gpio_id_e;

  // Register width information to check illegal writes
  parameter logic [3:0] GPIO_PERMIT [34] = '{
    4'b 1111, // index[ 0] GPIO_INTR_STATE
    4'b 1111, // index[ 1] GPIO_INTR_ENABLE
    4'b 1111, // index[ 2] GPIO_INTR_TEST
    4'b 0001, // index[ 3] GPIO_ALERT_TEST
    4'b 1111, // index[ 4] GPIO_DATA_IN
    4'b 1111, // index[ 5] GPIO_DIRECT_OUT
    4'b 1111, // index[ 6] GPIO_MASKED_OUT_LOWER
    4'b 1111, // index[ 7] GPIO_MASKED_OUT_UPPER
    4'b 1111, // index[ 8] GPIO_DIRECT_OE
    4'b 1111, // index[ 9] GPIO_MASKED_OE_LOWER
    4'b 1111, // index[10] GPIO_MASKED_OE_UPPER
    4'b 1111, // index[11] GPIO_INTR_CTRL_EN_RISING
    4'b 1111, // index[12] GPIO_INTR_CTRL_EN_FALLING
    4'b 1111, // index[13] GPIO_INTR_CTRL_EN_LVLHIGH
    4'b 1111, // index[14] GPIO_INTR_CTRL_EN_LVLLOW
    4'b 1111, // index[15] GPIO_CTRL_EN_INPUT_FILTER
    4'b 0001, // index[16] GPIO_HW_STRAPS_DATA_IN_VALID
    4'b 1111, // index[17] GPIO_HW_STRAPS_DATA_IN
    4'b 0111, // index[18] GPIO_INP_PRD_CNT_CTRL_0
    4'b 0111, // index[19] GPIO_INP_PRD_CNT_CTRL_1
    4'b 0111, // index[20] GPIO_INP_PRD_CNT_CTRL_2
    4'b 0111, // index[21] GPIO_INP_PRD_CNT_CTRL_3
    4'b 0111, // index[22] GPIO_INP_PRD_CNT_CTRL_4
    4'b 0111, // index[23] GPIO_INP_PRD_CNT_CTRL_5
    4'b 0111, // index[24] GPIO_INP_PRD_CNT_CTRL_6
    4'b 0111, // index[25] GPIO_INP_PRD_CNT_CTRL_7
    4'b 1111, // index[26] GPIO_INP_PRD_CNT_VAL_0
    4'b 1111, // index[27] GPIO_INP_PRD_CNT_VAL_1
    4'b 1111, // index[28] GPIO_INP_PRD_CNT_VAL_2
    4'b 1111, // index[29] GPIO_INP_PRD_CNT_VAL_3
    4'b 1111, // index[30] GPIO_INP_PRD_CNT_VAL_4
    4'b 1111, // index[31] GPIO_INP_PRD_CNT_VAL_5
    4'b 1111, // index[32] GPIO_INP_PRD_CNT_VAL_6
    4'b 1111  // index[33] GPIO_INP_PRD_CNT_VAL_7
  };

endpackage
