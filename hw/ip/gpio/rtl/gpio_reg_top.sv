// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Register Top module auto-generated by `reggen`

`include "prim_assert.sv"

module gpio_reg_top
  # (
    parameter bit          EnableRacl           = 1'b0,
    parameter bit          RaclErrorRsp         = 1'b1,
    parameter top_racl_pkg::racl_policy_sel_t RaclPolicySelVec[gpio_reg_pkg::NumRegs] =
      '{gpio_reg_pkg::NumRegs{0}}
  ) (
  input clk_i,
  input rst_ni,
  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o,
  // To HW
  output gpio_reg_pkg::gpio_reg2hw_t reg2hw, // Write
  input  gpio_reg_pkg::gpio_hw2reg_t hw2reg, // Read

  // RACL interface
  input  top_racl_pkg::racl_policy_vec_t racl_policies_i,
  output top_racl_pkg::racl_error_log_t  racl_error_o,

  // Integrity check errors
  output logic intg_err_o
);

  import gpio_reg_pkg::* ;

  localparam int AW = 7;
  localparam int DW = 32;
  localparam int DBW = DW/8;                    // Byte Width

  // register signals
  logic           reg_we;
  logic           reg_re;
  logic [AW-1:0]  reg_addr;
  logic [DW-1:0]  reg_wdata;
  logic [DBW-1:0] reg_be;
  logic [DW-1:0]  reg_rdata;
  logic           reg_error;

  logic          addrmiss, wr_err;

  logic [DW-1:0] reg_rdata_next;
  logic reg_busy;

  tlul_pkg::tl_h2d_t tl_reg_h2d;
  tlul_pkg::tl_d2h_t tl_reg_d2h;


  // incoming payload check
  logic intg_err;
  tlul_cmd_intg_chk u_chk (
    .tl_i(tl_i),
    .err_o(intg_err)
  );

  // also check for spurious write enables
  logic reg_we_err;
  logic [17:0] reg_we_check;
  prim_reg_we_check #(
    .OneHotWidth(18)
  ) u_prim_reg_we_check (
    .clk_i(clk_i),
    .rst_ni(rst_ni),
    .oh_i  (reg_we_check),
    .en_i  (reg_we && !addrmiss),
    .err_o (reg_we_err)
  );

  logic err_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      err_q <= '0;
    end else if (intg_err || reg_we_err) begin
      err_q <= 1'b1;
    end
  end

  // integrity error output is permanent and should be used for alert generation
  // register errors are transactional
  assign intg_err_o = err_q | intg_err | reg_we_err;

  // outgoing integrity generation
  tlul_pkg::tl_d2h_t tl_o_pre;
  tlul_rsp_intg_gen #(
    .EnableRspIntgGen(1),
    .EnableDataIntgGen(1)
  ) u_rsp_intg_gen (
    .tl_i(tl_o_pre),
    .tl_o(tl_o)
  );

  assign tl_reg_h2d = tl_i;
  assign tl_o_pre   = tl_reg_d2h;

  tlul_adapter_reg #(
    .RegAw(AW),
    .RegDw(DW),
    .EnableDataIntgGen(0)
  ) u_reg_if (
    .clk_i  (clk_i),
    .rst_ni (rst_ni),

    .tl_i (tl_reg_h2d),
    .tl_o (tl_reg_d2h),

    .en_ifetch_i(prim_mubi_pkg::MuBi4False),
    .intg_error_o(),

    .we_o    (reg_we),
    .re_o    (reg_re),
    .addr_o  (reg_addr),
    .wdata_o (reg_wdata),
    .be_o    (reg_be),
    .busy_i  (reg_busy),
    .rdata_i (reg_rdata),
    // Translate RACL error to TLUL error if enabled
    .error_i (reg_error | (RaclErrorRsp & racl_error_o.valid))
  );

  // cdc oversampling signals

  assign reg_rdata = reg_rdata_next ;
  assign reg_error = addrmiss | wr_err | intg_err;

  // Define SW related signals
  // Format: <reg>_<field>_{wd|we|qs}
  //        or <reg>_{wd|we|qs} if field == 1 or 0
  logic intr_state_we;
  logic [31:0] intr_state_qs;
  logic [31:0] intr_state_wd;
  logic intr_enable_we;
  logic [31:0] intr_enable_qs;
  logic [31:0] intr_enable_wd;
  logic intr_test_we;
  logic [31:0] intr_test_wd;
  logic alert_test_we;
  logic alert_test_wd;
  logic [31:0] data_in_qs;
  logic direct_out_re;
  logic direct_out_we;
  logic [31:0] direct_out_qs;
  logic [31:0] direct_out_wd;
  logic masked_out_lower_re;
  logic masked_out_lower_we;
  logic [15:0] masked_out_lower_data_qs;
  logic [15:0] masked_out_lower_data_wd;
  logic [15:0] masked_out_lower_mask_wd;
  logic masked_out_upper_re;
  logic masked_out_upper_we;
  logic [15:0] masked_out_upper_data_qs;
  logic [15:0] masked_out_upper_data_wd;
  logic [15:0] masked_out_upper_mask_wd;
  logic direct_oe_re;
  logic direct_oe_we;
  logic [31:0] direct_oe_qs;
  logic [31:0] direct_oe_wd;
  logic masked_oe_lower_re;
  logic masked_oe_lower_we;
  logic [15:0] masked_oe_lower_data_qs;
  logic [15:0] masked_oe_lower_data_wd;
  logic [15:0] masked_oe_lower_mask_qs;
  logic [15:0] masked_oe_lower_mask_wd;
  logic masked_oe_upper_re;
  logic masked_oe_upper_we;
  logic [15:0] masked_oe_upper_data_qs;
  logic [15:0] masked_oe_upper_data_wd;
  logic [15:0] masked_oe_upper_mask_qs;
  logic [15:0] masked_oe_upper_mask_wd;
  logic intr_ctrl_en_rising_we;
  logic [31:0] intr_ctrl_en_rising_qs;
  logic [31:0] intr_ctrl_en_rising_wd;
  logic intr_ctrl_en_falling_we;
  logic [31:0] intr_ctrl_en_falling_qs;
  logic [31:0] intr_ctrl_en_falling_wd;
  logic intr_ctrl_en_lvlhigh_we;
  logic [31:0] intr_ctrl_en_lvlhigh_qs;
  logic [31:0] intr_ctrl_en_lvlhigh_wd;
  logic intr_ctrl_en_lvllow_we;
  logic [31:0] intr_ctrl_en_lvllow_qs;
  logic [31:0] intr_ctrl_en_lvllow_wd;
  logic ctrl_en_input_filter_we;
  logic [31:0] ctrl_en_input_filter_qs;
  logic [31:0] ctrl_en_input_filter_wd;
  logic hw_straps_data_in_valid_qs;
  logic [31:0] hw_straps_data_in_qs;

  // Register instances
  // R[intr_state]: V(False)
  prim_subreg #(
    .DW      (32),
    .SwAccess(prim_subreg_pkg::SwAccessW1C),
    .RESVAL  (32'h0),
    .Mubi    (1'b0)
  ) u_intr_state (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),

    // from register interface
    .we     (intr_state_we),
    .wd     (intr_state_wd),

    // from internal hardware
    .de     (hw2reg.intr_state.de),
    .d      (hw2reg.intr_state.d),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.intr_state.q),
    .ds     (),

    // to register interface (read)
    .qs     (intr_state_qs)
  );


  // R[intr_enable]: V(False)
  prim_subreg #(
    .DW      (32),
    .SwAccess(prim_subreg_pkg::SwAccessRW),
    .RESVAL  (32'h0),
    .Mubi    (1'b0)
  ) u_intr_enable (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),

    // from register interface
    .we     (intr_enable_we),
    .wd     (intr_enable_wd),

    // from internal hardware
    .de     (1'b0),
    .d      ('0),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.intr_enable.q),
    .ds     (),

    // to register interface (read)
    .qs     (intr_enable_qs)
  );


  // R[intr_test]: V(True)
  logic intr_test_qe;
  logic [0:0] intr_test_flds_we;
  assign intr_test_qe = &intr_test_flds_we;
  prim_subreg_ext #(
    .DW    (32)
  ) u_intr_test (
    .re     (1'b0),
    .we     (intr_test_we),
    .wd     (intr_test_wd),
    .d      ('0),
    .qre    (),
    .qe     (intr_test_flds_we[0]),
    .q      (reg2hw.intr_test.q),
    .ds     (),
    .qs     ()
  );
  assign reg2hw.intr_test.qe = intr_test_qe;


  // R[alert_test]: V(True)
  logic alert_test_qe;
  logic [0:0] alert_test_flds_we;
  assign alert_test_qe = &alert_test_flds_we;
  prim_subreg_ext #(
    .DW    (1)
  ) u_alert_test (
    .re     (1'b0),
    .we     (alert_test_we),
    .wd     (alert_test_wd),
    .d      ('0),
    .qre    (),
    .qe     (alert_test_flds_we[0]),
    .q      (reg2hw.alert_test.q),
    .ds     (),
    .qs     ()
  );
  assign reg2hw.alert_test.qe = alert_test_qe;


  // R[data_in]: V(False)
  prim_subreg #(
    .DW      (32),
    .SwAccess(prim_subreg_pkg::SwAccessRO),
    .RESVAL  (32'h0),
    .Mubi    (1'b0)
  ) u_data_in (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),

    // from register interface
    .we     (1'b0),
    .wd     ('0),

    // from internal hardware
    .de     (hw2reg.data_in.de),
    .d      (hw2reg.data_in.d),

    // to internal hardware
    .qe     (),
    .q      (),
    .ds     (),

    // to register interface (read)
    .qs     (data_in_qs)
  );


  // R[direct_out]: V(True)
  logic direct_out_qe;
  logic [0:0] direct_out_flds_we;
  assign direct_out_qe = &direct_out_flds_we;
  prim_subreg_ext #(
    .DW    (32)
  ) u_direct_out (
    .re     (direct_out_re),
    .we     (direct_out_we),
    .wd     (direct_out_wd),
    .d      (hw2reg.direct_out.d),
    .qre    (),
    .qe     (direct_out_flds_we[0]),
    .q      (reg2hw.direct_out.q),
    .ds     (),
    .qs     (direct_out_qs)
  );
  assign reg2hw.direct_out.qe = direct_out_qe;


  // R[masked_out_lower]: V(True)
  logic masked_out_lower_qe;
  logic [1:0] masked_out_lower_flds_we;
  assign masked_out_lower_qe = &masked_out_lower_flds_we;
  //   F[data]: 15:0
  prim_subreg_ext #(
    .DW    (16)
  ) u_masked_out_lower_data (
    .re     (masked_out_lower_re),
    .we     (masked_out_lower_we),
    .wd     (masked_out_lower_data_wd),
    .d      (hw2reg.masked_out_lower.data.d),
    .qre    (),
    .qe     (masked_out_lower_flds_we[0]),
    .q      (reg2hw.masked_out_lower.data.q),
    .ds     (),
    .qs     (masked_out_lower_data_qs)
  );
  assign reg2hw.masked_out_lower.data.qe = masked_out_lower_qe;

  //   F[mask]: 31:16
  prim_subreg_ext #(
    .DW    (16)
  ) u_masked_out_lower_mask (
    .re     (1'b0),
    .we     (masked_out_lower_we),
    .wd     (masked_out_lower_mask_wd),
    .d      (hw2reg.masked_out_lower.mask.d),
    .qre    (),
    .qe     (masked_out_lower_flds_we[1]),
    .q      (reg2hw.masked_out_lower.mask.q),
    .ds     (),
    .qs     ()
  );
  assign reg2hw.masked_out_lower.mask.qe = masked_out_lower_qe;


  // R[masked_out_upper]: V(True)
  logic masked_out_upper_qe;
  logic [1:0] masked_out_upper_flds_we;
  assign masked_out_upper_qe = &masked_out_upper_flds_we;
  //   F[data]: 15:0
  prim_subreg_ext #(
    .DW    (16)
  ) u_masked_out_upper_data (
    .re     (masked_out_upper_re),
    .we     (masked_out_upper_we),
    .wd     (masked_out_upper_data_wd),
    .d      (hw2reg.masked_out_upper.data.d),
    .qre    (),
    .qe     (masked_out_upper_flds_we[0]),
    .q      (reg2hw.masked_out_upper.data.q),
    .ds     (),
    .qs     (masked_out_upper_data_qs)
  );
  assign reg2hw.masked_out_upper.data.qe = masked_out_upper_qe;

  //   F[mask]: 31:16
  prim_subreg_ext #(
    .DW    (16)
  ) u_masked_out_upper_mask (
    .re     (1'b0),
    .we     (masked_out_upper_we),
    .wd     (masked_out_upper_mask_wd),
    .d      (hw2reg.masked_out_upper.mask.d),
    .qre    (),
    .qe     (masked_out_upper_flds_we[1]),
    .q      (reg2hw.masked_out_upper.mask.q),
    .ds     (),
    .qs     ()
  );
  assign reg2hw.masked_out_upper.mask.qe = masked_out_upper_qe;


  // R[direct_oe]: V(True)
  logic direct_oe_qe;
  logic [0:0] direct_oe_flds_we;
  assign direct_oe_qe = &direct_oe_flds_we;
  prim_subreg_ext #(
    .DW    (32)
  ) u_direct_oe (
    .re     (direct_oe_re),
    .we     (direct_oe_we),
    .wd     (direct_oe_wd),
    .d      (hw2reg.direct_oe.d),
    .qre    (),
    .qe     (direct_oe_flds_we[0]),
    .q      (reg2hw.direct_oe.q),
    .ds     (),
    .qs     (direct_oe_qs)
  );
  assign reg2hw.direct_oe.qe = direct_oe_qe;


  // R[masked_oe_lower]: V(True)
  logic masked_oe_lower_qe;
  logic [1:0] masked_oe_lower_flds_we;
  assign masked_oe_lower_qe = &masked_oe_lower_flds_we;
  //   F[data]: 15:0
  prim_subreg_ext #(
    .DW    (16)
  ) u_masked_oe_lower_data (
    .re     (masked_oe_lower_re),
    .we     (masked_oe_lower_we),
    .wd     (masked_oe_lower_data_wd),
    .d      (hw2reg.masked_oe_lower.data.d),
    .qre    (),
    .qe     (masked_oe_lower_flds_we[0]),
    .q      (reg2hw.masked_oe_lower.data.q),
    .ds     (),
    .qs     (masked_oe_lower_data_qs)
  );
  assign reg2hw.masked_oe_lower.data.qe = masked_oe_lower_qe;

  //   F[mask]: 31:16
  prim_subreg_ext #(
    .DW    (16)
  ) u_masked_oe_lower_mask (
    .re     (masked_oe_lower_re),
    .we     (masked_oe_lower_we),
    .wd     (masked_oe_lower_mask_wd),
    .d      (hw2reg.masked_oe_lower.mask.d),
    .qre    (),
    .qe     (masked_oe_lower_flds_we[1]),
    .q      (reg2hw.masked_oe_lower.mask.q),
    .ds     (),
    .qs     (masked_oe_lower_mask_qs)
  );
  assign reg2hw.masked_oe_lower.mask.qe = masked_oe_lower_qe;


  // R[masked_oe_upper]: V(True)
  logic masked_oe_upper_qe;
  logic [1:0] masked_oe_upper_flds_we;
  assign masked_oe_upper_qe = &masked_oe_upper_flds_we;
  //   F[data]: 15:0
  prim_subreg_ext #(
    .DW    (16)
  ) u_masked_oe_upper_data (
    .re     (masked_oe_upper_re),
    .we     (masked_oe_upper_we),
    .wd     (masked_oe_upper_data_wd),
    .d      (hw2reg.masked_oe_upper.data.d),
    .qre    (),
    .qe     (masked_oe_upper_flds_we[0]),
    .q      (reg2hw.masked_oe_upper.data.q),
    .ds     (),
    .qs     (masked_oe_upper_data_qs)
  );
  assign reg2hw.masked_oe_upper.data.qe = masked_oe_upper_qe;

  //   F[mask]: 31:16
  prim_subreg_ext #(
    .DW    (16)
  ) u_masked_oe_upper_mask (
    .re     (masked_oe_upper_re),
    .we     (masked_oe_upper_we),
    .wd     (masked_oe_upper_mask_wd),
    .d      (hw2reg.masked_oe_upper.mask.d),
    .qre    (),
    .qe     (masked_oe_upper_flds_we[1]),
    .q      (reg2hw.masked_oe_upper.mask.q),
    .ds     (),
    .qs     (masked_oe_upper_mask_qs)
  );
  assign reg2hw.masked_oe_upper.mask.qe = masked_oe_upper_qe;


  // R[intr_ctrl_en_rising]: V(False)
  prim_subreg #(
    .DW      (32),
    .SwAccess(prim_subreg_pkg::SwAccessRW),
    .RESVAL  (32'h0),
    .Mubi    (1'b0)
  ) u_intr_ctrl_en_rising (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),

    // from register interface
    .we     (intr_ctrl_en_rising_we),
    .wd     (intr_ctrl_en_rising_wd),

    // from internal hardware
    .de     (1'b0),
    .d      ('0),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.intr_ctrl_en_rising.q),
    .ds     (),

    // to register interface (read)
    .qs     (intr_ctrl_en_rising_qs)
  );


  // R[intr_ctrl_en_falling]: V(False)
  prim_subreg #(
    .DW      (32),
    .SwAccess(prim_subreg_pkg::SwAccessRW),
    .RESVAL  (32'h0),
    .Mubi    (1'b0)
  ) u_intr_ctrl_en_falling (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),

    // from register interface
    .we     (intr_ctrl_en_falling_we),
    .wd     (intr_ctrl_en_falling_wd),

    // from internal hardware
    .de     (1'b0),
    .d      ('0),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.intr_ctrl_en_falling.q),
    .ds     (),

    // to register interface (read)
    .qs     (intr_ctrl_en_falling_qs)
  );


  // R[intr_ctrl_en_lvlhigh]: V(False)
  prim_subreg #(
    .DW      (32),
    .SwAccess(prim_subreg_pkg::SwAccessRW),
    .RESVAL  (32'h0),
    .Mubi    (1'b0)
  ) u_intr_ctrl_en_lvlhigh (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),

    // from register interface
    .we     (intr_ctrl_en_lvlhigh_we),
    .wd     (intr_ctrl_en_lvlhigh_wd),

    // from internal hardware
    .de     (1'b0),
    .d      ('0),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.intr_ctrl_en_lvlhigh.q),
    .ds     (),

    // to register interface (read)
    .qs     (intr_ctrl_en_lvlhigh_qs)
  );


  // R[intr_ctrl_en_lvllow]: V(False)
  prim_subreg #(
    .DW      (32),
    .SwAccess(prim_subreg_pkg::SwAccessRW),
    .RESVAL  (32'h0),
    .Mubi    (1'b0)
  ) u_intr_ctrl_en_lvllow (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),

    // from register interface
    .we     (intr_ctrl_en_lvllow_we),
    .wd     (intr_ctrl_en_lvllow_wd),

    // from internal hardware
    .de     (1'b0),
    .d      ('0),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.intr_ctrl_en_lvllow.q),
    .ds     (),

    // to register interface (read)
    .qs     (intr_ctrl_en_lvllow_qs)
  );


  // R[ctrl_en_input_filter]: V(False)
  prim_subreg #(
    .DW      (32),
    .SwAccess(prim_subreg_pkg::SwAccessRW),
    .RESVAL  (32'h0),
    .Mubi    (1'b0)
  ) u_ctrl_en_input_filter (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),

    // from register interface
    .we     (ctrl_en_input_filter_we),
    .wd     (ctrl_en_input_filter_wd),

    // from internal hardware
    .de     (1'b0),
    .d      ('0),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.ctrl_en_input_filter.q),
    .ds     (),

    // to register interface (read)
    .qs     (ctrl_en_input_filter_qs)
  );


  // R[hw_straps_data_in_valid]: V(False)
  prim_subreg #(
    .DW      (1),
    .SwAccess(prim_subreg_pkg::SwAccessRO),
    .RESVAL  (1'h0),
    .Mubi    (1'b0)
  ) u_hw_straps_data_in_valid (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),

    // from register interface
    .we     (1'b0),
    .wd     ('0),

    // from internal hardware
    .de     (hw2reg.hw_straps_data_in_valid.de),
    .d      (hw2reg.hw_straps_data_in_valid.d),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.hw_straps_data_in_valid.q),
    .ds     (),

    // to register interface (read)
    .qs     (hw_straps_data_in_valid_qs)
  );


  // R[hw_straps_data_in]: V(False)
  prim_subreg #(
    .DW      (32),
    .SwAccess(prim_subreg_pkg::SwAccessRO),
    .RESVAL  (32'h0),
    .Mubi    (1'b0)
  ) u_hw_straps_data_in (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),

    // from register interface
    .we     (1'b0),
    .wd     ('0),

    // from internal hardware
    .de     (hw2reg.hw_straps_data_in.de),
    .d      (hw2reg.hw_straps_data_in.d),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.hw_straps_data_in.q),
    .ds     (),

    // to register interface (read)
    .qs     (hw_straps_data_in_qs)
  );



  logic [17:0] addr_hit;
  top_racl_pkg::racl_role_vec_t racl_role_vec;
  top_racl_pkg::racl_role_t racl_role;

  logic [17:0] racl_addr_hit_read;
  logic [17:0] racl_addr_hit_write;

  if (EnableRacl) begin : gen_racl_role_logic
    // Retrieve RACL role from user bits and one-hot encode that for the comparison bitmap
    assign racl_role = top_racl_pkg::tlul_extract_racl_role_bits(tl_i.a_user.rsvd);

    prim_onehot_enc #(
      .OneHotWidth( $bits(top_racl_pkg::racl_role_vec_t) )
    ) u_racl_role_encode (
      .in_i ( racl_role     ),
      .en_i ( 1'b1          ),
      .out_o( racl_role_vec )
    );
  end else begin : gen_no_racl_role_logic
    assign racl_role     = '0;
    assign racl_role_vec = '0;
  end

  always_comb begin
    addr_hit = '0;
    racl_addr_hit_read  = '0;
    racl_addr_hit_write = '0;
    addr_hit[ 0] = (reg_addr == GPIO_INTR_STATE_OFFSET);
    addr_hit[ 1] = (reg_addr == GPIO_INTR_ENABLE_OFFSET);
    addr_hit[ 2] = (reg_addr == GPIO_INTR_TEST_OFFSET);
    addr_hit[ 3] = (reg_addr == GPIO_ALERT_TEST_OFFSET);
    addr_hit[ 4] = (reg_addr == GPIO_DATA_IN_OFFSET);
    addr_hit[ 5] = (reg_addr == GPIO_DIRECT_OUT_OFFSET);
    addr_hit[ 6] = (reg_addr == GPIO_MASKED_OUT_LOWER_OFFSET);
    addr_hit[ 7] = (reg_addr == GPIO_MASKED_OUT_UPPER_OFFSET);
    addr_hit[ 8] = (reg_addr == GPIO_DIRECT_OE_OFFSET);
    addr_hit[ 9] = (reg_addr == GPIO_MASKED_OE_LOWER_OFFSET);
    addr_hit[10] = (reg_addr == GPIO_MASKED_OE_UPPER_OFFSET);
    addr_hit[11] = (reg_addr == GPIO_INTR_CTRL_EN_RISING_OFFSET);
    addr_hit[12] = (reg_addr == GPIO_INTR_CTRL_EN_FALLING_OFFSET);
    addr_hit[13] = (reg_addr == GPIO_INTR_CTRL_EN_LVLHIGH_OFFSET);
    addr_hit[14] = (reg_addr == GPIO_INTR_CTRL_EN_LVLLOW_OFFSET);
    addr_hit[15] = (reg_addr == GPIO_CTRL_EN_INPUT_FILTER_OFFSET);
    addr_hit[16] = (reg_addr == GPIO_HW_STRAPS_DATA_IN_VALID_OFFSET);
    addr_hit[17] = (reg_addr == GPIO_HW_STRAPS_DATA_IN_OFFSET);

    if (EnableRacl) begin : gen_racl_hit
      for (int unsigned slice_idx = 0; slice_idx < 18; slice_idx++) begin
        racl_addr_hit_read[slice_idx] =
            addr_hit[slice_idx] & (|(racl_policies_i[RaclPolicySelVec[slice_idx]].read_perm
                                      & racl_role_vec));
        racl_addr_hit_write[slice_idx] =
            addr_hit[slice_idx] & (|(racl_policies_i[RaclPolicySelVec[slice_idx]].write_perm
                                      & racl_role_vec));
      end
    end else begin : gen_no_racl
      racl_addr_hit_read  = addr_hit;
      racl_addr_hit_write = addr_hit;
    end
  end

  assign addrmiss = (reg_re || reg_we) ? ~|addr_hit : 1'b0 ;
  // A valid address hit, access, but failed the RACL check
  assign racl_error_o.valid = |addr_hit & ((reg_re & ~|racl_addr_hit_read) |
                                           (reg_we & ~|racl_addr_hit_write));
  assign racl_error_o.racl_role  = racl_role;

  if (EnableRacl) begin : gen_racl_log
    assign racl_error_o.ctn_uid     = top_racl_pkg::tlul_extract_ctn_uid_bits(tl_i.a_user.rsvd);
    assign racl_error_o.read_access = tl_i.a_opcode == tlul_pkg::Get;
  end else begin : gen_no_racl_log
    assign racl_error_o.ctn_uid     = '0;
    assign racl_error_o.read_access = 1'b0;
  end

  // Check sub-word write is permitted
  always_comb begin
    wr_err = (reg_we &
              ((racl_addr_hit_write[ 0] & (|(GPIO_PERMIT[ 0] & ~reg_be))) |
               (racl_addr_hit_write[ 1] & (|(GPIO_PERMIT[ 1] & ~reg_be))) |
               (racl_addr_hit_write[ 2] & (|(GPIO_PERMIT[ 2] & ~reg_be))) |
               (racl_addr_hit_write[ 3] & (|(GPIO_PERMIT[ 3] & ~reg_be))) |
               (racl_addr_hit_write[ 4] & (|(GPIO_PERMIT[ 4] & ~reg_be))) |
               (racl_addr_hit_write[ 5] & (|(GPIO_PERMIT[ 5] & ~reg_be))) |
               (racl_addr_hit_write[ 6] & (|(GPIO_PERMIT[ 6] & ~reg_be))) |
               (racl_addr_hit_write[ 7] & (|(GPIO_PERMIT[ 7] & ~reg_be))) |
               (racl_addr_hit_write[ 8] & (|(GPIO_PERMIT[ 8] & ~reg_be))) |
               (racl_addr_hit_write[ 9] & (|(GPIO_PERMIT[ 9] & ~reg_be))) |
               (racl_addr_hit_write[10] & (|(GPIO_PERMIT[10] & ~reg_be))) |
               (racl_addr_hit_write[11] & (|(GPIO_PERMIT[11] & ~reg_be))) |
               (racl_addr_hit_write[12] & (|(GPIO_PERMIT[12] & ~reg_be))) |
               (racl_addr_hit_write[13] & (|(GPIO_PERMIT[13] & ~reg_be))) |
               (racl_addr_hit_write[14] & (|(GPIO_PERMIT[14] & ~reg_be))) |
               (racl_addr_hit_write[15] & (|(GPIO_PERMIT[15] & ~reg_be))) |
               (racl_addr_hit_write[16] & (|(GPIO_PERMIT[16] & ~reg_be))) |
               (racl_addr_hit_write[17] & (|(GPIO_PERMIT[17] & ~reg_be)))));
  end

  // Generate write-enables
  assign intr_state_we = racl_addr_hit_write[0] & reg_we & !reg_error;

  assign intr_state_wd = reg_wdata[31:0];
  assign intr_enable_we = racl_addr_hit_write[1] & reg_we & !reg_error;

  assign intr_enable_wd = reg_wdata[31:0];
  assign intr_test_we = racl_addr_hit_write[2] & reg_we & !reg_error;

  assign intr_test_wd = reg_wdata[31:0];
  assign alert_test_we = racl_addr_hit_write[3] & reg_we & !reg_error;

  assign alert_test_wd = reg_wdata[0];
  assign direct_out_re = racl_addr_hit_write[5] & reg_re & !reg_error;
  assign direct_out_we = racl_addr_hit_write[5] & reg_we & !reg_error;

  assign direct_out_wd = reg_wdata[31:0];
  assign masked_out_lower_re = racl_addr_hit_write[6] & reg_re & !reg_error;
  assign masked_out_lower_we = racl_addr_hit_write[6] & reg_we & !reg_error;

  assign masked_out_lower_data_wd = reg_wdata[15:0];

  assign masked_out_lower_mask_wd = reg_wdata[31:16];
  assign masked_out_upper_re = racl_addr_hit_write[7] & reg_re & !reg_error;
  assign masked_out_upper_we = racl_addr_hit_write[7] & reg_we & !reg_error;

  assign masked_out_upper_data_wd = reg_wdata[15:0];

  assign masked_out_upper_mask_wd = reg_wdata[31:16];
  assign direct_oe_re = racl_addr_hit_write[8] & reg_re & !reg_error;
  assign direct_oe_we = racl_addr_hit_write[8] & reg_we & !reg_error;

  assign direct_oe_wd = reg_wdata[31:0];
  assign masked_oe_lower_re = racl_addr_hit_write[9] & reg_re & !reg_error;
  assign masked_oe_lower_we = racl_addr_hit_write[9] & reg_we & !reg_error;

  assign masked_oe_lower_data_wd = reg_wdata[15:0];

  assign masked_oe_lower_mask_wd = reg_wdata[31:16];
  assign masked_oe_upper_re = racl_addr_hit_write[10] & reg_re & !reg_error;
  assign masked_oe_upper_we = racl_addr_hit_write[10] & reg_we & !reg_error;

  assign masked_oe_upper_data_wd = reg_wdata[15:0];

  assign masked_oe_upper_mask_wd = reg_wdata[31:16];
  assign intr_ctrl_en_rising_we = racl_addr_hit_write[11] & reg_we & !reg_error;

  assign intr_ctrl_en_rising_wd = reg_wdata[31:0];
  assign intr_ctrl_en_falling_we = racl_addr_hit_write[12] & reg_we & !reg_error;

  assign intr_ctrl_en_falling_wd = reg_wdata[31:0];
  assign intr_ctrl_en_lvlhigh_we = racl_addr_hit_write[13] & reg_we & !reg_error;

  assign intr_ctrl_en_lvlhigh_wd = reg_wdata[31:0];
  assign intr_ctrl_en_lvllow_we = racl_addr_hit_write[14] & reg_we & !reg_error;

  assign intr_ctrl_en_lvllow_wd = reg_wdata[31:0];
  assign ctrl_en_input_filter_we = racl_addr_hit_write[15] & reg_we & !reg_error;

  assign ctrl_en_input_filter_wd = reg_wdata[31:0];

  // Assign write-enables to checker logic vector.
  always_comb begin
    reg_we_check = '0;
    reg_we_check[0] = intr_state_we;
    reg_we_check[1] = intr_enable_we;
    reg_we_check[2] = intr_test_we;
    reg_we_check[3] = alert_test_we;
    reg_we_check[4] = 1'b0;
    reg_we_check[5] = direct_out_we;
    reg_we_check[6] = masked_out_lower_we;
    reg_we_check[7] = masked_out_upper_we;
    reg_we_check[8] = direct_oe_we;
    reg_we_check[9] = masked_oe_lower_we;
    reg_we_check[10] = masked_oe_upper_we;
    reg_we_check[11] = intr_ctrl_en_rising_we;
    reg_we_check[12] = intr_ctrl_en_falling_we;
    reg_we_check[13] = intr_ctrl_en_lvlhigh_we;
    reg_we_check[14] = intr_ctrl_en_lvllow_we;
    reg_we_check[15] = ctrl_en_input_filter_we;
    reg_we_check[16] = 1'b0;
    reg_we_check[17] = 1'b0;
  end

  // Read data return
  always_comb begin
    reg_rdata_next = '0;
    unique case (1'b1)
      racl_addr_hit_read[0]: begin
        reg_rdata_next[31:0] = intr_state_qs;
      end

      racl_addr_hit_read[1]: begin
        reg_rdata_next[31:0] = intr_enable_qs;
      end

      racl_addr_hit_read[2]: begin
        reg_rdata_next[31:0] = '0;
      end

      racl_addr_hit_read[3]: begin
        reg_rdata_next[0] = '0;
      end

      racl_addr_hit_read[4]: begin
        reg_rdata_next[31:0] = data_in_qs;
      end

      racl_addr_hit_read[5]: begin
        reg_rdata_next[31:0] = direct_out_qs;
      end

      racl_addr_hit_read[6]: begin
        reg_rdata_next[15:0] = masked_out_lower_data_qs;
        reg_rdata_next[31:16] = '0;
      end

      racl_addr_hit_read[7]: begin
        reg_rdata_next[15:0] = masked_out_upper_data_qs;
        reg_rdata_next[31:16] = '0;
      end

      racl_addr_hit_read[8]: begin
        reg_rdata_next[31:0] = direct_oe_qs;
      end

      racl_addr_hit_read[9]: begin
        reg_rdata_next[15:0] = masked_oe_lower_data_qs;
        reg_rdata_next[31:16] = masked_oe_lower_mask_qs;
      end

      racl_addr_hit_read[10]: begin
        reg_rdata_next[15:0] = masked_oe_upper_data_qs;
        reg_rdata_next[31:16] = masked_oe_upper_mask_qs;
      end

      racl_addr_hit_read[11]: begin
        reg_rdata_next[31:0] = intr_ctrl_en_rising_qs;
      end

      racl_addr_hit_read[12]: begin
        reg_rdata_next[31:0] = intr_ctrl_en_falling_qs;
      end

      racl_addr_hit_read[13]: begin
        reg_rdata_next[31:0] = intr_ctrl_en_lvlhigh_qs;
      end

      racl_addr_hit_read[14]: begin
        reg_rdata_next[31:0] = intr_ctrl_en_lvllow_qs;
      end

      racl_addr_hit_read[15]: begin
        reg_rdata_next[31:0] = ctrl_en_input_filter_qs;
      end

      racl_addr_hit_read[16]: begin
        reg_rdata_next[0] = hw_straps_data_in_valid_qs;
      end

      racl_addr_hit_read[17]: begin
        reg_rdata_next[31:0] = hw_straps_data_in_qs;
      end

      default: begin
        reg_rdata_next = '1;
      end
    endcase
  end

  // shadow busy
  logic shadow_busy;
  assign shadow_busy = 1'b0;

  // register busy
  assign reg_busy = shadow_busy;

  // Unused signal tieoff

  // wdata / byte enable are not always fully used
  // add a blanket unused statement to handle lint waivers
  logic unused_wdata;
  logic unused_be;
  assign unused_wdata = ^reg_wdata;
  assign unused_be = ^reg_be;
  logic unused_policy_sel;
  assign unused_policy_sel = ^racl_policies_i;

  // Assertions for Register Interface
  `ASSERT_PULSE(wePulse, reg_we, clk_i, !rst_ni)
  `ASSERT_PULSE(rePulse, reg_re, clk_i, !rst_ni)

  `ASSERT(reAfterRv, $rose(reg_re || reg_we) |=> tl_o_pre.d_valid, clk_i, !rst_ni)

  `ASSERT(en2addrHit, (reg_we || reg_re) |-> $onehot0(addr_hit), clk_i, !rst_ni)

  // this is formulated as an assumption such that the FPV testbenches do disprove this
  // property by mistake
  //`ASSUME(reqParity, tl_reg_h2d.a_valid |-> tl_reg_h2d.a_user.chk_en == tlul_pkg::CheckDis)

endmodule
