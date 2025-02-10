// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Register Top module auto-generated by `reggen`

`include "prim_assert.sv"

module racl_ctrl_reg_top
  # (
    parameter bit          EnableRacl           = 1'b0,
    parameter bit          RaclErrorRsp         = 1'b1
  ) (
  input clk_i,
  input rst_ni,
  input rst_shadowed_ni,
  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o,
  // To HW
  output racl_ctrl_reg_pkg::racl_ctrl_reg2hw_t reg2hw, // Write
  input  racl_ctrl_reg_pkg::racl_ctrl_hw2reg_t hw2reg, // Read

  output logic shadowed_storage_err_o,
  output logic shadowed_update_err_o,

  // RACL interface
  output top_racl_pkg::racl_error_log_t  racl_error_o,

  // Integrity check errors
  output logic intg_err_o
);

  import racl_ctrl_reg_pkg::* ;

  localparam int AW = 5;
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
  logic [4:0] reg_we_check;
  prim_reg_we_check #(
    .OneHotWidth(5)
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
  logic alert_test_we;
  logic alert_test_recov_ctrl_update_err_wd;
  logic alert_test_fatal_fault_wd;
  logic error_log_we;
  logic error_log_valid_qs;
  logic error_log_valid_wd;
  logic error_log_overflow_qs;
  logic error_log_read_access_qs;
  logic [3:0] error_log_role_qs;
  logic [4:0] error_log_ctn_uid_qs;
  logic policy_all_rd_wr_shadowed_re;
  logic policy_all_rd_wr_shadowed_we;
  logic [15:0] policy_all_rd_wr_shadowed_read_perm_qs;
  logic [15:0] policy_all_rd_wr_shadowed_read_perm_wd;
  logic policy_all_rd_wr_shadowed_read_perm_storage_err;
  logic policy_all_rd_wr_shadowed_read_perm_update_err;
  logic [15:0] policy_all_rd_wr_shadowed_write_perm_qs;
  logic [15:0] policy_all_rd_wr_shadowed_write_perm_wd;
  logic policy_all_rd_wr_shadowed_write_perm_storage_err;
  logic policy_all_rd_wr_shadowed_write_perm_update_err;
  logic policy_rot_private_shadowed_re;
  logic policy_rot_private_shadowed_we;
  logic [15:0] policy_rot_private_shadowed_read_perm_qs;
  logic [15:0] policy_rot_private_shadowed_read_perm_wd;
  logic policy_rot_private_shadowed_read_perm_storage_err;
  logic policy_rot_private_shadowed_read_perm_update_err;
  logic [15:0] policy_rot_private_shadowed_write_perm_qs;
  logic [15:0] policy_rot_private_shadowed_write_perm_wd;
  logic policy_rot_private_shadowed_write_perm_storage_err;
  logic policy_rot_private_shadowed_write_perm_update_err;
  logic policy_soc_rot_shadowed_re;
  logic policy_soc_rot_shadowed_we;
  logic [15:0] policy_soc_rot_shadowed_read_perm_qs;
  logic [15:0] policy_soc_rot_shadowed_read_perm_wd;
  logic policy_soc_rot_shadowed_read_perm_storage_err;
  logic policy_soc_rot_shadowed_read_perm_update_err;
  logic [15:0] policy_soc_rot_shadowed_write_perm_qs;
  logic [15:0] policy_soc_rot_shadowed_write_perm_wd;
  logic policy_soc_rot_shadowed_write_perm_storage_err;
  logic policy_soc_rot_shadowed_write_perm_update_err;

  // Register instances
  // R[alert_test]: V(True)
  logic alert_test_qe;
  logic [1:0] alert_test_flds_we;
  assign alert_test_qe = &alert_test_flds_we;
  //   F[recov_ctrl_update_err]: 0:0
  prim_subreg_ext #(
    .DW    (1)
  ) u_alert_test_recov_ctrl_update_err (
    .re     (1'b0),
    .we     (alert_test_we),
    .wd     (alert_test_recov_ctrl_update_err_wd),
    .d      ('0),
    .qre    (),
    .qe     (alert_test_flds_we[0]),
    .q      (reg2hw.alert_test.recov_ctrl_update_err.q),
    .ds     (),
    .qs     ()
  );
  assign reg2hw.alert_test.recov_ctrl_update_err.qe = alert_test_qe;

  //   F[fatal_fault]: 1:1
  prim_subreg_ext #(
    .DW    (1)
  ) u_alert_test_fatal_fault (
    .re     (1'b0),
    .we     (alert_test_we),
    .wd     (alert_test_fatal_fault_wd),
    .d      ('0),
    .qre    (),
    .qe     (alert_test_flds_we[1]),
    .q      (reg2hw.alert_test.fatal_fault.q),
    .ds     (),
    .qs     ()
  );
  assign reg2hw.alert_test.fatal_fault.qe = alert_test_qe;


  // R[error_log]: V(False)
  logic error_log_qe;
  logic [4:0] error_log_flds_we;
  prim_flop #(
    .Width(1),
    .ResetValue(0)
  ) u_error_log0_qe (
    .clk_i(clk_i),
    .rst_ni(rst_ni),
    .d_i(&(error_log_flds_we | 5'h1e)),
    .q_o(error_log_qe)
  );
  //   F[valid]: 0:0
  prim_subreg #(
    .DW      (1),
    .SwAccess(prim_subreg_pkg::SwAccessW1C),
    .RESVAL  (1'h0),
    .Mubi    (1'b0)
  ) u_error_log_valid (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),

    // from register interface
    .we     (error_log_we),
    .wd     (error_log_valid_wd),

    // from internal hardware
    .de     (hw2reg.error_log.valid.de),
    .d      (hw2reg.error_log.valid.d),

    // to internal hardware
    .qe     (error_log_flds_we[0]),
    .q      (reg2hw.error_log.valid.q),
    .ds     (),

    // to register interface (read)
    .qs     (error_log_valid_qs)
  );
  assign reg2hw.error_log.valid.qe = error_log_qe;

  //   F[overflow]: 1:1
  prim_subreg #(
    .DW      (1),
    .SwAccess(prim_subreg_pkg::SwAccessRO),
    .RESVAL  (1'h0),
    .Mubi    (1'b0)
  ) u_error_log_overflow (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),

    // from register interface
    .we     (1'b0),
    .wd     ('0),

    // from internal hardware
    .de     (hw2reg.error_log.overflow.de),
    .d      (hw2reg.error_log.overflow.d),

    // to internal hardware
    .qe     (error_log_flds_we[1]),
    .q      (),
    .ds     (),

    // to register interface (read)
    .qs     (error_log_overflow_qs)
  );

  //   F[read_access]: 2:2
  prim_subreg #(
    .DW      (1),
    .SwAccess(prim_subreg_pkg::SwAccessRO),
    .RESVAL  (1'h0),
    .Mubi    (1'b0)
  ) u_error_log_read_access (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),

    // from register interface
    .we     (1'b0),
    .wd     ('0),

    // from internal hardware
    .de     (hw2reg.error_log.read_access.de),
    .d      (hw2reg.error_log.read_access.d),

    // to internal hardware
    .qe     (error_log_flds_we[2]),
    .q      (),
    .ds     (),

    // to register interface (read)
    .qs     (error_log_read_access_qs)
  );

  //   F[role]: 6:3
  prim_subreg #(
    .DW      (4),
    .SwAccess(prim_subreg_pkg::SwAccessRO),
    .RESVAL  (4'h0),
    .Mubi    (1'b0)
  ) u_error_log_role (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),

    // from register interface
    .we     (1'b0),
    .wd     ('0),

    // from internal hardware
    .de     (hw2reg.error_log.role.de),
    .d      (hw2reg.error_log.role.d),

    // to internal hardware
    .qe     (error_log_flds_we[3]),
    .q      (),
    .ds     (),

    // to register interface (read)
    .qs     (error_log_role_qs)
  );

  //   F[ctn_uid]: 11:7
  prim_subreg #(
    .DW      (5),
    .SwAccess(prim_subreg_pkg::SwAccessRO),
    .RESVAL  (5'h0),
    .Mubi    (1'b0)
  ) u_error_log_ctn_uid (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),

    // from register interface
    .we     (1'b0),
    .wd     ('0),

    // from internal hardware
    .de     (hw2reg.error_log.ctn_uid.de),
    .d      (hw2reg.error_log.ctn_uid.d),

    // to internal hardware
    .qe     (error_log_flds_we[4]),
    .q      (),
    .ds     (),

    // to register interface (read)
    .qs     (error_log_ctn_uid_qs)
  );


  // R[policy_all_rd_wr_shadowed]: V(False)
  //   F[read_perm]: 15:0
  prim_subreg_shadow #(
    .DW      (16),
    .SwAccess(prim_subreg_pkg::SwAccessRW),
    .RESVAL  (16'h7),
    .Mubi    (1'b0)
  ) u_policy_all_rd_wr_shadowed_read_perm (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),
    .rst_shadowed_ni (rst_shadowed_ni),

    // from register interface
    .re     (policy_all_rd_wr_shadowed_re),
    .we     (policy_all_rd_wr_shadowed_we),
    .wd     (policy_all_rd_wr_shadowed_read_perm_wd),

    // from internal hardware
    .de     (1'b0),
    .d      ('0),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.policy_all_rd_wr_shadowed.read_perm.q),
    .ds     (),

    // to register interface (read)
    .qs     (policy_all_rd_wr_shadowed_read_perm_qs),

    // Shadow register phase. Relevant for hwext only.
    .phase  (),

    // Shadow register error conditions
    .err_update  (policy_all_rd_wr_shadowed_read_perm_update_err),
    .err_storage (policy_all_rd_wr_shadowed_read_perm_storage_err)
  );

  //   F[write_perm]: 31:16
  prim_subreg_shadow #(
    .DW      (16),
    .SwAccess(prim_subreg_pkg::SwAccessRW),
    .RESVAL  (16'h7),
    .Mubi    (1'b0)
  ) u_policy_all_rd_wr_shadowed_write_perm (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),
    .rst_shadowed_ni (rst_shadowed_ni),

    // from register interface
    .re     (policy_all_rd_wr_shadowed_re),
    .we     (policy_all_rd_wr_shadowed_we),
    .wd     (policy_all_rd_wr_shadowed_write_perm_wd),

    // from internal hardware
    .de     (1'b0),
    .d      ('0),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.policy_all_rd_wr_shadowed.write_perm.q),
    .ds     (),

    // to register interface (read)
    .qs     (policy_all_rd_wr_shadowed_write_perm_qs),

    // Shadow register phase. Relevant for hwext only.
    .phase  (),

    // Shadow register error conditions
    .err_update  (policy_all_rd_wr_shadowed_write_perm_update_err),
    .err_storage (policy_all_rd_wr_shadowed_write_perm_storage_err)
  );


  // R[policy_rot_private_shadowed]: V(False)
  //   F[read_perm]: 15:0
  prim_subreg_shadow #(
    .DW      (16),
    .SwAccess(prim_subreg_pkg::SwAccessRW),
    .RESVAL  (16'h1),
    .Mubi    (1'b0)
  ) u_policy_rot_private_shadowed_read_perm (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),
    .rst_shadowed_ni (rst_shadowed_ni),

    // from register interface
    .re     (policy_rot_private_shadowed_re),
    .we     (policy_rot_private_shadowed_we),
    .wd     (policy_rot_private_shadowed_read_perm_wd),

    // from internal hardware
    .de     (1'b0),
    .d      ('0),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.policy_rot_private_shadowed.read_perm.q),
    .ds     (),

    // to register interface (read)
    .qs     (policy_rot_private_shadowed_read_perm_qs),

    // Shadow register phase. Relevant for hwext only.
    .phase  (),

    // Shadow register error conditions
    .err_update  (policy_rot_private_shadowed_read_perm_update_err),
    .err_storage (policy_rot_private_shadowed_read_perm_storage_err)
  );

  //   F[write_perm]: 31:16
  prim_subreg_shadow #(
    .DW      (16),
    .SwAccess(prim_subreg_pkg::SwAccessRW),
    .RESVAL  (16'h1),
    .Mubi    (1'b0)
  ) u_policy_rot_private_shadowed_write_perm (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),
    .rst_shadowed_ni (rst_shadowed_ni),

    // from register interface
    .re     (policy_rot_private_shadowed_re),
    .we     (policy_rot_private_shadowed_we),
    .wd     (policy_rot_private_shadowed_write_perm_wd),

    // from internal hardware
    .de     (1'b0),
    .d      ('0),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.policy_rot_private_shadowed.write_perm.q),
    .ds     (),

    // to register interface (read)
    .qs     (policy_rot_private_shadowed_write_perm_qs),

    // Shadow register phase. Relevant for hwext only.
    .phase  (),

    // Shadow register error conditions
    .err_update  (policy_rot_private_shadowed_write_perm_update_err),
    .err_storage (policy_rot_private_shadowed_write_perm_storage_err)
  );


  // R[policy_soc_rot_shadowed]: V(False)
  //   F[read_perm]: 15:0
  prim_subreg_shadow #(
    .DW      (16),
    .SwAccess(prim_subreg_pkg::SwAccessRW),
    .RESVAL  (16'h5),
    .Mubi    (1'b0)
  ) u_policy_soc_rot_shadowed_read_perm (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),
    .rst_shadowed_ni (rst_shadowed_ni),

    // from register interface
    .re     (policy_soc_rot_shadowed_re),
    .we     (policy_soc_rot_shadowed_we),
    .wd     (policy_soc_rot_shadowed_read_perm_wd),

    // from internal hardware
    .de     (1'b0),
    .d      ('0),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.policy_soc_rot_shadowed.read_perm.q),
    .ds     (),

    // to register interface (read)
    .qs     (policy_soc_rot_shadowed_read_perm_qs),

    // Shadow register phase. Relevant for hwext only.
    .phase  (),

    // Shadow register error conditions
    .err_update  (policy_soc_rot_shadowed_read_perm_update_err),
    .err_storage (policy_soc_rot_shadowed_read_perm_storage_err)
  );

  //   F[write_perm]: 31:16
  prim_subreg_shadow #(
    .DW      (16),
    .SwAccess(prim_subreg_pkg::SwAccessRW),
    .RESVAL  (16'h5),
    .Mubi    (1'b0)
  ) u_policy_soc_rot_shadowed_write_perm (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),
    .rst_shadowed_ni (rst_shadowed_ni),

    // from register interface
    .re     (policy_soc_rot_shadowed_re),
    .we     (policy_soc_rot_shadowed_we),
    .wd     (policy_soc_rot_shadowed_write_perm_wd),

    // from internal hardware
    .de     (1'b0),
    .d      ('0),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.policy_soc_rot_shadowed.write_perm.q),
    .ds     (),

    // to register interface (read)
    .qs     (policy_soc_rot_shadowed_write_perm_qs),

    // Shadow register phase. Relevant for hwext only.
    .phase  (),

    // Shadow register error conditions
    .err_update  (policy_soc_rot_shadowed_write_perm_update_err),
    .err_storage (policy_soc_rot_shadowed_write_perm_storage_err)
  );



  logic [4:0] addr_hit;
  top_racl_pkg::racl_role_vec_t racl_role_vec;
  top_racl_pkg::racl_role_t racl_role;

  logic [4:0] racl_addr_hit_read;
  logic [4:0] racl_addr_hit_write;

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
    // For the static RACL assignment for racl_ctrl only one role (ROT_PRIVATE) is used,
    // leaving others unread. Intentionally read them to avoid linting errors.
    logic unused_role_vec;
    assign unused_role_vec = ^racl_role_vec;
  end else begin : gen_no_racl_role_logic
    assign racl_role     = '0;
    assign racl_role_vec = '0;
  end

  always_comb begin
    addr_hit = '0;
    racl_addr_hit_read  = '0;
    racl_addr_hit_write = '0;
    addr_hit[0] = (reg_addr == RACL_CTRL_ALERT_TEST_OFFSET);
    addr_hit[1] = (reg_addr == RACL_CTRL_ERROR_LOG_OFFSET);
    addr_hit[2] = (reg_addr == RACL_CTRL_POLICY_ALL_RD_WR_SHADOWED_OFFSET);
    addr_hit[3] = (reg_addr == RACL_CTRL_POLICY_ROT_PRIVATE_SHADOWED_OFFSET);
    addr_hit[4] = (reg_addr == RACL_CTRL_POLICY_SOC_ROT_SHADOWED_OFFSET);

    if (EnableRacl) begin : gen_racl_hit
      for (int unsigned slice_idx = 0; slice_idx < 5; slice_idx++) begin
        // Static RACL protection with ROT_PRIVATE policy
        racl_addr_hit_read[slice_idx] =
          addr_hit[slice_idx] & (|(top_racl_pkg::RACL_POLICY_ROT_PRIVATE_RD & racl_role_vec));
        racl_addr_hit_write[slice_idx] =
          addr_hit[slice_idx] & (|(top_racl_pkg::RACL_POLICY_ROT_PRIVATE_WR & racl_role_vec));
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
              ((racl_addr_hit_write[0] & (|(RACL_CTRL_PERMIT[0] & ~reg_be))) |
               (racl_addr_hit_write[1] & (|(RACL_CTRL_PERMIT[1] & ~reg_be))) |
               (racl_addr_hit_write[2] & (|(RACL_CTRL_PERMIT[2] & ~reg_be))) |
               (racl_addr_hit_write[3] & (|(RACL_CTRL_PERMIT[3] & ~reg_be))) |
               (racl_addr_hit_write[4] & (|(RACL_CTRL_PERMIT[4] & ~reg_be)))));
  end

  // Generate write-enables
  assign alert_test_we = racl_addr_hit_write[0] & reg_we & !reg_error;

  assign alert_test_recov_ctrl_update_err_wd = reg_wdata[0];

  assign alert_test_fatal_fault_wd = reg_wdata[1];
  assign error_log_we = racl_addr_hit_write[1] & reg_we & !reg_error;

  assign error_log_valid_wd = reg_wdata[0];
  assign policy_all_rd_wr_shadowed_re = racl_addr_hit_write[2] & reg_re & !reg_error;
  assign policy_all_rd_wr_shadowed_we = racl_addr_hit_write[2] & reg_we & !reg_error;

  assign policy_all_rd_wr_shadowed_read_perm_wd = reg_wdata[15:0];

  assign policy_all_rd_wr_shadowed_write_perm_wd = reg_wdata[31:16];
  assign policy_rot_private_shadowed_re = racl_addr_hit_write[3] & reg_re & !reg_error;
  assign policy_rot_private_shadowed_we = racl_addr_hit_write[3] & reg_we & !reg_error;

  assign policy_rot_private_shadowed_read_perm_wd = reg_wdata[15:0];

  assign policy_rot_private_shadowed_write_perm_wd = reg_wdata[31:16];
  assign policy_soc_rot_shadowed_re = racl_addr_hit_write[4] & reg_re & !reg_error;
  assign policy_soc_rot_shadowed_we = racl_addr_hit_write[4] & reg_we & !reg_error;

  assign policy_soc_rot_shadowed_read_perm_wd = reg_wdata[15:0];

  assign policy_soc_rot_shadowed_write_perm_wd = reg_wdata[31:16];

  // Assign write-enables to checker logic vector.
  always_comb begin
    reg_we_check = '0;
    reg_we_check[0] = alert_test_we;
    reg_we_check[1] = error_log_we;
    reg_we_check[2] = policy_all_rd_wr_shadowed_we;
    reg_we_check[3] = policy_rot_private_shadowed_we;
    reg_we_check[4] = policy_soc_rot_shadowed_we;
  end

  // Read data return
  always_comb begin
    reg_rdata_next = '0;
    unique case (1'b1)
      racl_addr_hit_read[0]: begin
        reg_rdata_next[0] = '0;
        reg_rdata_next[1] = '0;
      end

      racl_addr_hit_read[1]: begin
        reg_rdata_next[0] = error_log_valid_qs;
        reg_rdata_next[1] = error_log_overflow_qs;
        reg_rdata_next[2] = error_log_read_access_qs;
        reg_rdata_next[6:3] = error_log_role_qs;
        reg_rdata_next[11:7] = error_log_ctn_uid_qs;
      end

      racl_addr_hit_read[2]: begin
        reg_rdata_next[15:0] = policy_all_rd_wr_shadowed_read_perm_qs;
        reg_rdata_next[31:16] = policy_all_rd_wr_shadowed_write_perm_qs;
      end

      racl_addr_hit_read[3]: begin
        reg_rdata_next[15:0] = policy_rot_private_shadowed_read_perm_qs;
        reg_rdata_next[31:16] = policy_rot_private_shadowed_write_perm_qs;
      end

      racl_addr_hit_read[4]: begin
        reg_rdata_next[15:0] = policy_soc_rot_shadowed_read_perm_qs;
        reg_rdata_next[31:16] = policy_soc_rot_shadowed_write_perm_qs;
      end

      default: begin
        reg_rdata_next = '1;
      end
    endcase
  end

  // shadow busy
  logic shadow_busy;
  logic rst_done;
  logic shadow_rst_done;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rst_done <= '0;
    end else begin
      rst_done <= 1'b1;
    end
  end

  always_ff @(posedge clk_i or negedge rst_shadowed_ni) begin
    if (!rst_shadowed_ni) begin
      shadow_rst_done <= '0;
    end else begin
      shadow_rst_done <= 1'b1;
    end
  end

  // both shadow and normal resets have been released
  assign shadow_busy = ~(rst_done & shadow_rst_done);

  // Collect up storage and update errors
  assign shadowed_storage_err_o = |{
    policy_all_rd_wr_shadowed_read_perm_storage_err,
    policy_all_rd_wr_shadowed_write_perm_storage_err,
    policy_rot_private_shadowed_read_perm_storage_err,
    policy_rot_private_shadowed_write_perm_storage_err,
    policy_soc_rot_shadowed_read_perm_storage_err,
    policy_soc_rot_shadowed_write_perm_storage_err
  };
  assign shadowed_update_err_o = |{
    policy_all_rd_wr_shadowed_read_perm_update_err,
    policy_all_rd_wr_shadowed_write_perm_update_err,
    policy_rot_private_shadowed_read_perm_update_err,
    policy_rot_private_shadowed_write_perm_update_err,
    policy_soc_rot_shadowed_read_perm_update_err,
    policy_soc_rot_shadowed_write_perm_update_err
  };

  // register busy
  assign reg_busy = shadow_busy;

  // Unused signal tieoff

  // wdata / byte enable are not always fully used
  // add a blanket unused statement to handle lint waivers
  logic unused_wdata;
  logic unused_be;
  assign unused_wdata = ^reg_wdata;
  assign unused_be = ^reg_be;

  // Assertions for Register Interface
  `ASSERT_PULSE(wePulse, reg_we, clk_i, !rst_ni)
  `ASSERT_PULSE(rePulse, reg_re, clk_i, !rst_ni)

  `ASSERT(reAfterRv, $rose(reg_re || reg_we) |=> tl_o_pre.d_valid, clk_i, !rst_ni)

  `ASSERT(en2addrHit, (reg_we || reg_re) |-> $onehot0(addr_hit), clk_i, !rst_ni)

  // this is formulated as an assumption such that the FPV testbenches do disprove this
  // property by mistake
  //`ASSUME(reqParity, tl_reg_h2d.a_valid |-> tl_reg_h2d.a_user.chk_en == tlul_pkg::CheckDis)

endmodule
