CAPI=2:
# Copyright lowRISC contributors (OpenTitan project).
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
name: ${instance_vlnv("lowrisc:dv:otp_ctrl_cov")}
description: "OTP_CTRL functional coverage and bind files"

filesets:
  files_rtl:
    depend:
      - ${instance_vlnv("lowrisc:ip:pwrmgr_pkg")}
      - ${instance_vlnv("lowrisc:ip:otp_ctrl_top_specific_pkg")}

  files_dv:
    depend:
      - lowrisc:dv:dv_utils
    files:
      - otp_ctrl_cov_if.sv
      - otp_ctrl_cov_bind.sv
    file_type: systemVerilogSource

targets:
  default:
    filesets:
      - files_rtl
      - files_dv
