// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

//==================================================
// This file contains the Excluded objects
// Generated By User: gdessouky
// Format Version: 2
// Date: Sat Jul  6 04:05:21 2024
// ExclMode: default
//==================================================
CHECKSUM: "1683432060 1171249183"
INSTANCE: tb.dut.u_prim_sha2_512.gen_multimode_logic.u_prim_sha2_multimode.u_pad
Fsm st_q "1171249183"
ANNOTATION: "[INVALID] Intend to remove transition"
Transition StLenHi->StFifoReceive "4->1"
Fsm st_q "1171249183"
ANNOTATION: "[INVALID] Intend to remove transition"
Transition StPad80->StFifoReceive "2->1"
Fsm st_q "1171249183"
ANNOTATION: "[INVALID] Intend to remove transition"
Transition StPad00->StFifoReceive "3->1"
Fsm st_q "1171249183"
ANNOTATION: "[INVALID] Intend to remove transition"
Transition StLenLo->StFifoReceive "5->1"
CHECKSUM: "1785966602"
INSTANCE: tb.dut.u_tlul_adapter.u_rspfifo
ANNOTATION: "[INVALID] Disable this assertion as the FIFO is WO"
Assert DataKnown_A "assertion"
CHECKSUM: "1785966602"
INSTANCE: tb.dut.u_tlul_adapter.u_sramreqfifo
ANNOTATION: "[INVALID] Disable this assertion as the FIFO is WO"
Assert DataKnown_A "assertion"
CHECKSUM: "3919502532"
INSTANCE: tb.dut.u_tlul_adapter
ANNOTATION: "[INVALID] Disable this assertion as the FIFO is WO"
Assert rvalidHighReqFifoEmpty "assertion"
ANNOTATION: "[INVALID] Disable this assertion as the FIFO is WO"
Assert rvalidHighWhenRspFifoFull "assertion"
