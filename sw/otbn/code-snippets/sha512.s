/* Copyright lowRISC contributors. */
/* Licensed under the Apache License, Version 2.0, see LICENSE for details. */
/* SPDX-License-Identifier: Apache-2.0 */


.text
.globl hash_sha_512

/*
 * Memory map:
 *
 * H: DMEM[0..7]
 * L: DMEM[8]
 * M: DMEM[9..12]
 * K: DMEM[13..22]
 * W: DMEM[23..42]
 *
 * Registers:
 *
 * W[16..23]: H[0..7]
 * W[24..31]: Temp H registers (so that it is less sucky to program)
 */

sha512_expand:
  /* pointers to temp registers */
  li    x8,  8
  li    x9,  12  /* blinding_val_1 */
  li    x10, 13  /* blinding_val_2 */
  li    x11, 14  /* lo */
  li    x12, 15  /* hi */
  /* w16 = s0 */
  /* w17 = s1 */
  /* w18 = t0 */
  /* w19 = t1 */
  /* w20 = t2 */
  /* w21 = t3 */

  /* load pointer to message */
  lw    x16, 4(x0)

  /* load pointer to W */
  lw    x17, 8(x0)

  /* load message to WDR[8..11] and store in W[0..3] */
  loopi  4, 2
    bn.lid    x8, 0(x16++)
    bn.sid    x8, 0(x17++)
    addi      x8, x8, 1

  /* calculate W[16..80] */
  loopi 16, 17
    /* Operation on W[t-15] */
    bn.rshi w14, w8,  w12 >> 128
    bn.rshi w15, w13, w8  >> 64
    bn.rshi w16, w15, w14 >> 1
    bn.rshi w17, w15, w14 >> 8
    bn.xor  w16, w16, w17
    bn.rshi w17, w31, w14 >> 7
    bn.xor  w16, w16, w17

    /* temp =  W[t-15] + W[t-16] */
    bn.add  w18, w8,  w16 >> 192

    /* temp += W[t-7] */
    bn.add  w18, w18, w10 >> 64

    /* Operation on W[t-2] */
    bn.rshi w14, w11, w12 >> 192
    bn.rshi w15, w13, w11 >> 128
    bn.rshi w16, w15, w14 >> 19
    bn.rshi w17, w15, w14 >> 61
    bn.xor  w16, w16, w17
    bn.rshi w17, w31, w14 >> 6
    bn.xor  w16, w16, w17
    bn.add  w18, w18, w16 >> 192

sha512_compress_block:

  /* prepare all-zero reg */
  bn.xor    w31, w31, w31

  jal       x1, expand_msg

  /* number of registers used to store the hash state */
  ldi       x30, 8

  /* load pointer to hash state H[0] */
  lw        x16, 0(x0)

  /* load H state into wide registers */
  li   x8, 0
  loop x30, 2
    bn.lid  x8, 0(x16++)
    addi    x8, x8, 1
