// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
${gencmd}
<%
irq_peripheral_names = sorted({p.name for p in helper.irq_peripherals})

# For some rv_timer DIFs, tha hart argument follows the instance handle.
def args(p):
    extra_arg = ", kHart" if p.name == "rv_timer" else ""
    return f"&{p.inst_name}{extra_arg}"
%>\

#include "sw/device/lib/base/mmio.h"
#include "sw/device/lib/base/freestanding/limits.h"
% for n in sorted(irq_peripheral_names + ["rv_plic"]):
#include "sw/device/lib/dif/dif_${n}.h"
% endfor
#include "sw/device/lib/handler.h"
#include "sw/device/lib/irq.h"
#include "sw/device/lib/runtime/log.h"
#include "sw/device/lib/testing/check.h"
#include "sw/device/lib/testing/rv_plic_testutils.h"
#include "sw/device/lib/testing/test_framework/test_main.h"
#include "sw/device/lib/testing/test_framework/test_status.h"
#include "hw/top_earlgrey/sw/autogen/top_earlgrey.h"

% for p in helper.irq_peripherals:
static dif_${p.name}_t ${p.inst_name};
% endfor
static dif_rv_plic_t plic;
static const top_earlgrey_plic_target_t kHart = kTopEarlgreyPlicTargetIbex0;

/**
 * Flag indicating which peripheral is under test.
 *
 * Declared volatile because it is referenced in the main program flow as well
 * as the ISR.
 */
static volatile top_earlgrey_plic_peripheral_t peripheral_expected;

/**
 * Flags indicating the IRQ expected to have triggered and serviced within the
 * peripheral.
 *
 * Declared volatile because it is referenced in the main program flow as well
 * as the ISR.
 */
% for n in irq_peripheral_names:
static volatile dif_${n}_irq_t ${n}_irq_expected;
static volatile dif_${n}_irq_t ${n}_irq_serviced;
% endfor

/**
 * Provides external IRQ handling for this test.
 *
 * This function overrides the default external IRQ handler in
 * `sw/device/lib/handler.h`.
 *
 * For each IRQ, it performs the following:
 * 1. Claims the IRQ fired (finds PLIC IRQ index).
 * 2. Checks that the index belongs to the expected peripheral.
 * 3. Checks that the correct and the only IRQ from the expected peripheral
 *    triggered.
 * 4. Clears the IRQ at the peripheral.
 * 5. Completes the IRQ service at PLIC.
 */
void handler_irq_external(void) {
  dif_rv_plic_irq_id_t plic_irq_id;
  CHECK_DIF_OK(dif_rv_plic_irq_claim(&plic, kHart, &plic_irq_id));

  top_earlgrey_plic_peripheral_t peripheral = (top_earlgrey_plic_peripheral_t)
      top_earlgrey_plic_interrupt_for_peripheral[plic_irq_id];
  CHECK(peripheral == peripheral_expected,
        "Interrupt from incorrect peripheral: exp = %d, obs = %d",
        peripheral_expected, peripheral);

  switch (peripheral) {
    % for p in helper.irq_peripherals:
    case ${p.plic_name}: {
      dif_${p.name}_irq_t irq = (dif_${p.name}_irq_t)(
          plic_irq_id - (dif_rv_plic_irq_id_t)${p.plic_start_irq});
      CHECK(irq == ${p.name}_irq_expected,
            "Incorrect ${p.inst_name} IRQ triggered: exp = %d, obs = %d",
            ${p.name}_irq_expected, irq);
      ${p.name}_irq_serviced = irq;

      dif_${p.name}_irq_state_snapshot_t snapshot;
      CHECK_DIF_OK(dif_${p.name}_irq_get_state(${args(p)}, &snapshot));
      CHECK(snapshot == (dif_${p.name}_irq_state_snapshot_t)(1 << irq),
            "Only ${p.inst_name} IRQ %d expected to fire. Actual interrupt "
            "status = %x",
            irq, snapshot);

      CHECK_DIF_OK(dif_${p.name}_irq_acknowledge(&${p.inst_name}, irq));
      break;
    }

    % endfor

    default:
      LOG_FATAL("ISR is not implemented!");
      test_status_set(kTestStatusFailed);
  }

  // Complete the IRQ at PLIC.
  CHECK_DIF_OK(dif_rv_plic_irq_complete(&plic, kHart, plic_irq_id));
}

/**
 * Initializes the handles to all peripherals.
 */
static void peripherals_init(void) {
  mmio_region_t base_addr;

  % for p in helper.irq_peripherals:
  base_addr = mmio_region_from_addr(${p.base_addr_name});
  CHECK_DIF_OK(dif_${p.name}_init(base_addr, &${p.inst_name}));

  % endfor

  base_addr = mmio_region_from_addr(TOP_EARLGREY_RV_PLIC_BASE_ADDR);
  CHECK_DIF_OK(dif_rv_plic_init(base_addr, &plic));
}

/**
 * Clears pending IRQs in all peripherals.
 */
static void peripheral_irqs_clear(void) {
  % for p in helper.irq_peripherals:
  CHECK_DIF_OK(dif_${p.name}_irq_acknowledge_all(${args(p)}));
  % endfor
}

/**
 * Enables all IRQs in all peripherals.
 */
static void peripheral_irqs_enable(void) {
  % for n in irq_peripheral_names:
  <%
    if n == "aon_timer": continue
  %>\
  dif_${n}_irq_state_snapshot_t ${n}_irq_snapshot =
      (dif_${n}_irq_state_snapshot_t)UINT_MAX;
  % endfor

  % for p in helper.irq_peripherals:
  <%
    if p.name == "aon_timer": continue
  %>\
  CHECK_DIF_OK(dif_${p.name}_irq_restore_all(${args(p)}, &${p.name}_irq_snapshot));
  % endfor
}

/**
 * Triggers all IRQs in all peripherals one by one.
 *
 * Walks through all instances of all peripherals and triggers an interrupt one
 * by one, by forcing with the `intr_test` CSR. On trigger, the CPU instantly
 * jumps into the ISR. The main flow of execution thus proceeds to check that
 * the correct IRQ was serviced immediately. The ISR, in turn checks if the
 * expected IRQ from the expected peripheral triggered.
 */
static void peripheral_irqs_trigger(void) {
  % for p in helper.irq_peripherals:
  peripheral_expected = ${p.plic_name};
  for (dif_${p.name}_irq_t irq = ${p.start_irq}; irq <= ${p.end_irq}; ++irq) {
    ${p.name}_irq_expected = irq;
    LOG_INFO("Triggering ${p.inst_name} IRQ %d.", irq);
    CHECK_DIF_OK(dif_${p.name}_irq_force(&${p.inst_name}, irq));

    CHECK(${p.name}_irq_serviced == irq,
          "Incorrect ${p.inst_name} IRQ serviced: exp = %d, obs = %d", irq,
          ${p.name}_irq_serviced);
    LOG_INFO("IRQ %d from ${p.inst_name} is serviced.", irq);
  }

  % endfor
}

const test_config_t kTestConfig;

bool test_main(void) {
  irq_global_ctrl(true);
  irq_external_ctrl(true);
  peripherals_init();
  rv_plic_testutils_irq_range_enable(&plic, kHart,
      kTopEarlgreyPlicIrqIdNone + 1, kTopEarlgreyPlicIrqIdLast);
  peripheral_irqs_clear();
  peripheral_irqs_enable();
  peripheral_irqs_trigger();
  return true;
}
