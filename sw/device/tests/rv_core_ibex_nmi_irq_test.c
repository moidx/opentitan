// Copyright lowRISC contributors (OpenTitan project).
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <assert.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>

#include "sw/device/lib/base/mmio.h"
#include "sw/device/lib/dif/dif_aes.h"
#include "sw/device/lib/dif/dif_alert_handler.h"
#include "sw/device/lib/dif/dif_aon_timer.h"
#include "sw/device/lib/dif/dif_pwrmgr.h"
#include "sw/device/lib/dif/dif_rv_core_ibex.h"
#include "sw/device/lib/dif/dif_rv_timer.h"
#include "sw/device/lib/runtime/ibex.h"
#include "sw/device/lib/runtime/irq.h"
#include "sw/device/lib/runtime/log.h"
#include "sw/device/lib/testing/alert_handler_testutils.h"
#include "sw/device/lib/testing/aon_timer_testutils.h"
#include "sw/device/lib/testing/pwrmgr_testutils.h"
#include "sw/device/lib/testing/test_framework/check.h"
#include "sw/device/lib/testing/test_framework/ottf_isrs.h"
#include "sw/device/lib/testing/test_framework/ottf_main.h"

#include "hw/top_earlgrey/sw/autogen/top_earlgrey.h"
#include "sw/device/lib/testing/autogen/isr_testutils.h"

OTTF_DEFINE_TEST_CONFIG();

typedef void (*isr_handler)(void);

typedef struct counters {
  uint32_t wdog_barked;
  uint32_t alert_raised;
  uint64_t elapsed_ticks;
} counters_t;

static volatile counters_t counters = {0};

static volatile isr_handler expected_isr_handler;
static volatile dif_rv_core_ibex_nmi_state_t nmi_state;
static volatile dif_alert_handler_class_state_t alert_state;
static volatile bool nmi_fired = false;
static volatile bool ext_irq_fired = false;
static volatile bool irq_is_pending = false;

static dif_aes_t aes;
static dif_pwrmgr_t pwrmgr;
static dif_rv_core_ibex_t rv_core_ibex;
static dif_aon_timer_t aon_timer;
static dif_alert_handler_t alert_handler;
static dif_rv_timer_t timer;

/**
 * Program the alert handler to escalate on alerts upto phase 2 (i.e. reset)
 * but at the phase 1 (i.e. wipe secrets) the NMI interrupt handler should clear
 * the escalation.
 */
enum {
  kEscalationPhase0Micros = 100 * 1000,  // 100 ms
  kEscalationPhase2Micros = 100,         // 100 us
  kIrqDeadlineMicros = 100000,           // 10 us
  kWdogBarkMicros = 50,                  // 500 us
  kTick1us = 1 * 1000 * 1000,            // 1MHz - 1us.
  kHart = kTopEarlgreyPlicTargetIbex0,
};

/**
 * Defines how many times the two NMI tests are repeated back to back.
 */
const int kNumTestIterations = 3;

static status_t set_tick(uint32_t tick_hz) {
  dif_rv_timer_tick_params_t tick_params;
  TRY(dif_rv_timer_approximate_tick_params(kClockFreqPeripheralHz, tick_hz,
                                           &tick_params));
  TRY(dif_rv_timer_set_tick_params(&timer, kHart, tick_params));
  return OK_STATUS();
}

static void alert_handler_config(void) {
  uint32_t cycles[3] = {0};
  CHECK_STATUS_OK(alert_handler_testutils_get_cycles_from_us(
      kEscalationPhase0Micros, &cycles[0]));
  CHECK_STATUS_OK(alert_handler_testutils_get_cycles_from_us(
      kEscalationPhase2Micros, &cycles[1]));
  CHECK_STATUS_OK(alert_handler_testutils_get_cycles_from_us(kIrqDeadlineMicros,
                                                             &cycles[2]));
  dif_alert_handler_escalation_phase_t esc_phases[] = {
      {.phase = kDifAlertHandlerClassStatePhase0,
       .signal = 0,
       .duration_cycles = cycles[0]},
      {.phase = kDifAlertHandlerClassStatePhase1,
       .signal = 3,
       .duration_cycles = cycles[1]}};
  dif_alert_handler_class_config_t class_config[] = {{
      .auto_lock_accumulation_counter = kDifToggleDisabled,
      .accumulator_threshold = 0,
      .irq_deadline_cycles = cycles[2],
      .escalation_phases = esc_phases,
      .escalation_phases_len = ARRAYSIZE(esc_phases),
      .crashdump_escalation_phase = kDifAlertHandlerClassStatePhase2,
  }};

  dif_alert_handler_alert_t alerts[] = {
      kTopEarlgreyAlertIdAesRecovCtrlUpdateErr};
  dif_alert_handler_class_t alert_classes[] = {kDifAlertHandlerClassA};
  dif_alert_handler_class_t classes[] = {kDifAlertHandlerClassA};
  dif_alert_handler_config_t config = {
      .alerts = alerts,
      .alert_classes = alert_classes,
      .alerts_len = ARRAYSIZE(alerts),
      .classes = classes,
      .class_configs = class_config,
      .classes_len = ARRAYSIZE(class_config),
      .ping_timeout = 0x100,
  };

  LOG_INFO("configure alerts");
  CHECK_STATUS_OK(
      alert_handler_testutils_configure_all(&alert_handler, config,
                                            /*lock=*/kDifToggleEnabled));

  CHECK_STATUS_OK(set_tick(kTick1us));
  counters.elapsed_ticks = 0;
  CHECK_DIF_OK(
      dif_rv_timer_counter_write(&timer, kHart, counters.elapsed_ticks));
  CHECK_DIF_OK(dif_rv_timer_counter_read(&timer, kHart,
                                         (uint64_t *)&counters.elapsed_ticks));
  CHECK(counters.elapsed_ticks == 0, "Failed to write the counter");

  CHECK_DIF_OK(
      dif_rv_timer_counter_set_enabled(&timer, kHart, kDifToggleEnabled));
  CHECK_DIF_OK(dif_alert_handler_configure_ping_timer(
      &alert_handler, config.ping_timeout, kDifToggleEnabled,
      kDifToggleDisabled));
}

/**
 * Handle the wdog bark NMI IRQ.
 */
static void wdog_handler(void) {
  bool is_pending;
  // The watchdog bark external interrupt is also connected to the NMI input
  // of rv_core_ibex. We therefore expect the interrupt to be pending on the
  // peripheral side (the check is done later in the test function).
  CHECK_DIF_OK(dif_aon_timer_irq_is_pending(
      &aon_timer, kDifAonTimerIrqWdogTimerBark, &is_pending));
  if (is_pending) {
    irq_is_pending = is_pending;
    // Stop the watchdog.
    CHECK_DIF_OK(dif_aon_timer_watchdog_stop(&aon_timer));
    // In order to handle the NMI we need to acknowledge the interrupt status
    // bit it at the peripheral side. Note that this test does not use the
    // PLIC, so there is nothing to acknowledge on the PLIC side. We are hence
    // not using the isr_testutils_aon_timer_isr function here.
    CHECK_DIF_OK(dif_aon_timer_irq_acknowledge(&aon_timer,
                                               kDifAonTimerIrqWdogTimerBark));
  }
}

/**
 * Handle the alert handler NMI IRQ.
 */
static void alert_handler_handler(void) {
  bool can_clear;
  dif_alert_handler_class_state_t state = {0};

  CHECK_DIF_OK(dif_aes_alert_clear(&aes, kDifAesAlertRecovCtrlUpdateErr));

  // In this particular case the alert handler is configured to send out an NMI
  // when class A enters escalation phase 0. We therefore get the alert handler
  // state and check later in the test function whether class A has indeed
  // escalated.
  CHECK_DIF_OK(dif_alert_handler_get_class_state(
      &alert_handler, kDifAlertHandlerClassA, &state));
  if (state) {
    alert_state = state;
    LOG_INFO("class[A]=0x%x", alert_state);
  }

  for (dif_alert_handler_local_alert_t i = 0; i < 7; i++) {
    bool is_cause = false;
    CHECK_DIF_OK(
        dif_alert_handler_local_alert_is_cause(&alert_handler, i, &is_cause));
    if (is_cause) {
      CHECK_DIF_OK(
          dif_alert_handler_local_alert_acknowledge(&alert_handler, i));
      LOG_INFO("local_alert[%d] is_cause: %d", i, is_cause);
    }
  }

  for (size_t i = 0; i < kTopEarlgreyAlertIdLast; ++i) {
    dif_alert_handler_alert_t alert = i;
    bool is_cause = false;
    CHECK_DIF_OK(
        dif_alert_handler_alert_is_cause(&alert_handler, alert, &is_cause));
    if (is_cause) {
      CHECK_DIF_OK(dif_alert_handler_alert_acknowledge(&alert_handler, alert));
      LOG_INFO("alert_id: %d, is_cause: %d", alert, is_cause);
    }
  }

  // We expect the class to be clearable since it has not been locked.
  CHECK_DIF_OK(dif_alert_handler_escalation_can_clear(
      &alert_handler, kDifAlertHandlerClassA, &can_clear));
  CHECK(can_clear, "Alert handler is locked.");

  // Clear the class in order to stop the alert handler from sending
  // an NMI to the processor.
  CHECK_DIF_OK(dif_alert_handler_escalation_clear(&alert_handler,
                                                  kDifAlertHandlerClassA));
}

/**
 * Execute the wdog bark NMI interrupt test.
 */
static void wdog_nmi_test(void) {
  // Setup to handle the incoming wdog NMI IRQ.
  expected_isr_handler = wdog_handler;
  nmi_fired = false;
  nmi_state = (dif_rv_core_ibex_nmi_state_t){0};

  // Setup the wdog bark interrupt.
  uint32_t count_cycles = 0;
  CHECK_STATUS_OK(aon_timer_testutils_get_aon_cycles_from_us(kWdogBarkMicros,
                                                             &count_cycles));
  CHECK_STATUS_OK(
      aon_timer_testutils_watchdog_config(&aon_timer,
                                          /*bark_cycles=*/count_cycles,
                                          /*bite_cycles=*/UINT32_MAX,
                                          /*pause_in_sleep=*/false));
  LOG_INFO("Wait for aon_timer NMI");
  IBEX_SPIN_FOR(nmi_fired, kWdogBarkMicros * 20000);

  LOG_INFO("wdog_nmi_test: counter.wdog_barked: %d", counters.wdog_barked);
  LOG_INFO("wdog_nmi_test: counter.alert_raised: %d", counters.alert_raised);

  // We expect the watchdog bark interrupt to be pending on the peripheral side.
  CHECK(irq_is_pending, "Expected watchdog bark interrupt to be pending");
  irq_is_pending = false;

  // Check that the wdog NMI was enabled and barked during the NMI IRQ.
  CHECK(nmi_state.wdog_enabled && nmi_state.wdog_barked,
        "Wdog NMI state not expected:\n\t"
        "wdog_enable:%x\n\twdog_raised:%"
        "x\n",
        nmi_state.wdog_enabled, nmi_state.wdog_barked);

  // Check that the wdog NMI is enable and is cleared.
  CHECK_DIF_OK(dif_rv_core_ibex_get_nmi_state(
      &rv_core_ibex, (dif_rv_core_ibex_nmi_state_t *)&nmi_state));
  CHECK(nmi_state.wdog_enabled && !nmi_state.wdog_barked,
        "Wdog NMI state not expected:\n\t"
        "wdog_enable:%x\n\twdog_raised:%"
        "x\n",
        nmi_state.wdog_enabled, nmi_state.wdog_barked);
}

/**
 * Execute the alert handler NMI interrupt test.
 */
static void alert_handler_nmi_test(void) {
  alert_handler_config();

  // Setup to handle the incoming alert handler NMI IRQ.
  expected_isr_handler = alert_handler_handler;
  nmi_fired = false;
  nmi_state = (dif_rv_core_ibex_nmi_state_t){0};

  // Trigger the alert handler to escalate.
  // CHECK_DIF_OK(dif_aes_alert_force(&aes, kDifAesAlertRecovCtrlUpdateErr));

  LOG_INFO("Wait for alert NMI");
  IBEX_SPIN_FOR(nmi_fired, kEscalationPhase0Micros);

  LOG_INFO("alert_handler_nmi_test: counter.elapsed_ticks: %d",
           (uint32_t)counters.elapsed_ticks);
  LOG_INFO("alert_handler_nmi_test: counter.wdog_barked: %d",
           counters.wdog_barked);
  LOG_INFO("alert_handler_nmi_test: counter.alert_raised: %d",
           counters.alert_raised);

  dif_alert_handler_irq_state_snapshot_t snapshot;
  CHECK_DIF_OK(dif_alert_handler_irq_get_state(&alert_handler, &snapshot));

  // We expect class A to be in escalation phase 0 at this point.
  CHECK(alert_state == kDifAlertHandlerClassStatePhase0,
        "Alert handler class A is expected to be in phase 0.");
  alert_state = kDifAlertHandlerClassStateIdle;

  // Check that the alert handler NMI was enabled and raised an alert during the
  // NMI IRQ.
  CHECK(nmi_state.alert_enabled && nmi_state.alert_raised,
        "Alert handler NMI state not expected:\n\t"
        "alert_enable:%x\n\talert_raised:%x\n",
        nmi_state.alert_enabled, nmi_state.alert_raised);

  // Check that the alert handler NMI is enabled and is cleared.
  CHECK_DIF_OK(dif_rv_core_ibex_get_nmi_state(
      &rv_core_ibex, (dif_rv_core_ibex_nmi_state_t *)&nmi_state));
  CHECK(nmi_state.alert_enabled && !nmi_state.alert_raised,
        "Alert handler NMI state not expected:\n\t"
        "alert_enable:%x\n\talert_raised:%x\n",
        nmi_state.alert_enabled, nmi_state.alert_raised);
}

/**
 * OTTF external NMI internal IRQ handler.
 * This functions overrides the OTTF weak definition.
 * It calls the `expected_isr_handler` function pointer that is previously set
 * by the test to handler the specific peripheral IRQ.
 */
void ottf_external_nmi_handler(uint32_t *exc_info) {
  nmi_fired = true;

  CHECK_DIF_OK(
      dif_rv_timer_counter_set_enabled(&timer, kHart, kDifToggleDisabled));
  CHECK_DIF_OK(dif_rv_timer_counter_read(&timer, kHart,
                                         (uint64_t *)&counters.elapsed_ticks));

  CHECK_DIF_OK(dif_rv_core_ibex_get_nmi_state(
      &rv_core_ibex, (dif_rv_core_ibex_nmi_state_t *)&nmi_state));
  CHECK_DIF_OK(
      dif_rv_core_ibex_nmi_disable(&rv_core_ibex, kDifRvCoreIbexNmiSourceAll));
  CHECK_DIF_OK(dif_rv_core_ibex_clear_nmi_state(&rv_core_ibex,
                                                kDifRvCoreIbexNmiSourceAll));
  if (nmi_state.wdog_barked) {
    counters.wdog_barked++;
    wdog_handler();
  }
  if (nmi_state.alert_raised) {
    counters.alert_raised++;
    alert_handler_handler();
  }
}

/**
 * OTTF external IRQ handler
 * This functions overrides the OTTF weak definition.
 */
void ottf_external_isr(uint32_t *exc_info) { ext_irq_fired = true; }

/**
 * Initialized all peripherals used in this test.
 */
void init_peripherals(void) {
  CHECK_DIF_OK(
      dif_aes_init(mmio_region_from_addr(TOP_EARLGREY_AES_BASE_ADDR), &aes));

  mmio_region_t addr =
      mmio_region_from_addr(TOP_EARLGREY_RV_CORE_IBEX_CFG_BASE_ADDR);
  CHECK_DIF_OK(dif_rv_core_ibex_init(addr, &rv_core_ibex));

  addr = mmio_region_from_addr(TOP_EARLGREY_AON_TIMER_AON_BASE_ADDR);
  CHECK_DIF_OK(dif_aon_timer_init(addr, &aon_timer));

  addr = mmio_region_from_addr(TOP_EARLGREY_ALERT_HANDLER_BASE_ADDR);
  CHECK_DIF_OK(dif_alert_handler_init(addr, &alert_handler));

  addr = mmio_region_from_addr(TOP_EARLGREY_PWRMGR_AON_BASE_ADDR);
  CHECK_DIF_OK(dif_pwrmgr_init(addr, &pwrmgr));

  CHECK_DIF_OK(dif_rv_timer_init(
      mmio_region_from_addr(TOP_EARLGREY_RV_TIMER_BASE_ADDR), &timer));
  CHECK_DIF_OK(dif_rv_timer_reset(&timer));
}
/**
 * This test do the following steps:
 * - Trigger a watchdog bark to generate a NMI.
 * - Check rv_core_ibex's NMI interrupt register and clear the interrupt.
 * - Repeat the previous steps with alert handler.
 */
bool test_main(void) {
  // Disable external interrupts via the PLIC entirely.
  // The NMI should still get through to the processor regardless.
  irq_global_ctrl(false);
  irq_external_ctrl(false);
  init_peripherals();

  // Execute both NMI tests several times back-to-back to verify that
  // the causes are successfully cleared in the NMI handlers.
  for (int k = 0; k < kNumTestIterations; ++k) {
    // Enable both NMIs.
    CHECK_DIF_OK(dif_rv_core_ibex_enable_nmi(&rv_core_ibex,
                                             kDifRvCoreIbexNmiSourceWdog));
    CHECK_DIF_OK(dif_rv_core_ibex_enable_nmi(&rv_core_ibex,
                                             kDifRvCoreIbexNmiSourceAlert));

    LOG_INFO("Test iteration %d begin", k);
    alert_handler_nmi_test();

    CHECK_DIF_OK(dif_rv_core_ibex_nmi_disable(&rv_core_ibex,
                                              kDifRvCoreIbexNmiSourceAlert));
    wdog_nmi_test();
    LOG_INFO("Test iteration %d end", k);
    CHECK_DIF_OK(dif_rv_core_ibex_nmi_disable(&rv_core_ibex,
                                              kDifRvCoreIbexNmiSourceWdog));
  }

  // Double check that no external interrupts have occurred.
  CHECK(ext_irq_fired == false, "Unexpected external interrupt triggered.");
  // Double check that the system has not been reset due to escalation and that
  // the reset reason is still POR.
  return UNWRAP(pwrmgr_testutils_is_wakeup_reason(&pwrmgr, 0));
}
