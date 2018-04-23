#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <stdio.h>

void InitButtonS1() {
    /* upper switch S1 on BoostXL */
    GPIO_setAsInputPin (GPIO_PORT_P5, GPIO_PIN1);
    GPIO_clearInterruptFlag(GPIO_PORT_P5, GPIO_PIN1);
}

int ButtonS1Pressed() {
    return (GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN1) == 0);
}
void enableButtonS1Interrupt() {
    GPIO_enableInterrupt(GPIO_PORT_P5, GPIO_PIN1);
    Interrupt_enableInterrupt(INT_PORT5);
}

void InitLED() {
    /* red Color LED on BoostXL */
    GPIO_setAsOutputPin    (GPIO_PORT_P2,    GPIO_PIN6);
    GPIO_setOutputLowOnPin (GPIO_PORT_P2,    GPIO_PIN6);
}

void Toggle_Booster_LED(){
    GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN6);
}

#define WATCHDOG_A RESET_SRC_1

int main(void) {
  volatile int ii;

  // Stop WDT
  WDT_A_holdTimer();

  /* If the watchdog just reset us,
   * we want to toggle a GPIO quickly to illustrate
   * that the watchdog timed out.
   */
  if (ResetCtl_getSoftResetSource() & WATCHDOG_A) {
      InitLED();
      while(1) {
          Toggle_Booster_LED();
          for(ii=0;ii<40000;ii++) ;
      }
  }

  /* MCLK to REFO at 128Khz for LF mode and SMCLK to REFO */
  CS_setReferenceOscillatorFrequency(CS_REFO_128KHZ);
  CS_initClockSignal(CS_MCLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
  CS_initClockSignal(CS_HSMCLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
  CS_initClockSignal(CS_SMCLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);
  PCM_setPowerState(PCM_AM_LF_VCORE0);

  InitButtonS1();
  InitLED();

  /* Configuring WDT to timeout after 512k iterations of SMCLK, at 128k,
   * this will roughly equal 4 seconds*/
  SysCtl_setWDTTimeoutResetType(SYSCTL_SOFT_RESET);
  WDT_A_initWatchdogTimer(WDT_A_CLOCKSOURCE_SMCLK,
                          WDT_A_CLOCKITERATIONS_512K);

  /* SysTick to wake up every 128000 clocks */
  SysTick_enableModule();
  SysTick_setPeriod(128000);
  SysTick_enableInterrupt();

  /* Enabling interrupts and starting the watchdog timer*/
  enableButtonS1Interrupt();
  Interrupt_enableMaster();

  WDT_A_startTimer();

  while (1) {
      /* go to sleep. This disables the CPU while keeping all peripherals active */
      PCM_gotoLPM0();

      /* each time we wake up (from Systick Interrupt) we continue here */
      WDT_A_clearTimer();
  }

}

void SysTick_Handler(void) {
    Toggle_Booster_LED();
    return;
}

/* GPIO ISR for button press - When a button is pressed */
void PORT5_IRQHandler(void) {
    int status;

    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P5);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5, status);

    /* When we press S1, further interrupts from PORT_P5 PIN1 are disabled */
    if (status & GPIO_PIN1) {
        SysTick_disableInterrupt();
    }
}
