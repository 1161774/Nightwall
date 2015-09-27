//*****************************************************************************
//
// reload_interrupt.c - Example demonstrating the PWM interrupt.
//
// Copyright (c) 2010-2015 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision 2.1.1.71 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"

#include <stdlib.h>


struct _LED{
	uint32_t Position;
	uint32_t Target;
	uint32_t Speed;
	uint32_t Delay;
};

struct _LED BlueLED;
struct _LED GreenLED;
struct _LED RedLED;

uint32_t g_MaxVal = 0;





uint32_t randint(uint32_t n) {
  if ((n - 1) == 4294967295) {
    return 2*rand();
  } else {
    // Chop off all of the values that would cause skew...
    uint32_t end = 4294967295 / n; // truncate skew
    end *= n;

    // ... and ignore results from rand() that fall above that limit.
    // (Worst case the loop condition should succeed 50% of the time,
    // so we can expect to bail out of this loop pretty quickly.)
    uint32_t r;
    while ((r = 2*rand()) >= end);

    return r % n;
  }
}



//*****************************************************************************
//
// The interrupt handler for the for PWM interrupts.
//
//*****************************************************************************

void
PWM1IntHandler(void)
{
    //
    // Clear the PWM0 LOAD interrupt flag.  This flag gets set when the PWM
    // counter gets reloaded.
    //
    PWMGenIntClear(PWM1_BASE, PWM_GEN_1, PWM_INT_CNT_LOAD);

    if(GreenLED.Position <= (GreenLED.Target - GreenLED.Speed))
    {
    	GreenLED.Position += GreenLED.Speed;
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, GreenLED.Position);
    }
    else if (GreenLED.Position >= (GreenLED.Target + GreenLED.Speed))
    {
    	GreenLED.Position -= GreenLED.Speed;
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, GreenLED.Position);
    }
    else if(--GreenLED.Delay < 5)
    {
	    GreenLED.Target = randint(64000);
	    GreenLED.Speed = randint(64);
	    GreenLED.Delay = randint(5000);
	    if(GreenLED.Target < GreenLED.Speed)
	    {
	    	GreenLED.Target = GreenLED.Speed;
	    }
    }
}

void
PWM2IntHandler(void)
{
    //
    // Clear the PWM0 LOAD interrupt flag.  This flag gets set when the PWM
    // counter gets reloaded.
    //
    PWMGenIntClear(PWM1_BASE, PWM_GEN_2, PWM_INT_CNT_LOAD);

    if(RedLED.Position <= (RedLED.Target - RedLED.Speed))
    {
    	RedLED.Position += RedLED.Speed;
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, RedLED.Position);
    }
    else if (RedLED.Position >= (RedLED.Target + RedLED.Speed))
    {
    	RedLED.Position -= RedLED.Speed;
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, RedLED.Position);
    }
    else if(--RedLED.Delay < 5)
    {
    	RedLED.Target = randint(64000);
    	RedLED.Speed = randint(64);
    	RedLED.Delay = randint(5000);
	    if(RedLED.Target < RedLED.Speed)
	    {
	    	RedLED.Target = RedLED.Speed;
	    }
    }
}

void
PWM3IntHandler(void)
{
    //
    // Clear the PWM0 LOAD interrupt flag.  This flag gets set when the PWM
    // counter gets reloaded.
    //
    PWMGenIntClear(PWM1_BASE, PWM_GEN_3, PWM_INT_CNT_LOAD);

    if(BlueLED.Position <= (BlueLED.Target - BlueLED.Speed))
    {
    	BlueLED.Position += BlueLED.Speed;
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, BlueLED.Position);
    }
    else if (BlueLED.Position >= (BlueLED.Target + BlueLED.Speed))
    {
    	BlueLED.Position -= BlueLED.Speed;
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, BlueLED.Position);
    }
    else if(--BlueLED.Delay < 5)
    {
    	BlueLED.Target = randint(64000);
    	BlueLED.Speed = randint(64);
    	BlueLED.Delay = randint(5000);
	    if(BlueLED.Target < BlueLED.Speed)
	    {
	    	BlueLED.Target = BlueLED.Speed;
	    }
    }
}


//*****************************************************************************
//
// Configure PWM0 for a load interrupt.  This interrupt will trigger everytime
// the PWM0 counter gets reloaded.  In the interrupt, 0.1% will be added to
// the current duty cycle.  This will continue until a duty cycle of 75% is
// received, then the duty cycle will get reset to 0.1%.
//
//*****************************************************************************
int
main(void)
{
    //
    // Set the clocking to run directly from the external crystal/oscillator.
    // TODO: The SYSCTL_XTAL_ value must be changed to match the value of the
    // crystal on your board.
    //

    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);

    //
    // Set the PWM clock to the system clock.
    //
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    //
    // The PWM peripheral must be enabled for use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

    //
    // For this example PWM0 is used with PortB Pin6.  The actual port and pins
    // used may be different on your part, consult the data sheet for more
    // information.  GPIO port B needs to be enabled so these pins can be used.
    // TODO: change this to whichever GPIO port you are using.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Seed the random number with the current value in the PWM 1_1 count register
    srand((*((volatile uint32_t *)0x40029094)));

    GreenLED.Position = randint(64000);
    GreenLED.Target = randint(64000);
    GreenLED.Speed = randint(64);
    GreenLED.Delay = randint(5000);

    RedLED.Position = randint(64000);
    RedLED.Target = randint(64000);
    RedLED.Speed = randint(64);
    RedLED.Delay = randint(5000);

    BlueLED.Position = randint(64000);
    BlueLED.Target = randint(64000);
    BlueLED.Speed = randint(64);
    BlueLED.Delay = randint(5000);


    //
    // Configure the GPIO pin muxing to select PWM00 functions for these pins.
    // This step selects which alternate function is available for these pins.
    // This is necessary if your part supports GPIO pin function muxing.
    // Consult the data sheet to see which functions are allocated per pin.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinConfigure(GPIO_PE4_M1PWM2);
    GPIOPinConfigure(GPIO_PF1_M1PWM5);
    GPIOPinConfigure(GPIO_PF2_M1PWM6);

    //
    // Configure the PWM function for this pin.
    // Consult the data sheet to see which functions are allocated per pin.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);


    //
    // Configure the PWM0 to count down without synchronization.
    //
    PWMGenConfigure(PWM1_BASE, PWM_GEN_1,
                    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_2,
                    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3,
                    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    //
    // Set the PWM period to 250Hz.  To calculate the appropriate parameter
    // use the following equation: N = (1 / f) * SysClk.  Where N is the
    // function parameter, f is the desired frequency, and SysClk is the
    // system clock frequency.
    // In this case you get: (1 / 250Hz) * 16MHz = 64000 cycles.  Note that
    // the maximum period you can set is 2^16.
    //
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, 64000);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, 64000);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, 64000);
    //
    // For this example the PWM0 duty cycle will be variable.  The duty cycle
    // will start at 0.1% (0.01 * 64000 cycles = 640 cycles) and will increase
    // to 75% (0.5 * 64000 cycles = 32000 cycles).  After a duty cycle of 75%
    // is reached, it is reset to 0.1%.  This dynamic adjustment of the pulse
    // width is done in the PWM0 load interrupt, which increases the duty
    // cycle by 0.1% everytime the reload interrupt is received.
    //
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, GreenLED.Position);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, RedLED.Position);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, BlueLED.Position);

    //
    // Enable processor interrupts.
    //
    IntMasterEnable();

    //
    // Allow PWM0 generated interrupts.  This configuration is done to
    // differentiate fault interrupts from other PWM0 related interrupts.
    //
    PWMIntEnable(PWM1_BASE, PWM_INT_GEN_1);
    PWMIntEnable(PWM1_BASE, PWM_INT_GEN_2);
    PWMIntEnable(PWM1_BASE, PWM_INT_GEN_3);

    //
    // Enable the PWM0 LOAD interrupt on PWM0.
    //
    PWMGenIntTrigEnable(PWM1_BASE, PWM_GEN_1, PWM_INT_CNT_LOAD);
    PWMGenIntTrigEnable(PWM1_BASE, PWM_GEN_2, PWM_INT_CNT_LOAD);
    PWMGenIntTrigEnable(PWM1_BASE, PWM_GEN_3, PWM_INT_CNT_LOAD);

    //
    // Enable the PWM0 interrupts on the processor (NVIC).
    //
    IntEnable(INT_PWM1_1);
    IntEnable(INT_PWM1_2);
    IntEnable(INT_PWM1_3);

    //
    // Enable the PWM0 output signal (PD0).
    //
    PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
    PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, true);
    PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true);

    //
    // Enables the PWM generator block.
    //
    PWMGenEnable(PWM1_BASE, PWM_GEN_1);
    PWMGenEnable(PWM1_BASE, PWM_GEN_2);
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);

    //
    // Loop forever while the PWM signals are generated and PWM0 interrupts
    // get received.
    //
    while(1)
    {
    	if(GreenLED.Target > g_MaxVal) g_MaxVal = GreenLED.Target;
    	if(RedLED.Target > g_MaxVal) g_MaxVal = RedLED.Target;
    	if(BlueLED.Target > g_MaxVal) g_MaxVal = BlueLED.Target;
    }
}
