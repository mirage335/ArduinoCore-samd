/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "sam.h"
#include "variant.h"

#include <stdio.h>

#define NVM_SW_CALIB_DFLL48M_COARSE_VAL   (58)
#define NVM_SW_CALIB_DFLL48M_FINE_VAL     (64)


/**
 * \brief system_init() configures the needed clocks and according Flash Read Wait States.
 * At reset:
 * - OSC8M clock source is enabled with a divider by 8 (1MHz).
 * - Generic Clock Generator 0 (GCLKMAIN) is using OSC8M as source.
 * We need to:
 * 1) Enable XOSC32K clock (External on-board 32.768Hz oscillator), will be used as DFLL48M reference.
 * 2) Put XOSC32K as source of Generic Clock Generator 1
 * 3) Put Generic Clock Generator 1 as source for Generic Clock Multiplexer 0 (DFLL48M reference)
 * 4) Enable DFLL48M clock
 * 5) Switch Generic Clock Generator 0 to DFLL48M. CPU will run at 48MHz.
 * 6) Modify PRESCaler value of OSCM to have 8MHz
 * 7) Put OSC8M as source for Generic Clock Generator 3
 */
// Constants for Clock generators
#define GENERIC_CLOCK_GENERATOR_MAIN      (0u)
#define GENERIC_CLOCK_GENERATOR_OSC32K   (1u)
#define GENERIC_CLOCK_GENERATOR_OSCULP32K (2u) /* Initialized at reset for WDT */
#define GENERIC_CLOCK_GENERATOR_OSC8M     (3u)
#define GENERIC_CLOCK_GENERATOR_OSC1M     (5u)
#define GENERIC_CLOCK_GENERATOR_USB     (6u)
// Constants for Clock multiplexers
#define GENERIC_CLOCK_MULTIPLEXER_DFLL48M (0u)




void SystemInit(void)
{
  /* Set 1 Flash Wait State for 48MHz, cf tables 20.9 and 35.27 in SAMD21 Datasheet */
  NVMCTRL->CTRLB.bit.RWS = 1;//NVMCTRL_CTRLB_RWS_HALF_Val;

  /* Turn on the digital interface clock */
  PM->APBAMASK.reg |= PM_APBAMASK_GCLK;


  

  /* ----------------------------------------------------------------------------------------------
   * 1) Enable OSC32K clock (Internal on-board 32.768Hz oscillator)
   */
  SYSCTRL->OSC32K.bit.CALIB =
			((*(uint32_t *)SYSCTRL_FUSES_OSC32K_ADDR >> 
			SYSCTRL_FUSES_OSC32K_Pos) & 0x7Ful);

  SYSCTRL->OSC32K.reg = SYSCTRL_OSC32K_STARTUP( 0x6u ) | /* cf table 15.10 of product datasheet in chapter 15.8.6 */
                         SYSCTRL_OSC32K_EN32K;
  SYSCTRL->OSC32K.bit.CALIB =
			((*(uint32_t *)SYSCTRL_FUSES_OSC32K_ADDR >> 
			SYSCTRL_FUSES_OSC32K_Pos) & 0x7Ful);
                       
  SYSCTRL->OSC32K.bit.ENABLE = 1; /* separate call, as described in chapter 15.6.3 */

  while (  (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_OSC32KRDY) == 0 )
  {
    /* Wait for oscillator stabilization */
  }
  
  /* Software reset the module to ensure it is re-initialized correctly */
  /* Note: Due to synchronization, there is a delay from writing CTRL.SWRST until the reset is complete.
   * CTRL.SWRST and STATUS.SYNCBUSY will both be cleared when the reset is complete, as described in chapter 13.8.1
   */
  GCLK->CTRL.reg = GCLK_CTRL_SWRST;

  while ( (GCLK->CTRL.reg & GCLK_CTRL_SWRST) && (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) )
  {
    /* Wait for reset to complete */
  }

  /* Set OSC8M divisor to 1, CPU is now running at 8MHz */
  SYSCTRL->OSC8M.bit.PRESC = SYSCTRL_OSC8M_PRESC_1_Val;
  SYSCTRL->OSC8M.bit.ONDEMAND = 0;
  

  GCLK->GENDIV.reg = GCLK_GENDIV_ID( GENERIC_CLOCK_GENERATOR_OSC1M ) | GCLK_GENDIV_DIV(3); // Generic Clock Generator 5, divided by 8 (2^3)
  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )  {    /* Wait for synchronization */  }
  

  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( GENERIC_CLOCK_GENERATOR_OSC1M ) | // Generic Clock Generator 5
                      GCLK_GENCTRL_SRC_OSC8M | // Selected source is 8MHz Oscillator
                      GCLK_GENCTRL_DIVSEL| //divided by 8
//                      GCLK_GENCTRL_OE | // Output clock to a pin for tests
                      GCLK_GENCTRL_GENEN;

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )  {    /* Wait for synchronization */  }


  
  /* ----------------------------------------------------------------------------------------------
   * 2) Put OSC32K as source of Generic Clock Generator 1
   */
  GCLK->GENDIV.reg = GCLK_GENDIV_ID( GENERIC_CLOCK_GENERATOR_OSC32K ); // Generic Clock Generator 1

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }

  /* Write Generic Clock Generator 1 configuration */
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( GENERIC_CLOCK_GENERATOR_OSC32K ) | // Generic Clock Generator 1
                      GCLK_GENCTRL_SRC_OSC32K | // Selected source is Internal 32KHz Oscillator
//                      GCLK_GENCTRL_OE | // Output clock to a pin for tests
                      GCLK_GENCTRL_GENEN;



  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }
  

/* Set PLL input to GCLK5 and lock timer input to GCLK1*/

  uint32_t new_clkctrl_config = (0x1u << GCLK_CLKCTRL_ID_Pos) |  //PLL input clock source for reference
                                (0x5u << GCLK_CLKCTRL_GEN_Pos);  // is set to clock gen 5
  
  *((uint8_t*)&GCLK->CLKCTRL.reg) = 0x1u; //pll input source
  uint32_t prev_gen_id = GCLK->CLKCTRL.bit.GEN;
  GCLK->CLKCTRL.bit.GEN = 0;

	/* Disable the generic clock */
	GCLK->CLKCTRL.reg &= ~GCLK_CLKCTRL_CLKEN;
	while (GCLK->CLKCTRL.reg & GCLK_CLKCTRL_CLKEN) {
		/* Wait for clock to become disabled */
	}

	/* Restore previous configured clock generator */
	GCLK->CLKCTRL.bit.GEN = prev_gen_id;
    GCLK->CLKCTRL.reg = new_clkctrl_config;
    *((uint8_t*)&GCLK->CLKCTRL.reg) = 0x1u;

	/* Enable the generic clock */
	GCLK->CLKCTRL.reg |= GCLK_CLKCTRL_CLKEN;
    
    new_clkctrl_config = (0x2u << GCLK_CLKCTRL_ID_Pos) |  //PLL lock clock source for reference
                                (0x1u << GCLK_CLKCTRL_GEN_Pos);  // is set to clock gen 1
  
  *((uint8_t*)&GCLK->CLKCTRL.reg) = 0x2u; //pll input source
  prev_gen_id = GCLK->CLKCTRL.bit.GEN;
  GCLK->CLKCTRL.bit.GEN = 0;

	/* Disable the generic clock */
	GCLK->CLKCTRL.reg &= ~GCLK_CLKCTRL_CLKEN;
	while (GCLK->CLKCTRL.reg & GCLK_CLKCTRL_CLKEN) {
		/* Wait for clock to become disabled */
	}

	/* Restore previous configured clock generator */
	GCLK->CLKCTRL.bit.GEN = prev_gen_id;
    GCLK->CLKCTRL.reg = new_clkctrl_config;
    *((uint8_t*)&GCLK->CLKCTRL.reg) = 0x2u;

	/* Enable the generic clock */
	GCLK->CLKCTRL.reg |= GCLK_CLKCTRL_CLKEN;
    
    
/*configure PLL*/
    //integer multiplier (LDR) needs to be 95 , as output frequency is input*(LDR+1+(LDRFRAC/16))/2
    // I have no idea where the /2 is coming from
    //LDRFRAC will be zero
    SYSCTRL->DPLLCTRLA.reg = 0; // disable on demand mode and run in standby
    SYSCTRL->DPLLRATIO.reg = SYSCTRL_DPLLRATIO_LDR(95);
    SYSCTRL->DPLLCTRLB.reg =    SYSCTRL_DPLLCTRLB_REFCLK_GCLK;
    
    
    SYSCTRL->DPLLCTRLA.reg |= SYSCTRL_DPLLCTRLA_ENABLE;
  
  
  /* */
  /* ----------------------------------------------------------------------------------------------
   * 3) Put Generic Clock Generator 1 as source for Generic Clock Multiplexer 0 (DFLL48M reference)
   */
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( GENERIC_CLOCK_MULTIPLEXER_DFLL48M ) | // Generic Clock Multiplexer 0
                      GCLK_CLKCTRL_GEN_GCLK1 | // Generic Clock Generator 1 is source
                      GCLK_CLKCTRL_CLKEN;

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }

  /* ----------------------------------------------------------------------------------------------
   * 4) Enable DFLL48M clock
   */

   SYSCTRL->DFLLCTRL.bit.ONDEMAND = 0;
   
   while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0 )
   {
   /* Wait for synchronization */
   }
   uint32_t temp_genctrl;
   GCLK_GENCTRL_Type genctrl;
   genctrl.reg=0;
   SYSCTRL_DFLLCTRL_Type dfllctrl_conf;
   dfllctrl_conf.reg=0;
   SYSCTRL_DFLLVAL_Type dfllval_conf;
   dfllval_conf.reg=0;
   
   uint32_t coarse =( *((uint32_t *)(NVMCTRL_OTP4) + 
                        (NVM_SW_CALIB_DFLL48M_COARSE_VAL / 32)) 
                         >> (NVM_SW_CALIB_DFLL48M_COARSE_VAL % 32))
                         & ((1 << 6) - 1);
   if (coarse == 0x3f) {
        coarse = 0x1f;
   }
   uint32_t fine =( *((uint32_t *)(NVMCTRL_OTP4)
                    + (NVM_SW_CALIB_DFLL48M_FINE_VAL / 32))
                    >> (NVM_SW_CALIB_DFLL48M_FINE_VAL % 32))
                    & ((1 << 10) - 1);
   if (fine == 0x3ff) {
        fine = 0x1ff;
   }
   dfllval_conf.bit.COARSE = coarse;
   dfllval_conf.bit.FINE = fine;
   dfllctrl_conf.bit.USBCRM = 1;
   dfllctrl_conf.bit.BPLCKC = 0;
   dfllctrl_conf.bit.QLDIS = 0;
   dfllctrl_conf.bit.CCDIS = 1;
   dfllctrl_conf.bit.ENABLE = 1;
   SYSCTRL->DFLLCTRL.bit.ONDEMAND = 0;
   while (!(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY));
   SYSCTRL->DFLLMUL.reg = 48000;
   SYSCTRL->DFLLVAL.reg = dfllval_conf.reg;
   SYSCTRL->DFLLCTRL.reg = dfllctrl_conf.reg;
   GCLK_CLKCTRL_Type clkctrl;
   clkctrl.reg=0;
   uint16_t temp;
   GCLK->CLKCTRL.bit.ID = 0; /* GCLK_ID - DFLL48M Reference */
   temp = GCLK->CLKCTRL.reg;
   clkctrl.bit.CLKEN = 1;
   clkctrl.bit.WRTLOCK = 0;
   clkctrl.bit.GEN = GCLK_CLKCTRL_GEN_GCLK3_Val;
   GCLK->CLKCTRL.reg = (clkctrl.reg | temp);


   /* Configure DFLL48M as source for GCLK_GEN1 */
   GCLK->GENCTRL.bit.ID = 6; /* GENERATOR_ID - GCLK_GEN_1 */
   while(GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
   temp_genctrl = GCLK->GENCTRL.reg;
   genctrl.bit.SRC = GCLK_GENCTRL_SRC_DFLL48M_Val;
   genctrl.bit.GENEN = 1;
   genctrl.bit.RUNSTDBY = 0;
   GCLK->GENCTRL.reg = (genctrl.reg | temp_genctrl);
   while(GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0 )
  {
    /* Wait for synchronization */
  }


  /* ----------------------------------------------------------------------------------------------
   * 5) Switch Generic Clock Generator 0 to PLL. CPU will run at 48MHz.
   */
  GCLK->GENDIV.reg = GCLK_GENDIV_ID( GENERIC_CLOCK_GENERATOR_MAIN ); // Generic Clock Generator 0

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }

  /* Write Generic Clock Generator 0 configuration */
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( GENERIC_CLOCK_GENERATOR_MAIN ) | // Generic Clock Generator 0
                      (0x8u  << GCLK_GENCTRL_SRC_Pos) | // Selected source is DPLL 48MHz
//                      (GCLK_GENCTRL_SRC_DFLL48M_Val  << GCLK_GENCTRL_SRC_Pos) | // Selected source is DPLL 48MHz
//                      GCLK_GENCTRL_OE | // Output clock to a pin for tests
                      GCLK_GENCTRL_GENEN;

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }



  /* ----------------------------------------------------------------------------------------------
   * 7) Put OSC8M as source for Generic Clock Generator 3
   */
  GCLK->GENDIV.reg = GCLK_GENDIV_ID( GENERIC_CLOCK_GENERATOR_OSC8M ); // Generic Clock Generator 3

  /* Write Generic Clock Generator 3 configuration */
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( GENERIC_CLOCK_GENERATOR_OSC8M ) | // Generic Clock Generator 3
                      GCLK_GENCTRL_SRC_OSC8M | // Selected source is RC OSC 8MHz (already enabled at reset)
//                      GCLK_GENCTRL_OE | // Output clock to a pin for tests
                      GCLK_GENCTRL_GENEN;

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }

  /*
   * Now that all system clocks are configured, we can set CPU and APBx BUS clocks.
   * These values are normally the ones present after Reset.
   */
  PM->CPUSEL.reg  = PM_CPUSEL_CPUDIV_DIV1;
  PM->APBASEL.reg = PM_APBASEL_APBADIV_DIV1_Val;
  PM->APBBSEL.reg = PM_APBBSEL_APBBDIV_DIV1_Val;
  PM->APBCSEL.reg = PM_APBCSEL_APBCDIV_DIV1_Val;


  SystemCoreClock=48000000 ;

  /* ----------------------------------------------------------------------------------------------
   * 8) Load ADC factory calibration values
   */

  // ADC Bias Calibration
  uint32_t bias = (*((uint32_t *) ADC_FUSES_BIASCAL_ADDR) & ADC_FUSES_BIASCAL_Msk) >> ADC_FUSES_BIASCAL_Pos;

  // ADC Linearity bits 4:0
  uint32_t linearity = (*((uint32_t *) ADC_FUSES_LINEARITY_0_ADDR) & ADC_FUSES_LINEARITY_0_Msk) >> ADC_FUSES_LINEARITY_0_Pos;

  // ADC Linearity bits 7:5
  linearity |= ((*((uint32_t *) ADC_FUSES_LINEARITY_1_ADDR) & ADC_FUSES_LINEARITY_1_Msk) >> ADC_FUSES_LINEARITY_1_Pos) << 5;

  ADC->CALIB.reg = ADC_CALIB_BIAS_CAL(bias) | ADC_CALIB_LINEARITY_CAL(linearity);

  /*
   * 9) Disable automatic NVM write operations
   */
  NVMCTRL->CTRLB.bit.MANW = 1;
}
