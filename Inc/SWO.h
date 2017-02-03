/*
 * SWO.h
 *
 *  Created on: 27.01.2017
 *
 */

#ifndef SWO_H_
#define SWO_H_

/*SWO DEBUG*/
void SWO_PrintChar  (char c);
void SWO_PrintString(const char *s);

/*********************************************************************
*
*       Defines for Cortex-M debug unit
*/
#define ITM_STIM_U32 (*(volatile unsigned int*)0xE0000000)    // Stimulus Port Register word acces
#define ITM_STIM_U8  (*(volatile         char*)0xE0000000)    // Stimulus Port Register byte acces
#define ITM_ENA      (*(volatile unsigned int*)0xE0000E00)    // Trace Enable Ports Register
#define ITM_TCR      (*(volatile unsigned int*)0xE0000E80)    // Trace control register




#endif /* SWO_H_ */
