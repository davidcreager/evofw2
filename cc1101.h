/*********************************************************
*
* cc1101.h
* ========
*
* Hardware interface to TI CC1101 radio chip
*
*/

#ifndef _CC1101_H_
#define _CC1101_H_

#include <stdint.h>

extern uint8_t cc_put_octet( uint8_t octet );
extern void cc_tx_trigger(void);

extern void cc_init(void);
extern void cc_work(void);

extern void cc_set_pktlen( uint16_t len, uint16_t current );
extern void cc_check_pktLen( uint16_t current );
#endif
