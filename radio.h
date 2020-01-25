/***************************************************************************
** radio.h
*/
#ifndef _RADIO_H_
#define _RADIO_H_

extern void radio_rx_pktLen(uint16_t octets);

extern void radio_tx_sof(void);
extern void radio_tx_eof(void);
extern void radio_tx_octet(uint8_t octet);

//extern void radio_tx_pktLen(uint16_t octets);

#endif  // _RADIO_H_
