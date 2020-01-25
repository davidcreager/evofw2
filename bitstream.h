/***********************************************************************
 *
 * bitstream.h
 *
 */

#ifndef _BITSTREAM_H_
#define _BITSTREAM_H_

extern uint16_t bs_sync_word(void);


/**************************************************************************
* RX bitstream
* Radio pushs octets to bitstream
*/

extern void bs_rx_sof(void);
extern void bs_rx_rssi(uint8_t rssi);
extern void bs_rx_timeout(void);
extern void bs_rx_eof(void);
extern void bs_rx_octet(uint8_t octet);
extern void bs_rx_msgLen(uint8_t bytes);

/**************************************************************************
* TX bitstream
* Radio pulls octets from bitstream
*/

extern void bs_tx_sof(void);
extern void bs_tx_eof(void);
extern void bs_tx_octet(void);
extern void bs_tx_msgLen(uint8_t bytes);

#endif
