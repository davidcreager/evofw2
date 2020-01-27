/***************************************************************************
** bitstream.c
**
** The radio transmits the bytes of a packet as an 8N1 UART
**  i.e. each byte is framed with start/stop bitstream s<XX>p (s=0, p=1)
**
** A packet has the following format
** <training><sync word><message><training>
**
** <training> is an alternating sequence of 1's and 0's defining the bit rate
** <sync word> is a recognisable pattern to define byte alignment of the message
** <message> is a sequence of bytes defining the message
** a packet is followed by a further training sequence.
**
** The function of bitstream is to extract the message data bits from the RX stream
** and insert teh data bits into the TX stream.
** i.e determine the alignment of the message ytes and iremove/insert the start/stop bits
**
** The radio can be programmed to look for preamble and identify a 16 bit SYNC WORD
** Although the preamble is ideally several bytes in length the HGI80 only appears
** to transmit a few bits.
**
** bitstream needs to define a SYNC WORD that maximises the possibility of correctly
** identifying RX packets and simplifying the generation of TX packets
**
** The nominal value of a packet bitstream is
**    s<  AA  >ps<  FF  >ps<  00  >ps<  m1  >ps<  m2  >p...
**    0101010101011111111100000000010MMMMMMMM10mmmmmmmm1...
**
** Because of the restricted preamble transmitted by the HGI80 we want to allow
** as many bits as possible to be interpreted as preamble.
**
** So we will select the second 1 of the FF to be the first bit of the SYNC WORD
** to be detected by the radio making the SYNC WORD [FF00]
** 
**            s<  AA  >ps<  FF  >ps<  00  >ps<  m1  >ps<  m2  >p...
**            0101010101011111111100000000010MMMMMMMM10mmmmmmmm1...
** PREAMBLE    ...  010101
** SYNC WORD              <   FF     00  >
** ALIGNMENT                              01
** MESSAGE                                  s<  m1  >ps<  m2  >p...
**
** This leaves two bits of the stream as alignmnet before the start bit of the
** first message byte
**
** When transmitting the radio will automatically insert preamble and the SYNC WORD
** bitstream will need to insert the two ALIGNMENT bits before the message bytes
**
*/
#include <stdint.h>
#include <string.h>

#include "tty.h"

#include "radio.h"
#include "message.h"
#include "bitstream.h"


#define SYNC_WORD  0xFF00
#define ALIGNMENT  0x02
#define ALIGN_BITS 2

uint16_t bs_sync_word(void) { return SYNC_WORD; }

/*****************************************************************
* NOTE: The following shift_register structure is sensitive to
*       the endianness used by the MCU.  It may be necessary to
*       swap the order of .bits and .data
*
* The desired behaviour is that when .reg is left shifted the
* msb of .bits becomes the lsb of .data
*/
union shift_register {
  uint16_t reg;
  struct {
#if __BYTE_ORDER__ ==__ORDER_LITTLE_ENDIAN__
    uint8_t bits;
    uint8_t data;
#endif
#if __BYTE_ORDER__ ==__ORDER_BIG_ENDIAN__
    uint8_t data;
    uint8_t bits;
#endif
  };
};

struct bs_frame {
  uint16_t nOctets;
  uint16_t pktLen;
  
  uint8_t nBytes;
  uint8_t msgLen;

  uint8_t nBits;	// number of valid data bits in shift register
  union shift_register sr;
};

/**************************************************************************
* Raw octet trace
*/
static uint8_t rxOctet[256];
static uint8_t rxIn = 0;
static uint8_t rxOut = 0;

void bs_rx_raw( uint8_t start, uint8_t finish ) {

  if( start != finish ) {
    finish -= 1;
  }

  while( start!=finish ) {
    tty_write_char('.');
    start = ( start+1 ) % sizeof(rxOctet);
    tty_write_hex( rxOctet[start] );
    if( start==finish )
      tty_write_str("\r\n");
  }
    
}
/**************************************************************************
* convert message bytes to packet octets
*/
static uint16_t bytes2octets(uint8_t bytes) {
  uint16_t pktBits;
  
  pktBits = bytes;
  pktBits *= 10;          // bits including start/stop
  pktBits += ALIGN_BITS;
  pktBits += 7;           // Round up

  return pktBits/8;
}

/**************************************************************************
* RX bitstream
* Radio pushs octets to bitstream
*/

static struct bs_frame rx;
static void bs_rx_reset(void) { memset( &rx, 0, sizeof(rx) ); }

static inline void discard(uint8_t n) { rx.sr.bits <<= n; } 
static inline void keep(uint8_t n ) { rx.sr.reg <<= n; } 
static inline void rxData(void) { msg_rx_byte(rx.sr.data); rx.nBytes++; }

void bs_rx_sof(void) {
//tty_write_char('(');
  bs_rx_reset();
  msg_rx_sof(rxOut);
}

void bs_rx_rssi(uint8_t rssi){
  msg_rx_rssi(rssi);
}

void bs_rx_timeout(void) {
  msg_rx_timeout();
}

void bs_rx_eof(void) {
  bs_rx_octet( 0xAA );	// Flush any outstanding bits
  msg_rx_eof(rxOut);
//tty_write_char(')');
}

void bs_rx_octet(uint8_t octet)
{
  rx.sr.bits = octet;

  rxOctet[rxOut] = octet;
  rxOut = ( rxOut+1 ) % sizeof(rxOctet);
  if( rxOut==rxIn )
    rxIn = ( rxIn+1 ) % sizeof(rxOctet);
    
//  tty_write_char('.');

  if( rx.nOctets==0 ) { // first octet - deal with alignment
    discard(ALIGN_BITS);
	  discard(1);           // discard start bit
	  keep(5);
	  rx.nBits = 5;
  } else {
    // Every 5 RX octets contain 4 message bytes
    switch( rx.nBits ) { // Consume the 8 bits of this octet
    case 9:                    discard(1); keep(7); rx.nBits = 7; break;
    case 7: keep(1); rxData(); discard(2); keep(5); rx.nBits = 5; break;
    case 5: keep(3); rxData(); discard(2); keep(3); rx.nBits = 3; break;
    case 3: keep(5); rxData(); discard(2); keep(1); rx.nBits = 1; break;
    case 1: keep(7); rxData(); discard(1);          rx.nBits = 9; break;
    }
  };

  rx.nOctets++;
}

void bs_rx_msgLen(uint8_t bytes) {
  rx.msgLen = bytes;
  rx.pktLen = bytes2octets(bytes);
  
  radio_rx_pktLen(rx.pktLen);
}

/**************************************************************************
* TX bitstream
* Radio pulls octets from bitstream
*/

static struct bs_frame tx;
static void bs_tx_reset(void) { memset( &tx, 0, sizeof(tx) ); }

static inline void add_S(void) { tx.sr.bits >>=1 ;                  }
static inline void addP_(void) { tx.sr.bits >>=1 ; tx.sr.bits |= 0x80; }
static inline void addPS(void) { addP_(); add_S(); }
static inline void send(uint8_t n) { tx.sr.reg >>= n; } 
static inline void txOctet(void) { radio_tx_octet( tx.sr.bits); tx.nOctets++; }

void bs_tx_sof(void) {
  bs_tx_reset();
  radio_tx_sof();
}

void bs_tx_eof(void) {
  radio_tx_eof();
}

void bs_tx_octet(void)
{
  if( tx.nOctets==0 ) { // first octet - deal with alignment
    tx.sr.data = ALIGNMENT;
    send(ALIGN_BITS);
	  add_S();
	  tx.nBits = 3;
  } 

  // Pull next byte
  tx.sr.data = msg_tx_byte();
  
  // We generate 5 octets for evry 4 data bytes
  switch( tx.nBits ) { // Consume the 8 bits of this byte
  case 1: send(7); txOctet(); send(1); addPS(); tx.nBits = 9; break;
  case 3: send(5); txOctet(); send(3); addPS(); tx.nBits = 5; break;
  case 5: send(3); txOctet(); send(5); addPS(); tx.nBits = 7; break;
  case 7: send(1); txOctet(); send(7); add_S(); 
          // Fall through for next octet we already have available
                   txOctet();          addP_(); tx.nBits = 1; break;
  };

  tx.nBytes++;
  
  if( tx.msgLen && tx.msgLen==tx.nBytes )
    bs_tx_eof();
}

void bs_tx_msgLen(uint8_t bytes) {
  tx.msgLen = bytes;
  tx.pktLen = bytes2octets(bytes);

//  radio_tx_pktLen(rx.pktLen);
}

