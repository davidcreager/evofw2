/*********************************************************
*
* cc1101.c
* ========
*
* Hardware interface to TI CC1101 radio chip
*
*/

#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "tty.h"

#include "spi.h"
#include "config.h"
#include "bitstream.h"
#include "cc1101_const.h"
#include "cc1101.h"

#define FRAME_INT_ENTER DEBUG1_ON
#define FRAME_INT_LEAVE DEBUG1_OFF
#define FIFO_INT_ENTER  DEBUG3_ON
#define FIFO_INT_LEAVE  DEBUG3_OFF


// CC1101 register settings
static const uint8_t PROGMEM CC_REGISTER_VALUES[] = {

  CC1100_IOCFG2, 0x00,  // GDO2- FIFO interrupt
  CC1100_IOCFG1, 0x2E,  // GDO1- not used
  CC1100_IOCFG0, 0x06,  // GDO0- Frame interrupt
  
  CC1100_FIFOTHR,  0x00, // 
  CC1100_PKTLEN,   0xFF, //
  CC1100_PKTCTRL1, 0x20, //
  CC1100_PKTCTRL0, 0x02, //

  CC1100_FSCTRL1, 0x06,  //
  CC1100_FREQ2,   0x21,  //
  CC1100_FREQ1,   0x65,  //
  CC1100_FREQ0,   0x6C,  //
  
  CC1100_MDMCFG4, 0x6A,  //
  CC1100_MDMCFG3, 0x83,  // (DRATE_M=131 data rate=38,383.4838867Hz)
  CC1100_MDMCFG2, 0x16,  // (GFSK  15/16 Sync Word Carrier sense above threshold)
  CC1100_MDMCFG1, 0x22,  // (CHANSPC_E=2 NUM_PREAMBLE=4 FEC_EN=0)
  CC1100_MDMCFG0, 0xF8,  //
  CC1100_DEVIATN, 0x50,  //

  CC1100_MCSM2,   0x07,  //
  CC1100_MCSM1,   0x20,  // CCA_MODE unless currently receiving a packet, RXOFF_MODE to IDLE , TX_OFF_MODE to IDLE
  CC1100_MCSM0,   0x18,  // (0x18=11000 FS_AUTOCAL=1 When going from IDLE to RX or TX)

  CC1100_FOCCFG,  0x16,  //
  
  CC1100_AGCCTRL2, 0x43, //
  CC1100_AGCCTRL1, 0x40, //
  CC1100_AGCCTRL0, 0x91, //

  CC1100_FSCAL3,  0xE9,  //
  CC1100_FSCAL2,  0x2A,  //
  CC1100_FSCAL1,  0x00,  //
  CC1100_FSCAL0,  0x1F,  //

  CC1100_FSTEST,  0x59,  //
  CC1100_TEST2,   0x81,  //
  CC1100_TEST1,   0x35,  //
  CC1100_TEST0,   0x09,  //
  
  CC1100_PATABLE, 0xC0   //
};

// Radio states
#define RS_RX            0
#define RS_TX            1
#define RS_CHANGE_TO_TX  2
#define RS_CHANGE_TO_RX  3

static volatile uint8_t radio_state;

enum frame_state {
  FRAME_IDLE,
  FRAME_RX,
  FRAME_TX
};
static volatile uint8_t frame_state;

static uint8_t cc_read( uint8_t addr ) {
  uint8_t data ;

  spi_assert();

  while( spi_check_miso() );

  spi_send( addr | CC_READ );
  data = spi_send( 0 );

  spi_deassert();

  return data;
}

static uint8_t cc_write(uint8_t addr, uint8_t b) {
  uint8_t result;

  spi_assert();

  while( spi_check_miso() );

  spi_send(addr);
  result = spi_send(b);

  spi_deassert();
  return result;
}

/******************************************************
** The CC110 RX and TX FIFOs are shorter than 
** the length of messages we might have to handle.
** To handle packets longer than the FIFO
** we have to use Infinite packet mode
** until we reach the final buffer length of data
*/

static uint16_t PktLen = 0;	// length of current packet
static uint8_t PktMode = 3;
static uint8_t PktBuf = 0;	// Outstanding buffer boundaries

void cc_set_pktLen( uint16_t len, uint16_t current ) {
  uint8_t pktBuf,pktLen;
  uint8_t currBuf,currLen;
  uint8_t pktMode;
  
  // Protect against unexpected conditions
  if( current > len )
    len = current + 1;

  pktBuf = len / 256;
  pktLen = len % 256;
  currBuf = current / 256;
  currLen = current % 256;

#if 0
  tty_write_char('[');
  tty_write_hex(pktBuf); tty_write_char('.'); tty_write_hex(pktLen); 
  tty_write_char(':');
  tty_write_hex(currBuf); tty_write_char('.'); tty_write_hex(currLen); 
  tty_write_char(']');
#endif

  pktMode = 0;	// assume fixed packet mode
  
  PktBuf = pktBuf - currBuf ;
  
  // are we in final buffer yet
  if( PktBuf>0 ) { // NO
    // past end point in pen-ultimate buffer?
    if( ( PktBuf>1 ) || ( currLen<=pktLen ) ) { // NO
        pktMode = 2; // Infinite packet mode
    } else { // We're leaving infinite mode
	  PktBuf = 0; // We're past last buffer wrap
    }
  }

  // Only update the chip if something changes
  if( len != PktLen ) cc_write( CC1100_PKTLEN, pktLen );
  if( PktMode!=pktMode ) cc_write( CC1100_PKTCTRL0, pktMode );

  PktLen = len;
  PktMode = pktMode;
}

void cc_check_pktLen( uint16_t current )
{
  // Only need to check if we're handling a long packet
  if( PktBuf > 0 ) {
    if( current+256 > PktLen )
      cc_set_pktLen( PktLen, current );
  }
}

/******************************************************
* It is important to read the fifo with the minimal
* number of SPI cycles because data is accumulating
* while we are reading it.
*
* This is especially the case whenwe process the bytes
* that contain the Evo payload length becaue we don't
* want the last byte of the packet to get into the FIFO
* before we've set the pktLen.
*/
static uint8_t cc_read_fifo(uint8_t *buffer, uint8_t flush )
{ 
  uint8_t nOctet=0, nFifo, status;

  spi_assert();
  while( spi_check_miso() );

  // Read FIFO status
  status = spi_send(CC1100_FIFO|0x40|0x80); // FIFO+read+burst
  nFifo = status & 0x0F;

  // Decide how many octets we're going to remove from the FIFO
  if( !flush )
    nFifo -= 1;
  
  while( nFifo-- )
    buffer[nOctet++] = spi_send(0);

  spi_deassert();

  return nOctet;  
}


static uint16_t frameLen;
static uint16_t rxBytes;

static uint8_t rx_data[64];

static void read_fifo(uint8_t readAll)
{
 uint8_t *data =rx_data;
  uint8_t nOctet = cc_read_fifo( data, readAll );

  while( nOctet-- )
  {
    uint16_t bs_status = bs_accept_octet( *data );
    data++;
    rxBytes++;

    if( bs_status == BS_END_OF_PACKET ) { 
      readAll = 1;
    } else if( bs_status > BS_MAX_STATUS ) { // new packet length
      frameLen = bs_status;
      cc_set_pktLen( frameLen, rxBytes+nOctet );	// Include data we've taken from FIFO but not processed
    }
  }

  cc_check_pktLen( rxBytes );
}

//----------------------------------------------
static void detect_frame_start(void);
static void detect_frame_end(void);
static void detect_fifo_rx_high(void);
static void detect_fifo_tx_low(void);

static void cc_enable_rx(void);
static void cc_start_rx(void);
static void cc_process_rx(void);
static void cc_end_rx(void);

static void cc_enable_tx(void);
static void cc_start_tx(void);
static void cc_process_tx(uint8_t writeAll);
static void cc_end_tx(void);

static uint8_t tx_pending = 0;

//----------------------------------------------
static void cc_enable_rx(void) {
  cc_set_pktLen( 16,0 );	// TODO: get bitstream to tell us starting packet length

  // Configure FIFO interrupt to use RX fifo
  cc_write( CC1100_IOCFG2, 0 );
  cc_write( CC1100_FIFOTHR, 0 );  // 4 bytes in RX FIFO

  detect_fifo_rx_high();
  
  spi_strobe( CC1100_SRX );
}

// Called From Frame interrupt when RX start detected
static void cc_start_rx(void) {
//tty_write_char('{');

  frameLen = 0xFFFF;
  rxBytes = 0;

  bs_accept_octet(0x00);
}

static void cc_process_rx(void) {
//tty_write_char('%');
	
  read_fifo(0);      // Leave at least 1 byte in FIFO (see errata)
  //cc_process_tx(0);  // pull pending TX data
}

static void cc_end_rx(void) {
  read_fifo(1);  // We can empty FIFO now
  bs_accept_octet(0xFF);

//tty_write_char('}');

  if( tx_pending )
    cc_enable_tx();
  else
    cc_enable_rx();
}

//----------------------------------------------

static void cc_enable_tx(void) {
  if( bs_enable_tx() ) {
    // Kick the radio. If it can detect RX activity nothing will happen
    // Don't do anything else until we actually see a TX frame
    spi_strobe( CC1100_STX );
  }
}

static void cc_start_tx(void) {
  // TX Frame detected
  uint16_t pktLen = bs_start_tx();

  // Configure FIFO interrupt to use TX fifo
  // Signal asserts when FIFO is at or above threshhold
  // Signal clears when FIFO drained below threshhold
  cc_write( CC1100_IOCFG2, 2 );
  cc_write( CC1100_FIFOTHR, 7 );    // 32 bytes in TX FIFO

  detect_fifo_tx_low();

  cc_write( CC1100_PKTLEN, pktLen & 0xFF );

  // If all data is in FIFO we can also set fixed packet length
  if( bs_process_tx( 0 ) )
    cc_write( CC1100_PKTCTRL0, 0x00 );

  tx_pending = 0;
}

// Called whenever we have opportunity to update CC1101
static void cc_process_tx( uint8_t writeAll ) {
  // use txSpace to influence how much work we allow BS to do
  uint8_t txSpace = 64 - cc_read( CC1100_TXBYTES );
  if( !writeAll && txSpace > 8 )
    txSpace = 8;

  if( bs_process_tx( txSpace ) ) {
    // Everything is in FIFO 
    if( frame_state == FRAME_TX ) {
      // We never fill the FIFO so we're OK to set fixed packet length
      cc_write( CC1100_PKTCTRL0, 0x00 );
    }
  }
}

static void cc_end_tx(void) {
  cc_enable_rx();

  bs_end_tx();
}

/****************************************************************
* Frame Interrupt
*/
#define INT_MASK  ( FRAME_INT | FIFO_INT )

// Configure Frame interrupt to detect start of frame
static void detect_frame_start(void) {
  EICRA |=  ( 1 << FRAME_INT_ISCn1 ); // Set edge trigger
  EICRA |=  ( 1 << FRAME_INT_ISCn0 ); // Trigger on next rising edge
}

// Configure Frame interrupt to detect end of frame
static void detect_frame_end(void) {
  EICRA |=  ( 1 << FRAME_INT_ISCn1 ); // Set edge trigger
  EICRA &= ~( 1 << FRAME_INT_ISCn0 ); // Trigger on next falling edge
}


ISR(FRAME_INT_VECT) {
FRAME_INT_ENTER
  // Frame interrupt
  uint8_t status = spi_strobe(CC1100_SNOP);

  switch( frame_state )
  {
  case FRAME_IDLE:
    switch( CC_STATE(status) ) {
      case CC_STATE_RX:
        cc_start_rx();
		detect_frame_end();
        frame_state = FRAME_RX;
        break;

      case CC_STATE_TX:
        cc_start_tx();
		detect_frame_end();
        frame_state = FRAME_TX;
        break;
    }
    break;

  case FRAME_RX:  // End of RX Frame
    cc_end_rx();
    detect_frame_start();
    frame_state = FRAME_IDLE;
    break;

  case FRAME_TX:  // End of TX frame;
    cc_end_tx();
    detect_frame_start();
    frame_state = FRAME_IDLE;
    break;
  }
FRAME_INT_LEAVE
}

/****************************************************************
* FIFO Itterrupt
*/
// Signal asserts when FIFO is at or above threshhold
// Signal clears when FIFO drained below threshhold

// Configure FIFO interrupt to detect RX activity
static void detect_fifo_rx_high(void) {
  EICRA |=  ( 1 << FIFO_INT_ISCn1 );   // Set edge trigger
  EICRA |=  ( 1 << FIFO_INT_ISCn0 );   // ... rising edge
}

// Configure FIFO interrupt to detect TX activity
static void detect_fifo_tx_low(void) {
  EICRA |=  ( 1 << FIFO_INT_ISCn1 );   // Set edge trigger
  EICRA &= ~( 1 << FIFO_INT_ISCn0 );   // ... falling edge
}

ISR(FIFO_INT_VECT) {
  // Fifo Interrupt
FIFO_INT_ENTER
  switch( frame_state ) {
  case FRAME_RX:    cc_process_rx();   break;
  case FRAME_TX:    cc_process_tx(1);  break;
  }
FIFO_INT_LEAVE
}

/**************************************************
* TX Software interrupt
*
* All the information about the data to be transmitted 
* is held in bitstream
*
* We must only talk to the CC1101 from within an ISR
* This avoids conflicts between SPI cycles from 
* RX and TX activity that is simultaneous without
* explicitly having to disable/enable interrupts
*
* Whenever bitstream has updated its status it 
* must call cc_tx_trigger
*/

ISR(SW_INT_VECT) {
  switch( frame_state ) {
    case FRAME_IDLE:
      cc_process_tx( 1 );
//      cc_enable_tx();
      break;
    case FRAME_TX:
      cc_process_tx( 1 );
      break;
    case FRAME_RX:
      // Rely on RX activity to process data
      break;
  }
}

static void cc_tx_init(void) {
  SW_INT_DDR  |= SW_INT_PIN;
  SW_INT_MASK |= SW_INT_PIN;
  PCICR |= SW_INT;
}

void cc_tx_trigger(void) {
  SW_INT_PORT |= SW_INT_PIN;
}

uint8_t cc_put_octet( uint8_t octet ) { // Transfer to FIFO
  cc_write( CC1100_FIFO, octet );

  if( frame_state != FRAME_TX )
    tx_pending = 1;

  return 1;
}

static void cc_enter_rx_mode(void) {
  EIMSK &= ~INT_MASK;            // Disable interrupts

  while ( CC_STATE( spi_strobe( CC1100_SIDLE ) ) != CC_STATE_IDLE );
  spi_strobe( CC1100_SFRX );
  while ( CC_STATE( spi_strobe( CC1100_SRX ) ) != CC_STATE_RX );

  frame_state = FRAME_IDLE;

  detect_frame_start();
  cc_enable_rx();

  EIFR  |= INT_MASK;          // Acknowledge any  previous edges
  EIMSK |= INT_MASK;            // Enable interrupts
}

void cc_init(void) {
  spi_init();

  spi_deassert();
  _delay_us(1);

  spi_assert();
  _delay_us(10);

  spi_deassert();
  _delay_us(41);

  spi_strobe(CC1100_SRES);
  spi_strobe(CC1100_SCAL);

  for (uint8_t i = 0; i < sizeof(CC_REGISTER_VALUES); ) {
    uint8_t reg = pgm_read_byte(&CC_REGISTER_VALUES[i++]);
    uint8_t val = pgm_read_byte(&CC_REGISTER_VALUES[i++]);
    cc_write(reg, val);
  }

  // Configure the SYNC WORD required by bitstream 
  uint16_t syncWord = bs_sync_word();
  cc_write( CC1100_SYNC1, ( syncWord>>8 ) & 0xff );
  cc_write( CC1100_SYNC0, ( syncWord    ) & 0xff );
  
  cc_tx_init();     // Initialise Softwre TX  interrupt
  cc_enter_rx_mode();
}

void cc_work(void) {
}
