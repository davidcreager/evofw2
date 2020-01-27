/***************************************************************************
** message.c
**
** Message format is  <signature><message bytes><trailer>
** <signature> is a 3 byte sequence <CC><AA><AC>
** <message bytes> are pairs of little-endian Manchester coded bytes <MM><mm>
** <trailer> is a single byte <??>
**
** Once decoded and converted to big-endian values the message format is
** <type><addr0><addr1><addr2><param0><param1><opcode><len><payload><csum>
** where
** <type> is a 1-byte indicating message type and presence of optional fields
** <addr0>, <addr1> and <addr2> are optional 3-byte device addresses
** <param0> and <param1> are optional 1-byte parameters
** <opcode> is a 2-byte (16bit) payload type
** <len> is a 1-byte payload length
** <payload> is a <len> bytes long
** <csum> is a 1-byte checksum covering <type> to <payload> inclusive */
#include <string.h>

#include "trace.h"

#include "tty.h"
#include "bitstream.h"
#include "transcoder.h"

#include "message.h"

/*******************************************************
* Manchester Encoding
*
* The [Evo Message] is encoded in the bitstream with a
* Manchester encoding.  This is the only part of the
* complete packet encoded in this way so we cannot
* use the built in function of the CC1101
*
* While the bitstream is interpreted as a Big-endian stream
* the manchester codes inserted in the stream are little-endian
*
* The Manchester data here is designed to correspond with the
* Big-endian byte stream seen in the bitstream.
*
********
* NOTE *
********
* The manchester decode process converts the data from 
*     2x8 bit little-endian to 8 bit big-endian
* The manchester encode process converts the data from 
*     8 bit big-endian to 2x8 bit little-endian
*
* Since only a small subset of 8-bit values are actually allowed in
* the bitstream rogue values can be used to identify some errors in
* the bitstream.
*
*/

// Convert big-endian 4 bits to little-endian byte 
static uint8_t const man_encode[16] = {
  0x55, 0x95, 0x65, 0xA5, 0x59, 0x99, 0x69, 0xA9,
  0x56, 0x96, 0x66, 0xA6, 0x5A, 0x9A, 0x6A, 0xAA 
};

// Convert little-endian 4 bits to 2-bit big endian
static uint8_t man_decode[16] = {
  0xF, 0xF, 0xF, 0xF, 0xF, 0x0, 0x2, 0xF,
  0xF, 0x1, 0x3, 0xF, 0xF, 0xF, 0xF, 0xF 
};

static inline int manchester_code_valid( uint8_t code ) { 
 return ( man_decode[(code>>4)&0xF]!=0xF ) && ( man_decode[(code   )&0xF]!=0xF ) ; 
}

static inline uint8_t manchester_decode( uint8_t byte1, uint8_t byte2 ) {
  uint8_t decoded;
  
  decoded  = man_decode[( byte1    ) & 0xF ]<<6;
  decoded |= man_decode[( byte1>>4 ) & 0xF ]<<4;
  decoded |= man_decode[( byte2    ) & 0xF ]<<2;
  decoded |= man_decode[( byte2>>4 ) & 0xF ]   ;

  return decoded;
}

static inline void manchester_encode( uint8_t value, uint8_t *byte1, uint8_t *byte2 ) {
  *byte1 = man_encode[ ( value >> 4 ) & 0xF ];
  *byte2 = man_encode[ ( value      ) & 0xF ];
}


static uint8_t const evo_hdr[3] = { 0xCC, 0xAA, 0xCA };
static uint8_t const evo_tlr[1] = { 0xAC };
static uint8_t const train[1] = { 0xAA };

enum message_state {
  S_SIGNATURE,
  S_HEADER,
  S_ADDR0,
  S_ADDR1,
  S_ADDR2,
  S_PARAM0,
  S_PARAM1,
  S_OPCODE,
  S_LEN,
  S_PAYLOAD,
  S_CHECKSUM,
  S_COMPLETE,
  S_ERROR
};

static const uint8_t header_flags[16] = {
  FLAG_RQ + FLAG_ADDR0+FLAG_ADDR1+FLAG_ADDR2 ,
  FLAG_RQ +                       FLAG_ADDR2 ,
  FLAG_RQ + FLAG_ADDR0+           FLAG_ADDR2 ,
  FLAG_RQ + FLAG_ADDR0+FLAG_ADDR1            ,
  FLAG_I  + FLAG_ADDR0+FLAG_ADDR1+FLAG_ADDR2 ,
  FLAG_I  +                       FLAG_ADDR2 ,
  FLAG_I  + FLAG_ADDR0+           FLAG_ADDR2 ,
  FLAG_I  + FLAG_ADDR0+FLAG_ADDR1            ,
  FLAG_W  + FLAG_ADDR0+FLAG_ADDR1+FLAG_ADDR2 ,
  FLAG_W  +                       FLAG_ADDR2 ,
  FLAG_W  + FLAG_ADDR0+           FLAG_ADDR2 ,
  FLAG_W  + FLAG_ADDR0+FLAG_ADDR1            ,
  FLAG_RP + FLAG_ADDR0+FLAG_ADDR1+FLAG_ADDR2 ,
  FLAG_RP +                       FLAG_ADDR2 ,
  FLAG_RP + FLAG_ADDR0+           FLAG_ADDR2 ,
  FLAG_RP + FLAG_ADDR0+FLAG_ADDR1            ,
};

static uint8_t get_hdr_flags(uint8_t header ) {
  uint8_t flags = 0;
  uint8_t lookup = ( header>>2 ) & 0x0F;
  if( header!=0xFF && lookup < sizeof(header_flags) )
    flags = header_flags[lookup];

  if( TRACE(TRC_DETAIL) ) {
    tty_write_hex(header);
    tty_write_hex(flags);
  }
    
  return header_flags[ ( header>>2 ) & 0x0F ];
}

static uint8_t get_header( uint8_t flags ) {
  uint8_t i;

  for( i=0 ; i<sizeof(header_flags) ; i++ ) {
    if( flags==header_flags[i] )
	  break;
  }

  return i<<2;	// Will return 0x40 if not found
}
  
#define HDR_PARAM0 0x02
#define HDR_PARAM1 0x01
inline uint8_t hdr_param0(uint8_t header)    { return header & HDR_PARAM0; }
inline uint8_t hdr_param1(uint8_t header)    { return header & HDR_PARAM1; }

static uint8_t header_length(struct message *msg) {
  uint8_t len = 0;
  uint16_t flags = msg->flags;
  
  len = 1;	// packet type;
  if( has_addr0(flags)  ) len += sizeof(msg->addr[0]);
  if( has_addr1(flags)  ) len += sizeof(msg->addr[1]);
  if( has_addr2(flags)  ) len += sizeof(msg->addr[2]);
  if( has_param0(flags) ) len += sizeof(msg->param[0]);
  if( has_param1(flags) ) len += sizeof(msg->param[1]);
  len += sizeof(msg->opcode);
  len += sizeof(msg->len);
  
  return len;
}

static void msg_rx_reset(struct message *msg) { memset( msg, 0, sizeof(*msg) ); }

static struct message RX[2];

static struct message *msgList[4];
static uint8_t msgIn=0;
static uint8_t msgOut=0;

static struct message *rx = NULL;
uint8_t rxIdx = 0;

void msg_rx_raw( struct message *msg ) {
  bs_rx_raw( msg->bsStart,msg->bsFinish);
}

void msg_rx_done( struct message *msg ) {
  msgList[msgIn++] = msg;
  msgIn %= 4;
}

void msg_rx_sof(uint8_t start) {
  struct message *msg = NULL;
  if( msgIn != msgOut ) {
    msg = msgList[msgOut++];
    msgOut %= 4;
  }
  if( msg ) {
   rx = msg;
   msg_rx_reset(rx);
   rx->idx = rxIdx;
   rx->bsStart = start;
  } else {
    if( TRACE(TRACE_MSG) )
      tty_write_char('&');
  }
  rxIdx++;
}

void msg_rx_rssi( uint8_t rssi ) {
  if( rx ) {
    rx->rssi = rssi;
    set_rssi( &rx->flags );
  }else {
    if( TRACE(TRACE_MSG) )
      tty_write_char(':');
  }
}

void msg_rx_timeout(void) {
  if( rx ) {
    rx->state = S_ERROR;
    rx->error = MSG_TIMEOUT;
  }
}

void msg_rx_eof(uint8_t finish) {
  if( rx ) {
    rx->bsFinish = finish;
    if( (rx->state != S_ERROR) || TRACE(TRC_ERROR) )
      transcoder_rx_frame( rx );
    else
      msg_rx_done( rx );
    rx = NULL;
  }
}

void msg_rx_byte(uint8_t byte) {
  uint8_t decoded,temp;

  if( !rx )
    return;
  
  // Discard everything after error detected - wait for EOF
  if( rx->state == S_ERROR ){
    if( TRACE(TRACE_MSG) )
      tty_write_char('!');
    return;
  }

  rx->bytes++;
  
  // Validate signature - not manchester encoded
  if( rx->state==S_SIGNATURE ) { // signature bytes
    if( byte != evo_hdr[rx->nBytes++] ) {
	    rx->state = S_ERROR;
      rx->error = MSG_SIG_ERR;
      if( TRACE(TRACE_MSG) )
        tty_write_char('S');
      return;
    } 
    if( rx->nBytes==sizeof(evo_hdr) ) {
	    rx->nBytes = 0;
      rx->flags2 |= F_SIG;
  		rx->state = S_HEADER;
    }
    return;
  }
  
  // Validate trailer - not manchester encoded
  if( rx->state==S_COMPLETE ) {
    if( rx->nBytes < sizeof( evo_tlr ) ) { // Silently discard trailing bytes  
      if( byte != evo_tlr[rx->nBytes++] ) {
        rx->error = MSG_LEN_ERR;
        // silently ignore for now
      }
    }
    return;
	}
  
  if( !manchester_code_valid(byte) ) {
    rx->state = S_ERROR;
    rx->error = MSG_MANC_ERR;
    if( TRACE(TRACE_MSG) )
      tty_write_char('?');
    return;
  }

  // Is this first part of manchester pair?
  if( !( rx->last ) ) {
    rx->last = byte;
    return;
  }
  
  decoded = manchester_decode( rx->last, byte );
  rx->last = 0;
  rx->csum += decoded;

  switch( rx->state ) {
  case S_HEADER:
    temp = get_hdr_flags(decoded);
    if( temp==0 ) {
      rx->state = S_ERROR;
      rx->error = MSG_TYPE_ERR;
      return;
    }

    rx->flags |= temp;
    if( hdr_param0(decoded) )set_param0( &rx->flags );
    if( hdr_param1(decoded) )set_param1( &rx->flags );

	  // Update the message length to at least len
	  bs_rx_msgLen( sizeof(evo_hdr) + 2*header_length(rx) + 3 /* force extra fifo frame */ );

    rx->flags2 |= F_TYPE;
    rx->state = S_ADDR0;
	  break;
	
  case S_ADDR0:
    if( has_addr0(rx->flags) ) {
	    rx->addr[0][rx->nBytes++] = decoded;
	    if( rx->nBytes==3 ) {
        rx->flags2 |= F_ADDR0;
        rx->state = S_ADDR1;
        rx->nBytes=0;
	    }
	    break;
	  }
	  rx->state = S_ADDR1;
  	// Fall through..

  case S_ADDR1:
    if( has_addr1(rx->flags) ) {
	    rx->addr[1][rx->nBytes++] = decoded;
	    if( rx->nBytes==3 ) {
        rx->flags2 |= F_ADDR1;
        rx->state = S_ADDR2;
        rx->nBytes=0;
	    }
	    break;
	  }
	  rx->state = S_ADDR2;
	  // Fall through..

  case S_ADDR2:
    if( has_addr2(rx->flags) ) {
	    rx->addr[2][rx->nBytes++] = decoded;
	    if( rx->nBytes==3 ) {
        rx->flags2 |= F_ADDR2;
        rx->state = S_PARAM0;
        rx->nBytes=0;
	    }
	    break;
	  }
	  rx->state = S_PARAM0;
	  // Fall through..

  case S_PARAM0:
    if( has_param0(rx->flags) ) {
      rx->param[0] = decoded;
      rx->flags2 |= F_PARAM0;
      rx->state = S_PARAM1;
      break;
	  }
	  rx->state = S_PARAM1;
	  // Fall through..

  case S_PARAM1:
    if( has_param1(rx->flags) ) {
      rx->param[1] = decoded;
      rx->flags2 |= F_PARAM1;
      rx->state = S_OPCODE;
      break;
	  }
	  rx->state = S_OPCODE;
	  // Fall through..

  case S_OPCODE:
    rx->opcode[rx->nBytes++] = decoded;
	  if( rx->nBytes==2 ) {
      rx->flags2 |= F_OPCODE;
      rx->state = S_LEN;
      rx->nBytes=0;
	  }
	  break;

  case S_LEN:
    temp = rx->len = decoded;
    if( rx->len > MAX_PAYLOAD )
      temp = MAX_PAYLOAD;
      
	  // Update the message length to include payload, csum and trailer
	  bs_rx_msgLen( sizeof(evo_hdr) + 2*( header_length(rx) + temp + sizeof(rx->csum) ) + sizeof(evo_tlr) );
    
	  rx->state = S_PAYLOAD;
    break;
	
  case S_PAYLOAD:
    rx->payload[rx->nBytes++] = decoded;
	  if( (rx->nBytes==rx->len) || (rx->nBytes==MAX_PAYLOAD) ) {
      rx->state = S_CHECKSUM;
      rx->nBytes=0;
	  }
	  break;
  
  case S_CHECKSUM:
    if( rx->csum != 0 ) {
	    rx->state = S_ERROR;
      rx->error = MSG_CSUM_ERR;
      if( TRACE(TRACE_MSG) )
        tty_write_char('=');
	  } else {
	    rx->state = S_COMPLETE;
	  }
	  break;
  }
}

void msg_init(void) {
  msg_rx_done(&RX[0]);
  msg_rx_done(&RX[1]);
}