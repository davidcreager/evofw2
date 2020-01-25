#include <stdio.h>
#include <string.h>

#include "trace.h"

#include "config.h"
#include "errors.h"
#include "tty.h"
//#include "bitstream.h"
//#include "ringbuf.h"
#include "message.h"
#include "transcoder.h"

static write_str_fn write_str;
static send_byte_fn send_byte;

void transcoder_init(write_str_fn w, send_byte_fn s) {
  write_str = w;
  send_byte = s;
}

#if 0
void transcoder_accept_inbound_byte(uint8_t b, uint8_t status) {
  static uint8_t checksum = 0;
  static uint8_t state = S_HEADER;
  static uint8_t multi_bytes = 0;
  static union {
    uint16_t word16;
    uint32_t word32;
  } minibuf;
  static uint8_t header;
  static uint8_t flags;
  static char str[12];

  if (status != 0) {
    if (state != S_COMPLETE || status != ERR_NONE) {
      switch (status) {
        case ERR_BAD_START_BIT:
          write_str("\x09*ERR_BAD_START_BIT*");
          break;
        case ERR_BAD_STOP_BIT:
          write_str("\x09*ERR_BAD_STOP_BIT*");
          break;
        case ERR_UNEXPECTED_START_OF_FRAME:
          write_str("\x09*ERR_UNEXPECTED_START_OF_FRAME*");
          break;
        case ERR_UNEXPECTED_END_OF_FRAME:
          write_str("\x09*ERR_UNEXPECTED_END_OF_FRAME*");
          break;
        case ERR_MANCHESTER_ENCODING:
          write_str("\x09*ERR_MANCHESTER_ENCODING*");
          break;
        default:
          write_str("\x09*ERR_UNKNOWN*");
          break;
      }
    }
    write_str("\r\n");
    state = S_HEADER;
    return;
  }

  if (state == S_COMPLETE) {
    // bytes after end of packet?
    write_str("\x09*E-DATA*");
    state = S_ERROR;
    return;
  }

  if (state == S_ERROR) {
    // ignore bytes while in error state
    return;
  }

  if (state == S_HEADER) {
    checksum = b;
    header = b;
    flags = HEADER_FLAGS[(b >> 2) & 0x0F];
    state = S_ADDR0;
    multi_bytes = 0;
    minibuf.word32 = 0;

    if (is_information(flags)) { write_str("---  I --- "); return; }
    if (is_request(flags))     { write_str("--- RQ --- "); return; }
    if (is_response(flags))    { write_str("--- RP --- "); return; }
    if (is_write(flags))       { write_str("---  W --- "); return; }

    write_str("\x09*HDR*");
    state = S_ERROR;
    return;
  }

  checksum += b;

  if (state == S_ADDR0) {
    if (has_addr0(flags)) {
      minibuf.word32 <<= 8;
      minibuf.word32 |= b;

      multi_bytes++;
      if (multi_bytes == 3) {
        sprintf(str, "%02hu:%06lu ", (uint8_t)(minibuf.word32 >> 18) & 0x3F, minibuf.word32 & 0x3FFFF);
        write_str(str);

        state = S_ADDR1;
        multi_bytes = 0;
        minibuf.word32 = 0;
      }
      return;
    } else {
      write_str("--:------ ");
      state = S_ADDR1;  // and fall through
    }
  }

  if (state == S_ADDR1) {
    if (has_addr1(flags)) {
      minibuf.word32 <<= 8;
      minibuf.word32 |= b;

      multi_bytes++;
      if (multi_bytes == 3) {
        sprintf(str, "%02hu:%06lu ", (uint8_t)(minibuf.word32 >> 18) & 0x3F, minibuf.word32 & 0x3FFFF);
        write_str(str);

        state = S_ADDR2;
        multi_bytes = 0;
        minibuf.word32 = 0;
      }
      return;
    } else {
      write_str("--:------ ");
      state = S_ADDR2;  // and fall through
    }
  }

  if (state == S_ADDR2) {
    if (has_addr2(flags)) {
      minibuf.word32 <<= 8;
      minibuf.word32 |= b;

      multi_bytes++;
      if (multi_bytes == 3) {
        sprintf(str, "%02hu:%06lu ", (uint8_t)(minibuf.word32 >> 18) & 0x3F, minibuf.word32 & 0x3FFFF);
        write_str(str);

        state = S_PARAM0;
        multi_bytes = 0;
        minibuf.word32 = 0;
      }
      return;
    } else {
      write_str("--:------ ");
      state = S_PARAM0;  // and fall through
    }
  }

  if (state == S_PARAM0) {
    if (has_param0(header)) {
      // we don't use params; ditch it and move on
      state = S_PARAM1;
      return;
    } else {
      state = S_PARAM1;  // and fall through
    }
  }

  if (state == S_PARAM1) {
    if (has_param1(header)) {
      // we don't use params; ditch it and move on
      state = S_CMD;
      return;
    } else {
      state = S_CMD;  // and fall through
    }
  }

  if (state == S_CMD) {
    minibuf.word16 <<= 8;
    minibuf.word16 |= b;

    multi_bytes++;
    if (multi_bytes == 2) {
      sprintf(str, "%04X ", minibuf.word16);
      write_str(str);

      state = S_LEN;
      multi_bytes = 0;
      minibuf.word32 = 0;
    }
    return;
  }

  if (state == S_LEN) {
    multi_bytes = b;
    sprintf(str, "%03hu ", b);
    write_str(str);

    state = S_PAYLOAD;
    return;
  }

  if (state == S_PAYLOAD) {
    if (multi_bytes > 0) {
      sprintf(str, "%02hX", b);
      write_str(str);
      multi_bytes--;
    } else {
      state = S_CHECKSUM;  // and fall through
    }
  }

  if (state == S_CHECKSUM) {
    if (checksum != 0) {
      write_str("\x09*CHK*");
      state = S_ERROR;
    } else {
      state = S_COMPLETE;
    }
    return;
  }
}

#if defined(USE_FIFO)

RINGBUF(RX_MSG,128);

uint16_t transcoder_param_len( uint8_t hdr ) {
  uint16_t paramLen = 0;
  uint8_t flags = HEADER_FLAGS[ (hdr >> 2) & 0x0F ];

  if( has_addr0( flags ) ) paramLen += 3;
  if( has_addr1( flags ) ) paramLen += 3;
  if( has_addr2( flags ) ) paramLen += 3;

  if( has_param0( hdr ) ) paramLen += 1;
  if( has_param1( hdr ) ) paramLen += 1;

  return paramLen;
}

void transcoder_rx_byte( uint8_t byte ) {
  if( byte==0xFF ) rb_put( &RX_MSG.rb, 0xFF ); // Add escape char
  rb_put( &RX_MSG.rb, byte );
}

void transcoder_rx_status( uint8_t status ) {
  rb_put( &RX_MSG.rb, 0xFF );  // Add escape char
  rb_put( &RX_MSG.rb, status );
}

static void transcoder_rx_work(void) {
  static uint8_t status=0;

  while( !rb_empty( &RX_MSG.rb )) {
   uint8_t byte = rb_get(&RX_MSG.rb);
//tty_write_char('<');tty_write_hex(byte);tty_write_char('>');
    if( byte==0xFF && status==0 ) {
      status = 1; // Escape value seen
    } else {
      if( status==1 && byte!=0xFF ) {
//        tty_write_char('!');  tty_write_hex( byte);
//        if( byte == TC_RX_END ) tty_write_str("\r\n");
//#if 0
        switch( byte ) {
        case TC_RX_IDLE:
        case TC_RX_END:
          transcoder_accept_inbound_byte(0,ERR_NONE);
          break;

        case TC_RX_MANCHESTER_DECODE_ERROR:
          transcoder_accept_inbound_byte(0,ERR_MANCHESTER_ENCODING);
          break;

        case TC_RX_ABORTED:
          transcoder_accept_inbound_byte(0,ERR_UNEXPECTED_END_OF_FRAME);
          break;
        }
//#endif
      } else {
//        tty_write_hex(byte);
        transcoder_accept_inbound_byte(byte,0);
      }
      status = 0;
    }
  }
}

static void transcoder_tx_work(void) {
}

void transcoder_work(void) {
  transcoder_rx_work();
  transcoder_tx_work();
}
#endif

#define SEND(byte) { bs_send_data(byte); checksum += byte; }

static uint8_t pack_flags(uint8_t flags) {
  for (uint8_t i = 0; i < sizeof(HEADER_FLAGS); i++) {
    if (HEADER_FLAGS[i] == flags) return i << 2;
  }
  return 0xFF;
}
#endif

void transcoder_accept_outbound_byte(uint8_t b) {
#if 0
  static uint8_t flags = 0;
  static uint32_t addrs[3];
  static uint8_t field = 0;
  static uint8_t p = 0;
  static uint8_t checksum = 0;
  static uint16_t msgLen = 0;
  static char buff[12];

//tty_write_char(b);

  if (b == '\n' || b == '\r') {
tty_write_char('\n');
    if (field == 0 && p == 0) return;  // Empty string; most likely CR-LF pair

    if (field == 7) {
      SEND(-checksum);
      bs_send_message( msgLen );
    } else {
//      send_byte(0x11, 1);
    }

    // reset state
    flags = 0;
    field = 0;
    p = 0;
    checksum = 0;
    return;
  }

  if (field == -1) {
    // Something went wrong with the packet. Skip all further bytes until EOL
    return;
  }

  if (field == 7) {
    // Payload comes last; no further separators; buffer character pairs to
    // convert to bytes and write bytes directly as they're ready
    buff[p++] = b;

    if (p == 2) {
      buff[p] = 0; // null terminate
      uint8_t payload_byte;
      sscanf(buff, "%02hhX", &payload_byte);
      SEND(payload_byte);
      p = 0;
    }

    return;
  }

  // Fields before 7, buffer until whitespace
  if (b != ' ') {
    if (p < sizeof(buff) - 1) {
      buff[p++] = b;
    }
    return;
  }

  if (!p) return;  // Double whitespace

  buff[p] = 0;  // null terminate

  if (field == 0) {
    if (!strcmp(buff, "I")) {
      set_information(&flags);
    } else if(!strcmp(buff,"RQ")) {
      set_request(&flags);
    } else if(!strcmp(buff,"RP")) {
      set_response(&flags);
    } else if(!strcmp(buff,"W")) {
      set_write(&flags);
    }
  }

  if (field >= 2 && field <= 4 && buff[0] != '-') {
    uint8_t head;
    uint32_t tail;
    sscanf(buff, "%02hhu:%06lu", &head, &tail);
    if (head == 18 && tail == 730) {  // blank marker from Domoticz
      addrs[field - 2] = 0x48DADA;
    } else {
      addrs[field - 2] = (tail | ((uint32_t)head << 18));
    }
    flags |= (1 << (field - 2));
  }

  if (field == 4) {
    // We have now seen enough to complete the packet header byte, and
    // can start transmitting
    uint8_t header = pack_flags(flags);
    if (header != 0xFF) {
      SEND(header);
      msgLen = 1;
    } else {
      field = -1;
      return;
    }

    if (has_addr0(flags)) {
      SEND((addrs[0] >> 16) & 0xFF);
      SEND((addrs[0] >> 8) & 0xFF);
      SEND(addrs[0] & 0xFF);
      msgLen += 3;
   }

    if (has_addr1(flags)) {
      SEND((addrs[1] >> 16) & 0xFF);
      SEND((addrs[1] >> 8) & 0xFF);
      SEND(addrs[1] & 0xFF);
      msgLen += 3;
    }

    if (has_addr2(flags)) {
      SEND((addrs[2] >> 16) & 0xFF);
      SEND((addrs[2] >> 8) & 0xFF);
      SEND(addrs[2] & 0xFF);
      msgLen += 3;
    }
  }

  if (field == 5) {
    uint16_t cmd;
    sscanf(buff, "%04X", &cmd);
    SEND((cmd >> 8) & 0xFF);
    SEND(cmd & 0xFF);
    msgLen += 2;
  }

  if (field == 6) {
    uint8_t len;
    sscanf(buff, "%03hhu", &len);
    SEND(len);
    msgLen += 1;    // Payload length
    msgLen += len;  // Payload
    msgLen += 1;    // CHecksum
  }

  p = 0;
  field++;
#endif
}

static void tty_write_dec(uint32_t value, uint8_t size) {
  char buf[10];

  buf[size] = '\0';
  while( size-- ) {
    buf[size] = '0' + value%10;
	value /= 10;
  }

  tty_write_str(buf);
}

static char const * const msgType[4] = { "RQ", " I", " W", "RP" };

static uint8_t print_pkt( uint8_t type, uint8_t rcvd ) {
  uint8_t n = 0;
  tty_write_char( ' ' );
  if( rcvd ) {
    tty_write_str( msgType[type] );
    n = 1;
  } else {
    tty_write_str( "??" );
  }
  return n;
}

static uint8_t print_addr( uint8_t *addr, uint8_t valid, uint8_t rcvd ) {
  uint8_t n = 0;
  tty_write_char(' ');
  if( valid ) {
    if( rcvd ) {
      uint32_t dev = ( (uint32_t)addr[0]<<16 ) 
                   + ( (uint32_t)addr[1]<< 8 ) 
                   + ( (uint32_t)addr[2]     );

      tty_write_dec( ( dev >> 18 ) & 0x3f, 2 );
      tty_write_char(':');
      tty_write_dec( ( dev & 0x3FFFF ), 6 );
      n = 3;
    } else {
      tty_write_str("??:??????");
    }
  } else {
    tty_write_str("--:------");
  }
  return n;
}

static uint8_t print_param( uint8_t param, uint8_t valid, uint8_t rcvd ) {
  uint8_t n = 0;
  tty_write_char(' ');
  if( valid ) {
    if( rcvd )
    {
      tty_write_dec( param, 3 );
      n = 1;
    } else {
      tty_write_str("???");
    }      
  } else {
    tty_write_str("---");
  }
  return n;
}

static void print_rssi( uint8_t rssi, uint8_t valid ) {
  print_param( rssi, valid, 1 );
};

static uint8_t print_opcode( uint8_t *opcode, uint8_t rcvd ) {
  uint8_t n = 0;
  tty_write_char(' ');
  if( rcvd ) {
    tty_write_hex(opcode[0]);tty_write_hex(opcode[1]);
    n = 2;
  } else {
    tty_write_str("????");
  }
  return n;
}

static uint8_t print_len( uint8_t len ) {
  uint8_t n = 0;
  n = print_param( len, len>0, 1 );
  return n;
}

static uint8_t print_payload( uint8_t *payload, uint8_t len ) {
  uint8_t n = 0;
  tty_write_char(' ');
  while( len ) {
    tty_write_hex( *payload );
	  payload++;
	  len--;
    n++;
  }
  return n;
}

static void print_error( uint8_t error ) {
  if( error ) {
    tty_write_str(" * ");
    switch(error) {
    case MSG_TIMEOUT:   tty_write_str("Timeout");           break;
    case MSG_SIG_ERR:   tty_write_str("Bad signature");     break;
    case MSG_TYPE_ERR:  tty_write_str("Bad Type");          break;
    case MSG_MANC_ERR:  tty_write_str("Mancherter decode"); break;
    case MSG_CSUM_ERR:  tty_write_str("Checksum");          break;
    case MSG_LEN_ERR:   tty_write_str("Bad Length");        break;
    }   
    tty_write_str(" *");
  }
}

void tc_print_message( struct message *msg ) {
  static uint8_t lastIdx = 0xFF;
  static uint8_t nIdx;
  
  uint8_t flags = msg->flags;
  uint8_t flags2 = msg->flags2;
  uint8_t bytes = msg->bytes;
  
  nIdx = msg->idx - lastIdx;
  if( nIdx != 1 ) {
    tty_write_dec( nIdx, 3 );
    tty_write_str(" missing message(s)\r\n");
  }
  lastIdx = msg->idx;
  
  if( TRACE(TRC_DETAIL) ) {
    tty_write_hex( msg->idx );
    tty_write_char(':');tty_write_hex( msg->flags );
    tty_write_char(':');tty_write_hex( msg->flags2 );
    tty_write_char(':');tty_write_hex( msg->bytes );
    tty_write_char(':');tty_write_hex( msg->error );
    tty_write_str("\r\n");
  }
  
  
  print_rssi( msg->rssi, has_rssi(flags) );
  bytes -= print_pkt( pkt_type( flags ), flags2&F_TYPE );  
  bytes -= print_param( msg->param[0], has_param0( flags ), flags2&F_PARAM0 );
//  bytes -= print_param( msg->param[1], has_param1( flags ), flags2&F_PARAM1 );
  bytes -= print_addr( msg->addr[0], has_addr0( flags ), flags2&F_ADDR0 );
  bytes -= print_addr( msg->addr[1], has_addr1( flags ), flags2&F_ADDR1 );
  bytes -= print_addr( msg->addr[2], has_addr2( flags ), flags2&F_ADDR0 );
  bytes -= print_opcode( msg->opcode, flags2&F_OPCODE );
  bytes -= print_len( msg->len );
  bytes -= print_payload( msg->payload, msg->len );
  
  print_error( msg->error );
  
  tty_write_str("\r\n");
}

static struct message *rxMsgs[8];
static uint8_t rxIn=0;
static uint8_t rxOut=0;

void transcoder_rx_frame( struct message *msg ) {
  rxMsgs[rxIn++] = msg;
  rxIn %= 8;
};

void transcoder_work(void) {
  struct message *rxMsg = NULL;
  if( rxIn != rxOut ) {
    rxMsg = rxMsgs[rxOut++];
    rxOut %= 8;
  }
  
  if( rxMsg ) {
    tc_print_message( rxMsg );
    msg_rx_done( rxMsg );
  }
};
