/***************************************************************************
** message.h
*/
#ifndef _MESSAGE_H_
#define _MESSAGE_H_
#include <stdint.h>

#define FLAG_MASK   0x03
#define FLAG_RQ     0x00
#define FLAG_I		  0x01
#define FLAG_W      0x02
#define FLAG_RP     0x03
inline uint8_t pkt_type(uint8_t flags) { return flags & FLAG_MASK; }
inline void set_request(uint8_t *flags)     { *flags = FLAG_RQ; }
inline void set_information(uint8_t *flags) { *flags = FLAG_I; }
inline void set_write(uint8_t *flags)       { *flags = FLAG_W; }
inline void set_response(uint8_t *flags)    { *flags = FLAG_RP; }

#define FLAG_ADDR0  0x10
#define FLAG_ADDR1  0x20
#define FLAG_ADDR2  0x40
inline uint8_t has_addr0(uint8_t flags){ return flags & FLAG_ADDR0; }
inline uint8_t has_addr1(uint8_t flags){ return flags & FLAG_ADDR1; }
inline uint8_t has_addr2(uint8_t flags){ return flags & FLAG_ADDR2; }
inline void set_addr0(uint8_t *flags){ *flags |= FLAG_ADDR0; }
inline void set_addr1(uint8_t *flags){ *flags |= FLAG_ADDR1; }
inline void set_addr2(uint8_t *flags){ *flags |= FLAG_ADDR2; }

#define FLAG_PARAM0 0x04
#define FLAG_PARAM1 0x08
#define FLAG_RSSI   0x80
inline uint8_t has_param0(uint8_t flags){ return flags & FLAG_PARAM0; }
inline uint8_t has_param1(uint8_t flags){ return flags & FLAG_PARAM1; }
inline uint8_t has_rssi(uint8_t flags)  { return flags & FLAG_RSSI; }
inline void set_param0(uint8_t *flags){ *flags |= FLAG_PARAM0; }
inline void set_param1(uint8_t *flags){ *flags |= FLAG_PARAM1; } 
inline void set_rssi(uint8_t *flags)  { *flags |= FLAG_RSSI; }

#define F_SIG 0x01
#define F_TYPE 0x02
#define F_ADDR0 0x04
#define F_ADDR1 0x08
#define F_ADDR2 0x10
#define F_PARAM0 0x20
#define F_PARAM1 0x40
#define F_OPCODE 0x80

enum msg_error {
  MSG_OK,
  MSG_TIMEOUT,
  MSG_SIG_ERR,
  MSG_TYPE_ERR,
  MSG_MANC_ERR,
  MSG_CSUM_ERR,
  MSG_LEN_ERR
};

#define MAX_PAYLOAD 64
struct message {
  uint8_t idx;
  uint8_t flags;
  uint8_t flags2;
  uint8_t bytes;
  uint8_t error;
  
  uint8_t bsStart;
  uint8_t bsFinish;
  
  uint8_t state;
  uint8_t nBytes;
  uint8_t last;
  
  uint8_t addr[3][3];
  uint8_t param[2];
  
  uint8_t opcode[2];
  uint8_t len;
  
  uint8_t csum;
  uint8_t rssi;
  
  uint8_t payload[MAX_PAYLOAD];
};

extern void msg_init(void);

extern void msg_rx_sof(uint8_t start);
extern void msg_rx_byte(uint8_t byte);
extern void msg_rx_eof(uint8_t finish);
extern void msg_rx_rssi( uint8_t rssi );
extern void msg_rx_timeout(void);

extern void msg_rx_raw(struct message *msg);
extern void msg_rx_done(struct message *msg);

extern uint8_t msg_tx_byte(void);


#endif  // _MESSAGE_H_


