#ifndef _CONFIG_H_
#  error "Include config.h instead of this file"
#endif

#ifndef _HW_ARDUINO_H_
#define _HW_ARDUINO_H_

#include <avr/io.h>

#define RINGBUF_SIZE    128

// SPI port defs
#define SPI_PORT    PORTB
#define SPI_DDR     DDRB
#define SPI_SS      2
#define SPI_MOSI    3
#define SPI_MISO    4
#define SPI_SCLK    5
#define SPI_CLK_RATE 2000000

// Frame detect interrupt
#define FRAME_INT        ( 1 << INT0 )
#define FRAME_INT_VECT   INT0_vect
#define FRAME_INT_ISCn0  ISC00
#define FRAME_INT_ISCn1  ISC01

// FIFO level interrupt
#define FIFO_INT         ( 1 << INT1 )
#define FIFO_INT_VECT    INT1_vect
#define FIFO_INT_ISCn0   ISC10
#define FIFO_INT_ISCn1   ISC11

// Software interrupt
#define SW_INT           ( 1<<PCIE2 )
#define SW_INT_VECT      PCINT2_vect
#define SW_INT_MASK      PCMSK2
#define SW_INT_PORT      PIND
#define SW_INT_DDR       DDRD
#define SW_INT_PIN       ( 1<<7 )

// SOme debug pins
#define DEBUG_PORT        PORTD
#define DEBUG_DDR         DDRD
#define DEBUG_PIN1        ( 1<<4 )
#define DEBUG_PIN2        ( 1<<5 )
#define DEBUG_PIN3        ( 1<<6 )

// TTY USART
#define TTY_UDRE_VECT   USART_UDRE_vect
#define TTY_RX_VECT     USART_RX_vect
#define TTY_BAUD_RATE   115200

// LED
#define LED_DDR   DDRB
#define LED_PORT  PORTB
#define LED_PIN   5

#endif
