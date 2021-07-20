/*****************************************************************************
Title:     STK500v2 compatible bootloader
Author:    Peter Fleury  <pfleury@gmx.ch> http://tinyurl.com/peterfleury
File:      $Id: stk500boot.c,v 1.20 2016/07/24 11:17:12 peter Exp $
Compiler:  avr-gcc 4.8.1 / avr-libc 1.8
Hardware:  All AVRs with bootloader support, tested with ATmega8
License:   GNU General Public License Version 3

Modifications:
Author:    Basil Hussain <basil [at] stasisleak [dot] uk>
Date:      2019-02-22
Hardware:  Additionally supports ATmegaXXM1 series
Changes:   - Added support for the LIN-based UART on the ATmegaXXM1 chips.
           - LED indicator pin now has configurable polarity (active-low or
             active-high).
           - Device signature string is configurable to be either "AVRISP_2" or
             "STK500_2"; latter is default for Atmel Studio 7 compatibility.
           - Changed status response to unhandled commands to 'unknown' rather
             than 'failed'.
           - Added configurable handling for responses to VTARGET and VADJUST
             parameter queries. Can optionally be disabled.
           - Fixed bug with flash page erasure when writing; address of page
             being erased was being maintained separately and could in certain
             circumstances get out of sync.
           - Corrected command sequence number behaviour. Should just echo back
             the given number in answer.
           - Added proper handling of commands received with invalid checksum;
             will now respond with ANSWER_CKSUM_ERROR in that case.
           - Added option to use avr-libc library functions for EEPROM reading
             and writing (should be more general-purpose).
           - Changed code that reads device signature to actually read values
             from chip, rather than respond with compiled-in constants.
           - Added support for oscillator calibration read command.
           - Implemented chip erase command; erases entire application flash and
             also EEPROM if EESAVE fuse is not set. Can optionally be disabled.
           - Modified makefile to have MCU, F_CPU and BOOTLOADER_ADDRESS
             variables be overridable by command-line arguments.
           - Corrected various spelling mistakes in comments.

DESCRIPTION:
    This program allows an AVR with bootloader capabilities to
    read/write its own Flash/EEprom. To enter Programming mode
    an input pin is checked. If this pin is pulled low, programming mode
    is entered. If not, normal execution is done from $0000
    "reset" vector in Application area.
    Size < 500 words, fits into a 512 word bootloader section


USAGE:
    - Set AVR MCU type and clock-frequency (F_CPU) in the Makefile.
    - Set bootloader start address in bytes (BOOTLOADER_ADDRESS) in Makefile
      this must match selected "Boot Flash section size" fuses below
    - Set baud rate below (AVRISP only works with 115200 bps)
    - compile/link the bootloader with the supplied Makefile
    - program the "Boot Flash section size" (BOOTSZ fuses),
      for boot-size 512 words:  program BOOTSZ1
    - enable the BOOT Reset Vector (program BOOTRST)
    - Upload the hex file to the AVR using any ISP programmer
    - Program Boot Lock Mode 3 (program BootLock 11 and BootLock 12 lock bits)
    - Reset your AVR while keeping PROG_PIN pulled low
    - Start AVRISP Programmer (AVRStudio/Tools/Program AVR)
    - AVRISP will detect the bootloader
    - Program your application FLASH file and optional EEPROM file using AVRISP

Note:
    Erasing the device without flashing, through AVRISP GUI button "Erase Device"
    is not implemented, due to AVRStudio limitations.
    Flash is always erased before programming.

    Normally the bootloader accepts further commands after programming.
    The bootloader exits and starts application code after programming
    when ENABLE_LEAVE_BOOTLADER is defined.
    Use Auto Programming mode to program both flash and EEPROM,
    otherwise bootloader will exit after flash programming.

    AVRdude:
    Please uncomment #define REMOVE_CMD_SPI_MULTI when using AVRdude.
    Uncomment #define REMOVE_PROGRAM_LOCK_BIT_SUPPORT and
    #define REMOVE_READ_LOCK_FUSE_BIT_SUPPORT to reduce code size.
    Read Fuse Bits and Read/Write Lock Bits is not supported when using AVRdude.

NOTES:
    Based on Atmel Application Note AVR109 - Self-programming
    Based on Atmel Application Note AVR068 - STK500v2 Protocol

LICENSE:
    Copyright (C) 2006 Peter Fleury

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*****************************************************************************/
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include "command.h"


/*
 * Uncomment the following lines to save code space
 */
#define REMOVE_PROGRAM_LOCK_BIT_SUPPORT    // disable program lock bits
//#define REMOVE_READ_LOCK_FUSE_BIT_SUPPORT  // disable reading lock and fuse bits
//#define REMOVE_BOOTLOADER_LED              // no LED to show active bootloader
//#define REMOVE_PROG_PIN_PULLUP             // disable internal pull-up, use external pull-up resistor
#define REMOVE_CMD_SPI_MULTI               // disable processing of SPI_MULTI commands (SPI MULTI command only used by AVRdude)
//#define REMOVE_VOLTAGE_PARAMS              // disable responses to queries for VTARGET and VADJUST parameters
#define REMOVE_CMD_CHIP_ERASE_ISP          // disable chip erase, making that command a no-op

/*
 *  Uncomment to leave bootloader and jump to application after programming.
 */
//#define ENABLE_LEAVE_BOOTLADER

/*
 * Uncomment to use avr-libc library functions for reading/writing EEPROM.
 */
#define USE_LIBC_EEPROM_FUNCS

/*
 * Pin "PROG_PIN" on port "PROG_PORT" has to be pulled low
 * (active low) to start the bootloader
 * uncomment #define REMOVE_PROG_PIN_PULLUP if using an external pullup
 */
#define PROG_PORT  PORTC
#define PROG_DDR   DDRC
#define PROG_IN    PINC
#define PROG_PIN   PINC7

/*
 * LED on pin "PROGLED_PIN" on port "PROGLED_PORT" indicates that bootloader is
 * active. Set "PROGLED_POLARITY" to 1 or 0 to indicate whether pin is active-
 * high or active-low, respectively.
 */
#define PROGLED_POLARITY 1
#define PROGLED_PORT PORTD
#define PROGLED_DDR  DDRD
#define PROGLED_PIN  PIND7


/*
 * define which UART channel will be used, if device with two UARTs is used
 */
#define USE_USART0        // use first USART
//#define USE_USART1      // use second USART

/*
 * UART Baud rate, AVRStudio AVRISP only accepts 115200 bps
 */
#define BAUDRATE 115200


/*
 *  Enable (1) or disable (0) USART double speed operation
 */
#define UART_BAUDRATE_DOUBLE_SPEED 0


/*
 * The signature string the bootloader responds with to the 'sign on' command.
 * Change "CONFIG_SIGN_ON_SIG" to be one of the two defined strings. String
 * must be exactly 8 characters.
 */
#define CONFIG_SIGN_ON_SIG_AVRISP "AVRISP_2"
#define CONFIG_SIGN_ON_SIG_STK500 "STK500_2"
#define CONFIG_SIGN_ON_SIG CONFIG_SIGN_ON_SIG_STK500


/*
 * HW and SW version, reported to AVRISP, must match version of AVRStudio
 */
#define CONFIG_PARAM_BUILD_NUMBER_LOW   0
#define CONFIG_PARAM_BUILD_NUMBER_HIGH  0
#define CONFIG_PARAM_HW_VER             0x0F
#define CONFIG_PARAM_SW_MAJOR           2
#define CONFIG_PARAM_SW_MINOR           0x0A

/*
 * The voltage value that is given in response to queries for VTARGET and
 * VADJUST parameters (if enabled). Value is volts x10 in decimal - for example:
 * 5.0V => 50, 3.3V => 33.
 */
#define CONFIG_PARAM_VOLTAGE 50


/*
 * Calculate the address where the application section ends from FLASHEND and from start address of bootloader
 * The address where the bootloader starts is passed by Makefile using -DBOOTLOADER_ADDRESS
 */
#define APP_END (BOOTLOADER_ADDRESS - 1)


/*
 *  Defines for the various USART registers
 */
#ifdef USE_USART0

#ifdef LINDAT

#ifdef LINBRRL
#define UART_BAUD_RATE_LOW       LINBRRL
#endif
#ifdef LINBRRH
#define UART_BAUD_RATE_HIGH      LINBRRH
#endif
#ifdef LINBTR
#define UART_BIT_TIMING_REG      LINBTR
#endif
#ifdef LINSIR
#define UART_STATUS_REG          LINSIR
#endif
#ifdef LINCR
#define UART_CONTROL_REG         LINCR
#endif
#if defined(LSWRES)
#define UART_RESET               (1 << LSWRES)
#endif
#if defined(LENA) && defined(LCMD0) && defined(LCMD1) && defined(LCMD2)
#define UART_ENABLE_TRANSMITTER  ((1 << LENA) | (1 << LCMD2) | (1 << LCMD0))
#define UART_ENABLE_RECEIVER     ((1 << LENA) | (1 << LCMD2) | (1 << LCMD1))
#endif
#if defined(LTXOK) && defined(LRXOK)
#define UART_TRANSMIT_COMPLETE   (1 << LTXOK)
#define UART_RECEIVE_COMPLETE    (1 << LRXOK)
#endif
#ifdef LINDAT
#define UART_DATA_REG            LINDAT
#endif
#define UART_DOUBLE_SPEED        0

#else

#ifdef UBRRL
#define UART_BAUD_RATE_LOW       UBRRL
#else
#ifdef UBRR0L
#define UART_BAUD_RATE_LOW       UBRR0L
#endif
#endif

#ifdef UBRR0H
#define UART_BAUD_RATE_HIGH      UBRR0H
#endif

#ifdef UCSRA
#define UART_STATUS_REG          UCSRA
#else
#ifdef UCSR0A
#define UART_STATUS_REG          UCSR0A
#endif
#endif

#ifdef UCSRB
#define UART_CONTROL_REG         UCSRB
#else
#ifdef UCSR0B
#define UART_CONTROL_REG         UCSR0B
#endif
#endif

#ifdef TXEN
#define UART_ENABLE_TRANSMITTER  (1 << TXEN)
#else
#ifdef TXEN0
#define UART_ENABLE_TRANSMITTER  (1 << TXEN0)
#endif
#endif

#ifdef RXEN
#define UART_ENABLE_RECEIVER     (1 << RXEN)
#else
#ifdef RXEN0
#define UART_ENABLE_RECEIVER     (1 << RXEN0)
#endif
#endif

#ifdef TXC
#define UART_TRANSMIT_COMPLETE   (1 << TXC)
#else
#ifdef TXC0
#define UART_TRANSMIT_COMPLETE   (1 << TXC0)
#endif
#endif

#ifdef RXC
#define UART_RECEIVE_COMPLETE    (1 << RXC)
#else
#ifdef RXC0
#define UART_RECEIVE_COMPLETE    (1 << RXC0)
#endif
#endif

#ifdef UDR
#define UART_DATA_REG            UDR
#else
#ifdef UDR0
#define UART_DATA_REG            UDR0
#endif
#endif

#ifdef U2X
#define UART_DOUBLE_SPEED        (1 << U2X)
#else
#ifdef U2X0
#define UART_DOUBLE_SPEED        (1 << U2X0)
#endif
#endif

#endif

/* ATMega with two USART, select second USART for bootloader using USE_USART1 define */
#else
#ifdef USE_USART1

#ifdef UBRR1L
#define UART_BAUD_RATE_LOW       UBRR1L
#endif

#ifdef UBRR1H
#define UART_BAUD_RATE_HIGH      UBRR1H
#endif

#ifdef UCSR1A
#define UART_STATUS_REG          UCSR1A
#endif

#ifdef UCSR1B
#define UART_CONTROL_REG         UCSR1B
#endif

#ifdef TXEN1
#define UART_ENABLE_TRANSMITTER  (1 << TXEN1)
#endif

#ifdef RXEN1
#define UART_ENABLE_RECEIVER     (1 << RXEN1)
#endif

#ifdef TXC1
#define UART_TRANSMIT_COMPLETE   (1 << TXC1)
#endif

#ifdef RXC1
#define UART_RECEIVE_COMPLETE    (1 << RXC1)
#endif

#ifdef UDR1
#define UART_DATA_REG            UDR1
#endif

#ifdef U2X1
#define UART_DOUBLE_SPEED        (1 << U2X1)
#endif

#else
#error "USE_USART0 / USE_USART1 undefined  !"
#endif
#endif

#if defined(UART_BAUD_RATE_LOW) && defined(UART_STATUS_REG) && defined(UART_CONTROL_REG) && defined(UART_ENABLE_TRANSMITTER) && defined(UART_ENABLE_RECEIVER) && defined(UART_TRANSMIT_COMPLETE) && defined(UART_RECEIVE_COMPLETE) && defined(UART_DATA_REG) && defined(UART_DOUBLE_SPEED)
#else
#error "no UART definition for MCU available"
#endif


/*
 * Macros to map the new ATmega88/168 EEPROM bits
 */
#ifdef EEMPE
#define EEMWE EEMPE
#define EEWE  EEPE
#endif


/*
 * Macro to calculate UBBR from XTAL and baud rate
 */
#ifdef LINDAT

// Just hard-code for now for 115,200 baud and F_CPU of 16 MHz. Values give baud
// rate within 1%.
#define UART_BAUD_SELECT(baud, cpu_freq) 5
#define UART_BIT_TIMING_SELECT() ((1 << LDISR) | (23 << LBT0))
#warning "Using hard-coded UART baud and bit-timing values, assumes F_CPU = 16 MHz"

/*
#if UART_BAUDRATE_DOUBLE_SPEED
#define UART_BAUD_SELECT(baud, cpu_freq) (((cpu_freq) + 4UL * (baud)) / (8UL * (baud)) - 1UL)
#define UART_BIT_TIMING_SELECT() ((1 << LDISR) | (8 << LBT0))
#else
#define UART_BAUD_SELECT(baud, cpu_freq) (((cpu_freq) + 8UL * (baud)) / (16UL * (baud)) - 1UL)
#define UART_BIT_TIMING_SELECT() ((1 << LDISR) | (16 << LBT0))
#endif
*/

#else

#if UART_BAUDRATE_DOUBLE_SPEED
#define UART_BAUD_SELECT(baudRate,xtalCpu) (((float)(xtalCpu))/(((float)(baudRate))*8.0)-1.0+0.5)
#else
#define UART_BAUD_SELECT(baudRate,xtalCpu) (((float)(xtalCpu))/(((float)(baudRate))*16.0)-1.0+0.5)
#endif

#endif

/*
 * States used in the receive state machine
 */
#define ST_START        0
#define ST_GET_SEQ_NUM  1
#define ST_MSG_SIZE_1   2
#define ST_MSG_SIZE_2   3
#define ST_GET_TOKEN    4
#define ST_GET_DATA     5
#define ST_GET_CHECK    6
#define ST_PROCESS      7


/*
 * use 16bit address variable for ATmegas with <= 64K flash
 */
#if defined(RAMPZ)
typedef uint32_t address_t;
#else
typedef uint16_t address_t;
#endif


/*
 * function prototypes
 */
static void sendchar(const char c);
static unsigned char recchar(void);


/*
 * since this bootloader is not linked against the avr-gcc crt1 functions,
 * to reduce the code size, we need to provide our own initialization
 */
void __jumpMain     (void) __attribute__ ((naked)) __attribute__ ((section (".init9")));

void __jumpMain(void)
{
    asm volatile ( ".set __stack, %0" :: "i" (RAMEND) );       // init stack
    asm volatile ( "clr __zero_reg__" );                       // GCC depends on register r1 set to 0
    asm volatile ( "out %0, __zero_reg__" :: "I" (_SFR_IO_ADDR(SREG)) );  // set SREG to 0
#ifndef REMOVE_PROG_PIN_PULLUP
    PROG_PORT |= (1<<PROG_PIN);                                // Enable internal pullup
#endif
    asm volatile ( "rjmp main");                               // jump to main()
}


/*
 * send single byte to USART, wait until transmission is completed
 */
static void sendchar(const char c)
{
    UART_DATA_REG = c;                                         // prepare transmission
    while (!(UART_STATUS_REG & UART_TRANSMIT_COMPLETE));       // wait until byte sent
    UART_STATUS_REG |= UART_TRANSMIT_COMPLETE;                 // delete TXCflag
}

/*
 * Read single byte from USART, block if no data available
 */
static unsigned char recchar(void)
{
    while(!(UART_STATUS_REG & UART_RECEIVE_COMPLETE));  // wait for data
    return UART_DATA_REG;
}


int main(void) __attribute__ ((OS_main));
int main(void)
{
    address_t       address = 0;
    unsigned char   msgParseState;
    unsigned int    i = 0;
    unsigned char   checksum = 0;
    unsigned char   seqNum = 0;
    unsigned int    msgLength = 0;
    unsigned char   msgBuffer[285];
    unsigned char   c, *p;
    unsigned char   isLeave = 0;


    /*
     * Branch to bootloader or application code ?
     */
    if(!(PROG_IN & (1<<PROG_PIN)))
    {
#ifndef REMOVE_BOOTLOADER_LED
        /* PROG_PIN pulled low, indicate with LED that bootloader is active */
        PROGLED_DDR  |= (1<<PROGLED_PIN);
#if PROGLED_POLARITY
		PROGLED_PORT |= (1<<PROGLED_PIN);
#else
        PROGLED_PORT &= ~(1<<PROGLED_PIN);
#endif
#endif
        /*
         * Init UART
         * Begin by resetting UART peripheral if specified.
         * Set baud rate and enable USART receiver and transmitter without interrupts
         * Also set bit timing if necessary.
         */
#ifdef UART_RESET
        UART_CONTROL_REG = UART_RESET;
#endif
#if UART_BAUDRATE_DOUBLE_SPEED
        UART_STATUS_REG   |=  UART_DOUBLE_SPEED;
#endif

#ifdef UART_BAUD_RATE_HIGH
        UART_BAUD_RATE_HIGH = 0;
#endif
        UART_BAUD_RATE_LOW = UART_BAUD_SELECT(BAUDRATE,F_CPU);
#if defined(UART_BIT_TIMING_REG) && defined(UART_BIT_TIMING_SELECT)
        UART_BIT_TIMING_REG = UART_BIT_TIMING_SELECT();
#endif
        UART_CONTROL_REG   = UART_ENABLE_RECEIVER | UART_ENABLE_TRANSMITTER;


        /* main loop */
        while(!isLeave)
        {
            /*
             * Collect received bytes to a complete message
             */
            msgParseState = ST_START;
            while ( msgParseState != ST_PROCESS )
            {
                c = recchar();
                switch (msgParseState)
                {
                case ST_START:
                    if( c == MESSAGE_START )
                    {
                        msgParseState = ST_GET_SEQ_NUM;
                        checksum = MESSAGE_START^0;
                    }
                    break;

                case ST_GET_SEQ_NUM:
                    seqNum = c;
                    msgParseState = ST_MSG_SIZE_1;
                    checksum ^= c;
                    break;

                case ST_MSG_SIZE_1:
                    msgLength = (unsigned int)c<<8;
                    msgParseState = ST_MSG_SIZE_2;
                    checksum ^= c;
                    break;

                case ST_MSG_SIZE_2:
                    msgLength |= c;
                    msgParseState = ST_GET_TOKEN;
                    checksum ^= c;
                    break;

                case ST_GET_TOKEN:
                    if ( c == TOKEN )
                    {
                        msgParseState = ST_GET_DATA;
                        checksum ^= c;
                        i = 0;
                    }
                    else
                    {
                        msgParseState = ST_START;
                    }
                    break;

                case ST_GET_DATA:
                    msgBuffer[i++] = c;
                    checksum ^= c;
                    if ( i == msgLength )
                    {
                        msgParseState = ST_GET_CHECK;
                    }
                    break;

                case ST_GET_CHECK:
                    if( c != checksum ) {
                        msgBuffer[0] = ANSWER_CKSUM_ERROR;
                    }
                    msgParseState = ST_PROCESS;
                    /*
                    if( c == checksum )
                    {
                        msgParseState = ST_PROCESS;
                    }
                    else
                    {
                        msgParseState = ST_START;
                    }
                    */
                    break;
                }//switch
            }//while(msgParseState)

            /*
             * Now process the STK500 commands, see Atmel Appnote AVR068
             */

            switch (msgBuffer[0])
            {
#ifndef REMOVE_CMD_SPI_MULTI
            case CMD_SPI_MULTI:
                {
                    unsigned char answerByte = 0;

                    // only Read Signature Bytes implemented, return dummy value for other instructions
                    if ( msgBuffer[4]== 0x30 )
                    {
                        answerByte = boot_signature_byte_get(msgBuffer[6] << 1);
                    }
                    msgLength = 7;
                    msgBuffer[1] = STATUS_CMD_OK;
                    msgBuffer[2] = 0;
                    msgBuffer[3] = msgBuffer[4];  // Instruction Byte 1
                    msgBuffer[4] = msgBuffer[5];  // Instruction Byte 2
                    msgBuffer[5] = answerByte;
                    msgBuffer[6] = STATUS_CMD_OK;
                }
                break;
#endif
            case CMD_SIGN_ON:
                msgLength = 11;
                msgBuffer[1]  = STATUS_CMD_OK;
                msgBuffer[2]  = 8;
                memcpy_P(&msgBuffer[3], PSTR(CONFIG_SIGN_ON_SIG), 8);
                break;

            case CMD_GET_PARAMETER:
                switch(msgBuffer[1])
                {
                case PARAM_BUILD_NUMBER_LOW:
                    msgBuffer[2] = CONFIG_PARAM_BUILD_NUMBER_LOW;
                    break;
                case PARAM_BUILD_NUMBER_HIGH:
                    msgBuffer[2] = CONFIG_PARAM_BUILD_NUMBER_HIGH;
                    break;
                case PARAM_HW_VER:
                    msgBuffer[2] = CONFIG_PARAM_HW_VER;
                    break;
                case PARAM_SW_MAJOR:
                    msgBuffer[2] = CONFIG_PARAM_SW_MAJOR;
                    break;
                case PARAM_SW_MINOR:
                    msgBuffer[2] = CONFIG_PARAM_SW_MINOR;
                    break;
#ifndef REMOVE_VOLTAGE_PARAMS
                case PARAM_VTARGET:
                case PARAM_VADJUST:
                    msgBuffer[2] = CONFIG_PARAM_VOLTAGE;
                    break;
#endif
                default:
                    msgBuffer[2] = 0;
                    break;
                }
                msgLength = 3;
                msgBuffer[1] = STATUS_CMD_OK;
                break;

            case CMD_LEAVE_PROGMODE_ISP:
#ifdef ENABLE_LEAVE_BOOTLADER
                isLeave = 1;
#endif
            case CMD_ENTER_PROGMODE_ISP:
            case CMD_SET_PARAMETER:
                msgLength = 2;
                msgBuffer[1] = STATUS_CMD_OK;
                break;

            case CMD_READ_SIGNATURE_ISP:
            case CMD_READ_OSCCAL_ISP:
                msgLength = 4;
                msgBuffer[1] = STATUS_CMD_OK;
                msgBuffer[2] = boot_signature_byte_get(msgBuffer[0] == CMD_READ_OSCCAL_ISP ? 0x01 : (msgBuffer[4] << 1));
                msgBuffer[3] = STATUS_CMD_OK;
                break;

#ifndef REMOVE_READ_LOCK_FUSE_BIT_SUPPORT
            case CMD_READ_LOCK_ISP:
                msgLength = 4;
                msgBuffer[1] = STATUS_CMD_OK;
                msgBuffer[2] = boot_lock_fuse_bits_get( GET_LOCK_BITS );
                msgBuffer[3] = STATUS_CMD_OK;
                break;

            case CMD_READ_FUSE_ISP:
                {
                    unsigned char fuseBits;

                    if ( msgBuffer[2] == 0x50 )
                    {
                        if ( msgBuffer[3] == 0x08 )
                            fuseBits = boot_lock_fuse_bits_get( GET_EXTENDED_FUSE_BITS );
                        else
                            fuseBits = boot_lock_fuse_bits_get( GET_LOW_FUSE_BITS );
                    }
                    else
                    {
                        fuseBits = boot_lock_fuse_bits_get( GET_HIGH_FUSE_BITS );
                    }
                    msgLength = 4;
                    msgBuffer[1] = STATUS_CMD_OK;
                    msgBuffer[2] = fuseBits;
                    msgBuffer[3] = STATUS_CMD_OK;
                }
                break;
#endif
#ifndef REMOVE_PROGRAM_LOCK_BIT_SUPPORT
            case CMD_PROGRAM_LOCK_ISP:
                {
                    unsigned char lockBits = msgBuffer[4];

                    lockBits = (~lockBits) & 0x3C;  // mask BLBxx bits
                    boot_lock_bits_set(lockBits);   // and program it
                    boot_spm_busy_wait();

                    msgLength = 3;
                    msgBuffer[1] = STATUS_CMD_OK;
                    msgBuffer[2] = STATUS_CMD_OK;
                }
                break;
#endif
            case CMD_CHIP_ERASE_ISP:
                {
#ifndef REMOVE_CMD_CHIP_ERASE_ISP
                    // Erase entire application section of flash, page-by-page.
                    for(address_t addr = 0; addr < APP_END; addr += SPM_PAGESIZE) {
                        boot_page_erase(addr);
                        boot_spm_busy_wait();
                    }
                    boot_rww_enable();

                    // Erase entire EEPROM, but only if EESAVE fuse is not set (fuse
                    // bits are programmed if value is zero).
                    if((boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS) & (~FUSE_EESAVE)) == 0) {
                        for(size_t addr = 0; addr <= E2END; addr++) {
                            eeprom_write_byte((uint8_t *)addr, 0xFF);
                        }
                    }
#endif
                    msgLength = 2;
                    msgBuffer[1] = STATUS_CMD_OK;
                }
                break;

            case CMD_LOAD_ADDRESS:
#if defined(RAMPZ)
                address =  ((address_t)(msgBuffer[1])<<24)|((address_t)(msgBuffer[2])<<16)|((address_t)(msgBuffer[3])<<8)|(msgBuffer[4]);
#else
                address =  ((msgBuffer[3])<<8)|(msgBuffer[4]);
#endif

                msgLength = 2;
                msgBuffer[1] = STATUS_CMD_OK;
                break;

            case CMD_PROGRAM_FLASH_ISP:
            case CMD_PROGRAM_EEPROM_ISP:
                {
                    unsigned int  size = (((unsigned int)msgBuffer[1])<<8) | msgBuffer[2];
                    unsigned char *p = msgBuffer+10;
                    unsigned int  data;
                    unsigned char highByte, lowByte;
                    address_t     byteAddress = (address<<1); // Convert word to byte address (boot_xx macro needs byte address)

                    if ( msgBuffer[0] == CMD_PROGRAM_FLASH_ISP )
                    {
                        // Erase first, only within main section (bootloader protection)
                        if ( byteAddress < APP_END )
                        {
                            boot_page_erase(byteAddress);   // Perform page erase
                            boot_spm_busy_wait();           // Wait until the memory is erased.
                        }

                        /* Write FLASH */
                        do {
                            lowByte   = *p++;
                            highByte  = *p++;

                            data =  (highByte << 8) | lowByte;
                            boot_page_fill(address << 1, data);

                            address++;          // Select next word in memory
                            size -= 2;          // Reduce number of bytes to write by two
                        } while(size);          // Loop until all bytes written

                        boot_page_write(byteAddress);
                        boot_spm_busy_wait();
                        boot_rww_enable();              // Re-enable the RWW section
                    }
                    else
                    {
                        /* write EEPROM */
                        do {
#ifdef USE_LIBC_EEPROM_FUNCS
                            eeprom_write_byte((uint8_t *)(uint16_t)address, *p++);
                            address++;           // Select next EEPROM byte
                            size--;              // Decrease number of bytes to write
#else
                            EEARL = address;            // Setup EEPROM address
                            EEARH = (address >> 8);
                            address++;                  // Select next EEPROM byte

                            EEDR= *p++;                 // get byte from buffer
                            EECR |= (1<<EEMWE);         // Write data into EEPROM
                            EECR |= (1<<EEWE);

                            while (EECR & (1<<EEWE));   // Wait for write operation to finish
                            size--;                     // Decrease number of bytes to write
#endif
                        } while(size);                  // Loop until all bytes written
                    }
                    msgLength = 2;
                    msgBuffer[1] = STATUS_CMD_OK;
                }
                break;

            case CMD_READ_FLASH_ISP:
            case CMD_READ_EEPROM_ISP:
                {
                    unsigned int  size = (((unsigned int)msgBuffer[1])<<8) | msgBuffer[2];
                    unsigned char *p = msgBuffer+1;
                    msgLength = size+3;

                    *p++ = STATUS_CMD_OK;
                    if (msgBuffer[0] == CMD_READ_FLASH_ISP )
                    {
                        unsigned int data;

                        // Read FLASH
                        do {
#if defined(RAMPZ)
                            data = pgm_read_word_far(address<<1);  //convert word to byte address
#else
                            data = pgm_read_word_near(address<<1); //convert word to byte address
#endif
                            *p++ = (unsigned char)data;         //LSB
                            *p++ = (unsigned char)(data >> 8);  //MSB
                            address++;     // Select next word in memory
                            size -= 2;
                        }while (size);
                    }
                    else
                    {
                        /* Read EEPROM */
                        do {
#ifdef USE_LIBC_EEPROM_FUNCS
                            *p++ = eeprom_read_byte((uint8_t *)(uint16_t)address);
                            address++;     // Select next EEPROM byte
                            size--;
#else
                            EEARL = address;            // Setup EEPROM address
                            EEARH = ((address >> 8));
                            address++;                  // Select next EEPROM byte
                            EECR |= (1<<EERE);          // Read EEPROM
                            *p++ = EEDR;                // Send EEPROM data
                            size--;
#endif
                        }while(size);
                    }
                    *p++ = STATUS_CMD_OK;
                }
                break;

            case ANSWER_CKSUM_ERROR:
                // Not really a command, but means the last command received had
                // an invalid checksum, and we should answer back saying so.
                msgLength = 2;
                msgBuffer[1] = STATUS_CKSUM_ERROR;
                break;

            default:
                msgLength = 2;
                msgBuffer[1] = STATUS_CMD_UNKNOWN;
                break;
            }

            /*
             * Now send answer message back
             */
            sendchar(MESSAGE_START);
            checksum = MESSAGE_START^0;

            sendchar(seqNum);
            checksum ^= seqNum;

            c = ((msgLength>>8)&0xFF);
            sendchar(c);
            checksum ^= c;

            c = msgLength&0x00FF;
            sendchar(c);
            checksum ^= c;

            sendchar(TOKEN);
            checksum ^= TOKEN;

            p = msgBuffer;
            while ( msgLength )
            {
               c = *p++;
               sendchar(c);
               checksum ^=c;
               msgLength--;
            }
            sendchar(checksum);

        }//main loop

#ifndef REMOVE_BOOTLOADER_LED
        PROGLED_DDR  &= ~(1<<PROGLED_PIN);   // set to default
#endif
    }

    /*
     * Now leave bootloader
     */
#ifndef REMOVE_PROG_PIN_PULLUP
    PROG_PORT &= ~(1<<PROG_PIN);    // set to default
#endif
    boot_rww_enable();              // enable application section

    // Jump to Reset vector in Application Section
    // (clear register, push this register to the stack twice = address 0x0000/words, and return to this address)
    asm volatile (
        "clr r1" "\n\t"
        "push r1" "\n\t"
        "push r1" "\n\t"
        "ret"     "\n\t"
    ::);

     /*
     * Never return to stop GCC to generate exit return code
     * Actually we will never reach this point, but the compiler doesn't
     * understand this
     */
    for(;;);
}
