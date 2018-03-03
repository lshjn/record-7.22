/****************************************************************************
 * drivers/wireless/cc1101.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *
 *   Authors: Uros Platise <uros.platise@isotel.eu>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* Features:
 *   - Maximum data length: 61 bytes CC1101_PACKET_MAXDATALEN
 *   - Packet length includes two additional bytes: CC1101_PACKET_MAXTOTALLEN
 *   - Requires one GDO to trigger end-of-packets in RX and TX modes.
 *   - Variable packet length with data payload between 1..61 bytes
 *     (three bytes are reserved for packet length, and RSSI and LQI
 *      appended at the end of RXFIFO after each reception)
 *   - Support for General Digital Outputs with overload protection
 *     (single XOSC pin is allowed, otherwise error is returned)
 *   - Loadable RF settings, one for ISM Region 1 (Europe) and one for
 *     ISM Region 2 (Complete America)
 *
 * Todo:
 *   - Extend max packet length up to 255 bytes or rather infinite < 4096 bytes
 *   - Power up/down modes
 *   - Sequencing between states or add protection for correct termination of
 *     various different state (so that CC1101 does not block in case of improper use)
 *
 * RSSI and LQI value interpretation
 *
 * The LQI can be read from the LQI status register or it can be appended
 * to the received packet in the RX FIFO. LQI is a metric of the current
 * quality of the received signal. The LQI gives an estimate of how easily
 * a received signal can be demodulated by accumulating the magnitude of
 * the error between ideal constellations and the received signal over
 * the 64 symbols immediately following the sync word. LQI is best used
 * as a relative measurement of the link quality (a high value indicates
 * a better link than what a low value does), since the value is dependent
 * on the modulation format.
 *
 * To simplify: If the received modulation is FSK or GFSK, the receiver
 * will measure the frequency of each "bit" and compare it with the
 * expected frequency based on the channel frequency and the deviation
 * and the measured frequency offset. If other modulations are used, the
 * error of the modulated parameter (frequency for FSK/GFSK, phase for
 * MSK, amplitude for ASK etc) will be measured against the expected
 * ideal value
 *
 * RSSI (Received Signal Strength Indicator) is a signal strength
 * indication. It does not care about the "quality" or "correctness" of
 * the signal. LQI does not care about the actual signal strength, but
 * the signal quality often is linked to signal strength. This is because
 * a strong signal is likely to be less affected by noise and thus will
 * be seen as "cleaner" or more "correct" by the receiver.
 *
 * There are four to five "extreme cases" that can be used to illustrate
 * how RSSI and LQI work:
 *  1. A weak signal in the presence of noise may give low RSSI and low LQI.
 *  2. A weak signal in "total" absence of noise may give low RSSI and high LQI.
 *  3. Strong noise (usually coming from an interferer) may give high RSSI and low LQI.
 *  4. A strong signal without much noise may give high RSSI and high LQI.
 *  5. A very strong signal that causes the receiver to saturate may give
 *     high RSSI and low LQI.
 *
 * Note that both RSSI and LQI are best used as relative measurements since
 * the values are dependent on the modulation format.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <poll.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wireless/cc1101.h>

//add by liushuhe 2017.11.29

#include <nuttx/kmalloc.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/irq.h>
#include <nuttx/board.h>
//#include <arch/board/board.h>
#include <nuttx/board.h>
#include <sys/boardctl.h>
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_CC1101_NPOLLWAITERS
	#define CONFIG_CC1101_NPOLLWAITERS 2
#endif


#define CC1101_SPIFREQ_BURST    6000000 /* Hz, no delay */
#define CC1101_SPIFREQ_SINGLE   6000000 /* Hz, single access only - no delay */

#define CC1101_MCSM0_VALUE      0x1C

FAR struct cc1101_upperhalf_s *cc1101_fd;


//#define CC1101_GDO2_TX  0x02
//#define CC1101_GDO2_RX  0x00

#define CC1101_GDO2_TX  0x06
#define CC1101_GDO2_RX  0x06

#define CC1101_THER_TX  0x07
#define CC1101_THER_RX  0x07

/****************************************************************************
 * Chipcon CC1101 Internal Registers
 ****************************************************************************/

/* Configuration Registers */

#define CC1101_IOCFG2           0x00        /* GDO2 output pin configuration */
#define CC1101_IOCFG1           0x01        /* GDO1 output pin configuration */
#define CC1101_IOCFG0           0x02        /* GDO0 output pin configuration */
#define CC1101_FIFOTHR          0x03        /* RX FIFO and TX FIFO thresholds */
#define CC1101_SYNC1            0x04        /* Sync word, high byte */
#define CC1101_SYNC0            0x05        /* Sync word, low byte */
#define CC1101_PKTLEN           0x06        /* Packet length */
#define CC1101_PKTCTRL1         0x07        /* Packet automation control */
#define CC1101_PKTCTRL0         0x08        /* Packet automation control */
#define CC1101_ADDR             0x09        /* Device address */
#define CC1101_CHANNR           0x0A        /* Channel number */
#define CC1101_FSCTRL1          0x0B        /* Frequency synthesizer control */
#define CC1101_FSCTRL0          0x0C        /* Frequency synthesizer control */
#define CC1101_FREQ2            0x0D        /* Frequency control word, high byte */
#define CC1101_FREQ1            0x0E        /* Frequency control word, middle byte */
#define CC1101_FREQ0            0x0F        /* Frequency control word, low byte */
#define CC1101_MDMCFG4          0x10        /* Modem configuration */
#define CC1101_MDMCFG3          0x11        /* Modem configuration */
#define CC1101_MDMCFG2          0x12        /* Modem configuration */
#define CC1101_MDMCFG1          0x13        /* Modem configuration */
#define CC1101_MDMCFG0          0x14        /* Modem configuration */
#define CC1101_DEVIATN          0x15        /* Modem deviation setting */
#define CC1101_MCSM2            0x16        /* Main Radio Cntrl State Machine config */
#define CC1101_MCSM1            0x17        /* Main Radio Cntrl State Machine config */
#define CC1101_MCSM0            0x18        /* Main Radio Cntrl State Machine config */
#define CC1101_FOCCFG           0x19        /* Frequency Offset Compensation config */
#define CC1101_BSCFG            0x1A        /* Bit Synchronization configuration */
#define CC1101_AGCCTRL2         0x1B        /* AGC control */
#define CC1101_AGCCTRL1         0x1C        /* AGC control */
#define CC1101_AGCCTRL0         0x1D        /* AGC control */
#define CC1101_WOREVT1          0x1E        /* High byte Event 0 timeout */
#define CC1101_WOREVT0          0x1F        /* Low byte Event 0 timeout */
#define CC1101_WORCTRL          0x20        /* Wake On Radio control */
#define CC1101_FREND1           0x21        /* Front end RX configuration */
#define CC1101_FREND0           0x22        /* Front end TX configuration */
#define CC1101_FSCAL3           0x23        /* Frequency synthesizer calibration */
#define CC1101_FSCAL2           0x24        /* Frequency synthesizer calibration */
#define CC1101_FSCAL1           0x25        /* Frequency synthesizer calibration */
#define CC1101_FSCAL0           0x26        /* Frequency synthesizer calibration */
#define CC1101_RCCTRL1          0x27        /* RC oscillator configuration */
#define CC1101_RCCTRL0          0x28        /* RC oscillator configuration */
#define CC1101_FSTEST           0x29        /* Frequency synthesizer cal control */
#define CC1101_PTEST            0x2A        /* Production test */
#define CC1101_AGCTEST          0x2B        /* AGC test */
#define CC1101_TEST2            0x2C        /* Various test settings */
#define CC1101_TEST1            0x2D        /* Various test settings */
#define CC1101_TEST0            0x2E        /* Various test settings */

/* Status registers */

#define CC1101_PARTNUM          (0x30 | 0xc0)   /* Part number */
#define CC1101_VERSION          (0x31 | 0xc0)   /* Current version number */
#define CC1101_FREQEST          (0x32 | 0xc0)   /* Frequency offset estimate */
#define CC1101_LQI              (0x33 | 0xc0)   /* Demodulator estimate for link quality */
#define CC1101_RSSI             (0x34 | 0xc0)   /* Received signal strength indication */
#define CC1101_MARCSTATE        (0x35 | 0xc0)   /* Control state machine state */
#define CC1101_WORTIME1         (0x36 | 0xc0)   /* High byte of WOR timer */
#define CC1101_WORTIME0         (0x37 | 0xc0)   /* Low byte of WOR timer */
#define CC1101_PKTSTATUS        (0x38 | 0xc0)   /* Current GDOx status and packet status */
#define CC1101_VCO_VC_DAC       (0x39 | 0xc0)   /* Current setting from PLL cal module */
#define CC1101_TXBYTES          (0x3A | 0xc0)   /* Underflow and # of bytes in TXFIFO */
#define CC1101_RXBYTES          (0x3B | 0xc0)   /* Overflow and # of bytes in RXFIFO */
#define CC1101_RCCTRL1_STATUS   (0x3C | 0xc0)   /* Last RC oscilator calibration results */
#define CC1101_RCCTRL0_STATUS   (0x3D | 0xc0)   /* Last RC oscilator calibration results */

/* Multi byte memory locations */

#define CC1101_PATABLE          0x3E
#define CC1101_TXFIFO           0x3F
#define CC1101_RXFIFO           0x3F

/* Definitions for burst/single access to registers */

#define CC1101_WRITE_BURST      0x40
#define CC1101_READ_SINGLE      0x80
#define CC1101_READ_BURST       0xC0

/* Strobe commands */

#define CC1101_SRES             0x30        /* Reset chip. */
#define CC1101_SFSTXON          0x31        /* Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1). */
#define CC1101_SXOFF            0x32        /* Turn off crystal oscillator. */
#define CC1101_SCAL             0x33        /* Calibrate frequency synthesizer and turn it off */
#define CC1101_SRX              0x34        /* Enable RX. Perform calibration first if switching from IDLE and MCSM0.FS_AUTOCAL=1. */
#define CC1101_STX              0x35        /* Enable TX. Perform calibration first if IDLE and MCSM0.FS_AUTOCAL=1.  */
                                            /* If switching from RX state and CCA is enabled then go directly to TX if channel is clear. */
#define CC1101_SIDLE            0x36        /* Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable. */
#define CC1101_SAFC             0x37        /* Perform AFC adjustment of the frequency synthesizer */
#define CC1101_SWOR             0x38        /* Start automatic RX polling sequence (Wake-on-Radio) */
#define CC1101_SPWD             0x39        /* Enter power down mode when CSn goes high. */
#define CC1101_SFRX             0x3A        /* Flush the RX FIFO buffer. */
#define CC1101_SFTX             0x3B        /* Flush the TX FIFO buffer. */
#define CC1101_SWORRST          0x3C        /* Reset real time clock. */
#define CC1101_SNOP             0x3D        /* No operation. */

/* Modem Control */

#define CC1101_MCSM0_XOSC_FORCE_ON  0x01

/* Chip Status Byte
 */

/* Bit fields in the chip status byte */

#define CC1101_STATUS_CHIP_RDYn_BM              0x80
#define CC1101_STATUS_STATE_BM                  0x70
#define CC1101_STATUS_FIFO_BYTES_AVAILABLE_BM   0x0F

/* Chip states */

#define CC1101_STATE_MASK                       0x70
#define CC1101_STATE_IDLE                       0x00
#define CC1101_STATE_RX                         0x10
#define CC1101_STATE_TX                         0x20
#define CC1101_STATE_FSTXON                     0x30
#define CC1101_STATE_CALIBRATE                  0x40
#define CC1101_STATE_SETTLING                   0x50
#define CC1101_STATE_RX_OVERFLOW                0x60
#define CC1101_STATE_TX_UNDERFLOW               0x70

/* Values of the MACRSTATE register */

#define CC1101_MARCSTATE_SLEEP                  0x00
#define CC1101_MARCSTATE_IDLE                   0x01
#define CC1101_MARCSTATE_XOFF                   0x02
#define CC1101_MARCSTATE_VCOON_MC               0x03
#define CC1101_MARCSTATE_REGON_MC               0x04
#define CC1101_MARCSTATE_MANCAL                 0x05
#define CC1101_MARCSTATE_VCOON                  0x06
#define CC1101_MARCSTATE_REGON                  0x07
#define CC1101_MARCSTATE_STARTCAL               0x08
#define CC1101_MARCSTATE_BWBOOST                0x09
#define CC1101_MARCSTATE_FS_LOCK                0x0A
#define CC1101_MARCSTATE_IFADCON                0x0B
#define CC1101_MARCSTATE_ENDCAL                 0x0C
#define CC1101_MARCSTATE_RX                     0x0D
#define CC1101_MARCSTATE_RX_END                 0x0E
#define CC1101_MARCSTATE_RX_RST                 0x0F
#define CC1101_MARCSTATE_TXRX_SWITCH            0x10
#define CC1101_MARCSTATE_RXFIFO_OVERFLOW        0x11
#define CC1101_MARCSTATE_FSTXON                 0x12
#define CC1101_MARCSTATE_TX                     0x13
#define CC1101_MARCSTATE_TX_END                 0x14
#define CC1101_MARCSTATE_RXTX_SWITCH            0x15
#define CC1101_MARCSTATE_TXFIFO_UNDERFLOW       0x16

/* Part number and version */

#define CC1101_PARTNUM_VALUE                    0x00
//#define CC1101_VERSION_VALUE                    0x04
//add by liushuhe 2017.11.28
#define CC1101_VERSION_VALUE                    0x03

/*  Others ... */

#define CC1101_LQI_CRC_OK_BM                    0x80
#define CC1101_LQI_EST_BM                       0x7F

#define CC1101_DELAY_TIME                      50

//add by liushuhe 2018.01.04
#define CC1101_MODE_RX                      1
#define CC1101_MODE_TX                      2

#define SUCCESS                      		 1
#define FAIL                      			 0

#define TIMR1_CNT_ADDR 0x40010024
#define TIMR2_CNT_ADDR 0x40000024



#define ALIGN __attribute__((packed))


typedef struct ALIGN _timemsg{
	uint8_t  start_flag;
	uint8_t  msglen;
	uint8_t  type;
	uint8_t  dist;
	uint8_t  src;
	uint32_t second; 		//定时器中断累计值
	uint32_t us;            //定时器的cnt
	uint8_t  endflag;        
}_cc110x_timemsg;

//_cc110x_timemsg cc1101_timemsg_tx;

_cc110x_timemsg * _Pcc1101_timemsg_tx = NULL;
_cc110x_timemsg * _Pcc1101_timemsg_timer_rx = NULL;

uint32_t  cc1101_timer2_us = 0;

/*
uint8_t PA_table[8] = {0x60 ,0x60 ,0x60 ,0x60 ,0x60 ,0x60 ,0x60 ,0x60};

struct c1101_rfsettings_s rfSettings = {
	.IOCFG2 = CC1101_GDO2_RX,    // GDO2 output pin configuration 00
	.IOCFG1 = 0x2e,    // GDO0 output pin configuration
	.IOCFG0 = CC1101_GDO2_RX,    // GDO0 output pin configuration

	.FIFOTHR = CC1101_THER_RX,    // FIFOTHR 60

	.SYNC1 = 0xd3,      // Frequency control word, middle byte. 
	.SYNC0 = 0x91,      // Frequency control word, low byte.

	.PKTLEN = 0xff,    // Packet length.

	.PKTCTRL1 = 0x04,  // Packet automation control.
	.PKTCTRL0 = 0x05,  // Packet automation control.


	.ADDR = 0x00,      // Device address.

	.CHANNR = 0x00,     // Channel number

	.FSCTRL1 = 0x08,    // Frequency synthesizer control. 
	.FSCTRL0 = 0x00,    // Frequency synthesizer control. 

	.FREQ2 = 0x10,      // Frequency control word, high byte. 
	.FREQ1 = 0xa7,      // Frequency control word, middle byte. 
	.FREQ0 = 0x62,      // Frequency control word, low byte. 

	.MDMCFG4 = 0x5b,  // MDMCFG4 Modem configuration.
	.MDMCFG3 = 0xf8,  // MDMCFG3 Modem configuration.
	.MDMCFG2 = 0x03,  // MDMCFG2 Modem configuration.
	.MDMCFG1 = 0x22, // MDMCFG1 Modem configuration.
	.MDMCFG0 = 0xF8, // MDMCFG0 Modem configuration.

	.DEVIATN = 0x47,    // Modem deviation setting (when FSK modulation is enabled). 

	.MCSM2 = 0x07,     // Main Radio Control State Machine configuration.
	.MCSM1 = 0x30,     // Main Radio Control State Machine configuration.
	.MCSM0 = 0x18,     // Main Radio Control State Machine configuration.

	.FOCCFG = 0x1d,     // Frequency Offset Compensation Configuration. 
	.BSCFG = 0x1c,      // Bit synchronization Configuration. 

	.AGCCTRL2 =0xc7, // AGCCTRL2 AGC control.
	.AGCCTRL1 =0x00, // AGCCTRL1 AGC control.
	.AGCCTRL0 =0xb2, // AGCCTRL0 AGC control.

	.WOREVT1 = 0x87,
	.WOREVT0 = 0x6b,	
	.WORCTRL = 0xf8,

	.FREND1 = 0xb6,     // Front end RX configuration. 
	.FREND0 = 0x10,

	.FSCAL3 =0xea, // FSCAL3 Frequency synthesizer calibration.
	.FSCAL2 =0x2A, // FSCAL2 Frequency synthesizer calibration.
	.FSCAL1 =0x00, // FSCAL1 Frequency synthesizer calibration.
	.FSCAL0 =0x11, // FSCAL0 Frequency synthesizer calibration.

    .RCCTRL1 = 0x41,
    .RCCTRL1 = 0x00,
	.FSTEST =0x59, // FSTEST Frequency synthesizer calibration.
	.PTEST  =0x7f,
	.AGCTEST=0x3f,
	
	.TEST2  =0x81, // TEST2 Various test settings.
	.TEST1  =0x35, // TEST1 Various test settings.
	.TEST0  =0x09, // TEST0 Various test settings.
			
};

*/

/*
lhc old
uint8_t PA_table[8] = {0xc0,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

struct c1101_rfsettings_s rfSettings = {
    .IOCFG2 = 0x06,    // GDO2 output pin configuration
    .IOCFG1 = 0x2e,    // GDO0 output pin configuration
    .IOCFG0 = 0x0e,    // GDO0 output pin configuration

    .FIFOTHR = 0x07,    // FIFOTHR

	.SYNC1 = 0x9b,      // Frequency control word, middle byte. 
	.SYNC0 = 0xab,      // Frequency control word, low byte.

    .PKTLEN = 0xff,    // Packet length.

    .PKTCTRL1 = 0x04,  // Packet automation control.
    .PKTCTRL0 = 0x05,  // Packet automation control.


    .ADDR = 0xff,      // Device address.

	.CHANNR = 0x00,     // Channel number

	.FSCTRL1 = 0x0f,    // Frequency synthesizer control. 
	.FSCTRL0 = 0x00,    // Frequency synthesizer control. 
	
	.FREQ2 = 0x10,      // Frequency control word, high byte. 
	.FREQ1 = 0xa7,      // Frequency control word, middle byte. 
	.FREQ0 = 0x62,      // Frequency control word, low byte. 

	.MDMCFG4 =0x6c,    // Modem configuration. 
	.MDMCFG3 =0x48,    // Modem configuration. 
	.MDMCFG2 =0x06,    // Modem configuration. 
	.MDMCFG1 =0xc3,    // Modem configuration. 
	.MDMCFG0 =0x3b,    // Modem configuration. 

	.DEVIATN = 0x44,    // Modem deviation setting (when FSK modulation is enabled). 
	
	.MCSM2 = 0x07,     // Main Radio Control State Machine configuration.
	.MCSM1 = 0x03,     // Main Radio Control State Machine configuration.
	.MCSM0 = 0x18,     // Main Radio Control State Machine configuration.

	.FOCCFG = 0x16,     // Frequency Offset Compensation Configuration. 
	.BSCFG = 0x6c,      // Bit synchronization Configuration. 

	.AGCCTRL2 = 0x45,   // AGC control. 
	.AGCCTRL1 = 0x40,   // AGC control. 
	.AGCCTRL0 =0x91,   // AGC control. 

	.WOREVT1 = 0x28,
	.WOREVT0 = 0xa0,	
	.WORCTRL = 0x38,

	
	.FREND1 = 0x56,     // Front end RX configuration. 
	.FREND0 = 0x10,

	.FSCAL3 = 0xea,     // Frequency synthesizer calibration. 
	.FSCAL2 = 0x2a,     // Frequency synthesizer calibration. 
	.FSCAL1 = 0x00,     // Frequency synthesizer calibration. 
	.FSCAL0 = 0x1f,    // Frequency synthesizer calibration. 
	
    .RCCTRL1 = 0x00,
};
*/


/*
//lhc new1
uint8_t PA_table[8] = {0xc0,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

struct c1101_rfsettings_s rfSettings = {
    .IOCFG2 = 0x06,    // GDO2 output pin configuration
    .IOCFG1 = 0x2e,    // GDO0 output pin configuration
    .IOCFG0 = 0x0e,    // GDO0 output pin configuration

    .FIFOTHR = 0x07,    // FIFOTHR

	.SYNC1 = 0x9b,      // Frequency control word, middle byte. 
	.SYNC0 = 0xad,      // Frequency control word, low byte.

    .PKTLEN = 0xff,    // Packet length.

    .PKTCTRL1 = 0x04,  // Packet automation control.
    .PKTCTRL0 = 0x05,  // Packet automation control.


    .ADDR = 0xff,      // Device address.

	.CHANNR = 0x00,     // Channel number

	.FSCTRL1 = 0x0f,    // Frequency synthesizer control. 
	.FSCTRL0 = 0x00,    // Frequency synthesizer control. 
	
	.FREQ2 = 0x10,      // Frequency control word, high byte. 
	.FREQ1 = 0xa7,      // Frequency control word, middle byte. 
	.FREQ0 = 0x62,      // Frequency control word, low byte. 

	.MDMCFG4 =0x1e,    // Modem configuration. 
	.MDMCFG3 =0x3b,    // Modem configuration. 
	.MDMCFG2 =0x73,    // Modem configuration. 
	.MDMCFG1 =0x42,    // Modem configuration. 
	.MDMCFG0 =0xf8,    // Modem configuration. 

	.DEVIATN = 0x44,    // Modem deviation setting (when FSK modulation is enabled). 
	
	.MCSM2 = 0x07,     // Main Radio Control State Machine configuration.
	.MCSM1 = 0x03,     // Main Radio Control State Machine configuration.
	.MCSM0 = 0x18,     // Main Radio Control State Machine configuration.

	.FOCCFG = 0x16,     // Frequency Offset Compensation Configuration. 
	.BSCFG = 0x6c,      // Bit synchronization Configuration. 

	.AGCCTRL2 = 0x45,   // AGC control. 
	.AGCCTRL1 = 0x40,   // AGC control. 
	.AGCCTRL0 =0x91,   // AGC control. 

	.WOREVT1 = 0x28,
	.WOREVT0 = 0xa0,	
	.WORCTRL = 0x38,

	
	.FREND1 = 0x56,     // Front end RX configuration. 
	.FREND0 = 0x10,

	.FSCAL3 = 0xea,     // Frequency synthesizer calibration. 
	.FSCAL2 = 0x2a,     // Frequency synthesizer calibration. 
	.FSCAL1 = 0x00,     // Frequency synthesizer calibration. 
	.FSCAL0 = 0x1f,    // Frequency synthesizer calibration. 
	
    .RCCTRL1 = 0x00,
};
*/

/*

//lhc new2
uint8_t PA_table[8] = {0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
struct c1101_rfsettings_s rfSettings = {
    .IOCFG2 = 0x06,    // GDO2 output pin configuration
    .IOCFG1 = 0x2e,    // GDO0 output pin configuration
    .IOCFG0 = 0x0e,    // GDO0 output pin configuration
    .FIFOTHR = 0x07,    // FIFOTHR
	.SYNC1 = 0x9b,      // Frequency control word, middle byte. 
	.SYNC0 = 0xad,      // Frequency control word, low byte.
    .PKTLEN = 0xff,    // Packet length.
    .PKTCTRL1 = 0x04,  // Packet automation control.
    .PKTCTRL0 = 0x05,  // Packet automation control.
    .ADDR = 0x00,      // Device address.
	.CHANNR = 0x00,     // Channel number
	.FSCTRL1 = 0x0a,    // Frequency synthesizer control. 
	.FSCTRL0 = 0x00,    // Frequency synthesizer control. 
	.FREQ2 = 0x10,      // Frequency control word, high byte. 
	.FREQ1 = 0xb1,      // Frequency control word, middle byte. 
	.FREQ0 = 0x3b,      // Frequency control word, low byte. 
	.MDMCFG4 =0x1e,    // Modem configuration. 
	.MDMCFG3 =0x3b,    // Modem configuration. 
	.MDMCFG2 =0x73,    // Modem configuration. 
	.MDMCFG1 =0x23,    // Modem configuration. 
	.MDMCFG0 =0xf8,    // Modem configuration. 
	.DEVIATN = 0x00,    // Modem deviation setting (when FSK modulation is enabled). 
	.MCSM2 = 0x07,     // Main Radio Control State Machine configuration.
	.MCSM1 = 0x30,     // Main Radio Control State Machine configuration.
	.MCSM0 = 0x18,     // Main Radio Control State Machine configuration.
	.FOCCFG = 0x1d,     // Frequency Offset Compensation Configuration. 
	.BSCFG = 0x1c,      // Bit synchronization Configuration. 
	.AGCCTRL2 = 0xc7,   // AGC control. 
	.AGCCTRL1 = 0x00,   // AGC control. 
	.AGCCTRL0 =0xb0,   // AGC control. 
	.WOREVT1 = 0x28,
	.WOREVT0 = 0xa0,	
	.WORCTRL = 0x38,
	.FREND1 = 0xb6,     // Front end RX configuration. 
	.FREND0 = 0x10,
	.FSCAL3 = 0xea,     // Frequency synthesizer calibration. 
	.FSCAL2 = 0x2a,     // Frequency synthesizer calibration. 
	.FSCAL1 = 0x00,     // Frequency synthesizer calibration. 
	.FSCAL0 = 0x1f,    // Frequency synthesizer calibration. 
    .RCCTRL1 = 0x41,
    .RCCTRL1 = 0x00,
	.FSTEST =0x59, // FSTEST Frequency synthesizer calibration.
	.PTEST  =0x7f,
	.AGCTEST=0x3f,
	.TEST2  =0x88, // TEST2 Various test settings.
	.TEST1  =0x31, // TEST1 Various test settings.
	.TEST0  =0x09, // TEST0 Various test settings.};
};

*/


//lhc new3
uint8_t PA_table[8] = {0x8e,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
struct c1101_rfsettings_s rfSettings = {
    .IOCFG2 = 0x06,    // GDO2 output pin configuration
    .IOCFG1 = 0x2e,    // GDO0 output pin configuration
    .IOCFG0 = 0x0e,    // GDO0 output pin configuration
    .FIFOTHR = 0x07,    // FIFOTHR
	.SYNC1 = 0x9b,      // Frequency control word, middle byte. 
	.SYNC0 = 0xad,      // Frequency control word, low byte.
    .PKTLEN = 0xff,    // Packet length.
    .PKTCTRL1 = 0x05,  // Packet automation control.
    //.PKTCTRL1 = 0x04,  // Packet automation control.
    .PKTCTRL0 = 0x05,  // Packet automation control.
    .ADDR = 0x10,      // Device address.
	.CHANNR = 0x00,     // Channel number
	.FSCTRL1 = 0x0f,    // Frequency synthesizer control. 
	.FSCTRL0 = 0x00,    // Frequency synthesizer control. 
	.FREQ2 = 0x10,      // Frequency control word, high byte. 
	.FREQ1 = 0xa7,      // Frequency control word, middle byte. 
	.FREQ0 = 0x62,      // Frequency control word, low byte. 
	.MDMCFG4 =0x1e,    // Modem configuration. 
	.MDMCFG3 =0x3b,    // Modem configuration. 
	.MDMCFG2 =0x73,    // Modem configuration. 
	.MDMCFG1 =0x42,    // Modem configuration. 
	.MDMCFG0 =0xf8,    // Modem configuration. 
	.DEVIATN = 0x44,    // Modem deviation setting (when FSK modulation is enabled). 
	.MCSM2 = 0x07,     // Main Radio Control State Machine configuration.
	//.MCSM1 = 0x03,     // Main Radio Control State Machine configuration.
	.MCSM1 = 0x0f,     // Main Radio Control State Machine configuration.
	.MCSM0 = 0x18,     // Main Radio Control State Machine configuration.
	.FOCCFG = 0x16,     // Frequency Offset Compensation Configuration. 
	.BSCFG = 0x6c,      // Bit synchronization Configuration. 
	.AGCCTRL2 = 0x45,   // AGC control. 
	.AGCCTRL1 = 0x40,   // AGC control. 
	.AGCCTRL0 =0x91,   // AGC control. 
	.WOREVT1 = 0x28,
	.WOREVT0 = 0xa0,	
	.WORCTRL = 0x38,
	.FREND1 = 0x56,     // Front end RX configuration. 
	.FREND0 = 0x10,
	.FSCAL3 = 0xea,     // Frequency synthesizer calibration. 
	.FSCAL2 = 0x2a,     // Frequency synthesizer calibration. 
	.FSCAL1 = 0x00,     // Frequency synthesizer calibration. 
	.FSCAL0 = 0x1f,    // Frequency synthesizer calibration. 
    .RCCTRL1 = 0x41,
    .RCCTRL1 = 0x00,
	.FSTEST =0x59, // FSTEST Frequency synthesizer calibration.
	.PTEST  =0x7f,
	.AGCTEST=0x3f,
	.TEST2  =0x88, // TEST2 Various test settings.
	.TEST1  =0x31, // TEST1 Various test settings.
	.TEST0  =0x09, // TEST0 Various test settings.};
};





/****************************************************************************
 * Private Data Types
 ****************************************************************************/

#define FLAGS_RXONLY        1   /* Indicates receive operation only */
#define FLAGS_XOSCENABLED   2   /* Indicates that one pin is configured as XOSC/n */
/*********************************************************************************/
static int fs_open(FAR struct file *filep);
static int fs_close(FAR struct file *filep);
static ssize_t fs_read(FAR struct file *filep, FAR char *buffer, size_t buflen);
static ssize_t fs_write(FAR struct file *filep, FAR const char *buffer, size_t buflen);
static int fs_poll(FAR struct file *filep, FAR struct pollfd *fds,bool setup);
static int fs_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

#define  GETCC1101BUF_BYTES  0x01


struct cc1101_status
{
	int		workmode;
	int		rx_status;
	int		tx_status;
	
    uint8_t rxbuf[200]; 

	int		rx_len_drv;
	int		rx_len;
	int		tx_len;
};

struct cc1101_status  cc1101_rxtx_status;

struct cc1101_dev_s
{
  const struct c1101_rfsettings_s *rfsettings;

  FAR struct spi_dev_s *spi;
  uint8_t           isrpin;     /* CC1101 pin used to trigger interrupts */
  uint32_t          pin_int;     /* GPIO of the MCU */
  uint32_t          pin_miso;   /* spi miso */
  uint32_t          pin_mosi;   /* spi mosi */
  uint8_t           flags;
  uint8_t           channel;
  uint8_t           power;
};

/*********************************************************************************/
//add by liushuhe 2017.11.30
struct cc1101_operations_s
{
	CODE int (*init)(struct cc1101_dev_s *dev);
	CODE int (*setmode_receive)(struct cc1101_dev_s *dev);
	CODE int (*setmode_send)(struct cc1101_dev_s *dev);
	CODE int (*setmode_idle)(struct cc1101_dev_s *dev);

	CODE int (*read)(struct cc1101_dev_s *dev, uint8_t * buf, size_t size);
	CODE int (*write)(struct cc1101_dev_s *dev, const uint8_t *buf, size_t size);
};

/*********************************************************************************/
//add by liushuhe 2017.11.30
static struct cc1101_operations_s g_cc1101_devops =
{
	.init      			= cc1101_init,
	.read 				= cc1101_read,
	.write 				= cc1101_write,
	.setmode_receive    = cc1101_receive,
	.setmode_send 		= cc1101_send,
	.setmode_idle 		= cc1101_idle,
};

/*********************************************************************************/
//add by liushuhe 2017.11.30
struct cc1101_lowerhalf_s
{
	FAR const struct cc1101_operations_s *ops;
	FAR struct cc1101_dev_s *c1101_dev;
};
/*********************************************************************************/
//add by liushuhe 2017.11.30
struct cc1101_upperhalf_s
{
  	FAR struct cc1101_lowerhalf_s *dev;  /* lower-half state */
	uint8_t crefs;                      /* Number of open references */
	bool unlinked;                      /* True if the driver has been unlinked */
	sem_t exclsem;  					/* Supports mutual exclusion */
    sem_t devsem;
    struct pollfd *fds[CONFIG_CC1101_NPOLLWAITERS];
};
/*********************************************************************************/
//add by liushuhe 2017.11.30
static const struct file_operations g_cc1101_drvrops =
{
  fs_open,    		/* open */
  fs_close,   		/* close */
  fs_read,    		/* read */
  fs_write,   		/* write */
  0,           		/* seek */
  fs_ioctl,   		/* ioctl */
  fs_poll,		   	/*poll*/
  0,      			/* unlink */
};

struct ring_buf
{
	char	*g_Buf;
	int		g_iReadPos;
	int		g_iWritePos;
};

#define CC1101_BUF_SIZE   (1024*4)

struct ring_buf cc1101_buf;
/****************************************************************************
 * Private Data
 ****************************************************************************/

static volatile int cc1101_interrupt = 0;

/************************************************************************************/
//环形缓冲区
static int isFull(void)
{
	return (((cc1101_buf.g_iWritePos + 1) % CC1101_BUF_SIZE) == cc1101_buf.g_iReadPos);
}


static int isEmpty(void)
{
	return (cc1101_buf.g_iWritePos == cc1101_buf.g_iReadPos);
}

static int PutData(uint8_t cVal)
{
	if (isFull())
		return -1;
	else
	{
		cc1101_buf.g_Buf[cc1101_buf.g_iWritePos] = cVal;
		cc1101_buf.g_iWritePos = (cc1101_buf.g_iWritePos + 1) % CC1101_BUF_SIZE;
		return 0;
	}	
}

static int GetData(char *pcVal)
{
	if (isEmpty())
		return -1;
	else
	{
		*pcVal = cc1101_buf.g_Buf[cc1101_buf.g_iReadPos];
		cc1101_buf.g_iReadPos = (cc1101_buf.g_iReadPos + 1) % CC1101_BUF_SIZE;
		return 0;
	}
}


static int DatePrint(uint8_t *strData,int len)
{
	/* 把数据放入环形缓冲区 */
	int i;
	
	for (i = 0; i < len; i++)
	{
		if (0 != PutData(strData[i]))
			break;
	}	
	return i;	
}



/************************************************************************************/

/*********************************************************************************/
//add by liushuhe 2017.11.30
static void CC1101_pollnotify(FAR struct cc1101_upperhalf_s *priv)
{
  DEBUGASSERT(priv != NULL);
  int i;

  /* If there are threads waiting on poll() for data to become available,
   * then wake them up now.  NOTE: we wake up all waiting threads because we
   * do not know that they are going to do.  If they all try to read the data,
   * then some make end up blocking after all.
   */

  for (i = 0; i < CONFIG_CC1101_NPOLLWAITERS; i++)
    {
      FAR struct pollfd *fds = priv->fds[i];
		   
      if (fds)
        {
          fds->revents |= POLLIN;
          nxsem_post(fds->sem);
        }
    }
  
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

void cc1101_access_begin(FAR struct cc1101_dev_s *dev)
{
  //(void)SPI_LOCK(dev->spi, true);
  SPI_SELECT(dev->spi, SPIDEV_WIRELESS(0), true);
  SPI_SETMODE(dev->spi, SPIDEV_MODE0);     /* CPOL=0, CPHA=0 */
  SPI_SETBITS(dev->spi, 8);
  (void)SPI_HWFEATURES(dev->spi, 0);
}

void cc1101_access_end(FAR struct cc1101_dev_s *dev)
{
  SPI_SELECT(dev->spi, SPIDEV_WIRELESS(0), false);
  //(void)SPI_LOCK(dev->spi, false);
}

/* CC1101 Access with Range Check
 *
 * Input Paramters:
 *   dev CC1101 Private Structure
 *   addr CC1101 Address
 *   buf Pointer to buffer, either for read or write access
 *   length when >0 it denotes read access, when <0 it denotes write
 *        access of -length. abs(length) greater of 1 implies burst mode,
 *        however
 *
 * Returned Value:
 *   OK on success or errno is set.
 */



int cc1101_access(FAR struct cc1101_dev_s *dev, uint8_t addr,
                  FAR uint8_t *buf, int length)
{
  int stabyte;

  /* Address cannot explicitly define READ command while length WRITE.
   * Also access to these cells is only permitted as one byte, eventhough
   * transfer is marked as BURST!
   */

  if ((addr & CC1101_READ_SINGLE) && length != 1)
    {
      return ERROR;
    }

  /* Prepare SPI */

  cc1101_access_begin(dev);


#if 0
  //add by liushuhe 2017.12.03
  while(stm32_gpioread(dev->pin_miso));
#else if
 int i=0;
 for(i=0;i<168*10;i++);
#endif

  if (length > 1 || length < -1)
    {
      SPI_SETFREQUENCY(dev->spi, CC1101_SPIFREQ_BURST);
    }
  else
    {
      SPI_SETFREQUENCY(dev->spi, CC1101_SPIFREQ_SINGLE);
    }

  /* Transfer */

  if (length <= 0)
    {
      /* 0 length are command strobes */

      if (length < -1)
        {
          addr |= CC1101_WRITE_BURST;
        }

      stabyte = SPI_SEND(dev->spi, addr);
	  
      if (length)
        {
          SPI_SNDBLOCK(dev->spi, buf, -length);
        }
    }
  else
    {
      addr |= CC1101_READ_SINGLE;
      if (length > 1)
        {
          addr |= CC1101_READ_BURST;
        }

      stabyte = SPI_SEND(dev->spi, addr);
      SPI_RECVBLOCK(dev->spi, buf, length);
    }

  cc1101_access_end(dev);
  //add by liushuhe 2017.12.03
  //usleep(CC1101_DELAY_TIME*1000);

  return abs(length);
}


int cc1101_sendData(FAR struct cc1101_dev_s *dev, uint8_t addr,
                  FAR uint8_t *buflen,FAR uint8_t *buf, int length)
{
  int stabyte;

  /* Address cannot explicitly define READ command while length WRITE.
   * Also access to these cells is only permitted as one byte, eventhough
   * transfer is marked as BURST!
   */

  if ((addr & CC1101_READ_SINGLE) && length != 1)
    {
      return ERROR;
    }

  /* Prepare SPI */

  cc1101_access_begin(dev);

  //add by liushuhe 2017.12.03
  while(stm32_gpioread(dev->pin_miso));



  if (length > 1 || length < -1)
    {
      SPI_SETFREQUENCY(dev->spi, CC1101_SPIFREQ_BURST);
    }
  else
    {
      SPI_SETFREQUENCY(dev->spi, CC1101_SPIFREQ_SINGLE);
    }

  /* Transfer */

  if (length <= 0)
    {
      /* 0 length are command strobes */

      stabyte = SPI_SEND(dev->spi, addr);
	  
      SPI_SNDBLOCK(dev->spi, buflen, -1);

      addr |= CC1101_WRITE_BURST;

      stabyte = SPI_SEND(dev->spi, addr);
	  
      SPI_SNDBLOCK(dev->spi, buf, -length);

	  
    }

  cc1101_access_end(dev);

  return stabyte;
}


/* Strobes command and returns chip status byte
 *
 *  By default commands are send as Write. To a command,
 *  CC1101_READ_SINGLE may be OR'ed to obtain the number of RX bytes
 *  pending in RX FIFO.
 */

inline uint8_t cc1101_strobe(FAR struct cc1101_dev_s *dev, uint8_t command)
{
  uint8_t status;

  cc1101_access_begin(dev);
  SPI_SETFREQUENCY(dev->spi, CC1101_SPIFREQ_SINGLE);


#if 0
  //add by liushuhe 2017.12.03
  while(stm32_gpioread(dev->pin_miso));
#else if
 int i=0;
 for(i=0;i<168*10;i++);
#endif

  status = SPI_SEND(dev->spi, command);

  cc1101_access_end(dev);

  //add by liushuhe 2017.12.03
  //usleep(CC1101_DELAY_TIME*1000);
  return status;
}


/*
int cc1101_reset(struct cc1101_dev_s *dev)
{
	
	cc1101_strobe(dev, CC1101_SRES);
	return OK;
}
*/
//add by liushuhe 2017.12.01
int cc1101_reset(FAR struct cc1101_dev_s *dev)
{
 	//(void)SPI_LOCK(dev->spi, true);
	SPI_SETMODE(dev->spi, SPIDEV_MODE0);     /* CPOL=0, CPHA=0 */
	SPI_SETBITS(dev->spi, 8);
	(void)SPI_HWFEATURES(dev->spi, 0);
	
	SPI_SELECT(dev->spi, SPIDEV_WIRELESS(0), false);     //high
    usleep(2);
	SPI_SELECT(dev->spi, SPIDEV_WIRELESS(0), true);      //lower
    usleep(2);
	SPI_SELECT(dev->spi, SPIDEV_WIRELESS(0), false);     //high
    usleep(45);
	SPI_SELECT(dev->spi, SPIDEV_WIRELESS(0), true);      //lower
	
	while(stm32_gpioread(dev->pin_miso));
	SPI_SEND(dev->spi, CC1101_SRES);
	while(stm32_gpioread(dev->pin_miso));
	
	SPI_SELECT(dev->spi, SPIDEV_WIRELESS(0), false);      //high
	//(void)SPI_LOCK(dev->spi, false);
	return OK;
}


int cc1101_checkpart(FAR struct cc1101_dev_s *dev)
{
  uint8_t partnum;
  uint8_t version;

  if (cc1101_access(dev, CC1101_PARTNUM, &partnum, 1) < 0 ||
      cc1101_access(dev, CC1101_VERSION, &version, 1) < 0)
    {
      return ERROR;
    }
/******************************************************************************/
    return OK;  

	if (partnum == CC1101_PARTNUM_VALUE && version == CC1101_VERSION_VALUE)
	{
		return OK;
	}
	else
	{
	}

  
/******************************************************************************/
  return ERROR;
}

void cc1101_dumpregs(FAR struct cc1101_dev_s *dev, uint8_t addr, uint8_t length)
{
  uint8_t buf[0x30], i;

  cc1101_access(dev, addr, (FAR uint8_t *)buf, length);

  /* REVISIT: printf() should not be used from within the OS */

  spierr("CC1101[%2x]: ", addr);
  for (i = 0; i < length; i++)
    {
      spierr(" %2x,", buf[i]);
    }

  spierr("\n");
}

void cc1101_setpacketctrl(FAR struct cc1101_dev_s *dev)
{
  uint8_t values[3];

  values[0] = 0;      /* Rx FIFO threshold = 32, Tx FIFO threshold = 33 */
  cc1101_access(dev, CC1101_FIFOTHR, values, -1);

  /* Packet length
   * Limit it to 61 bytes in total: pktlen, data[61], rssi, lqi
   */

  values[0] = CC1101_PACKET_MAXDATALEN;
  cc1101_access(dev, CC1101_PKTLEN, values, -1);

  /* Packet Control */

  values[0] = 0x04;   /* Append status: RSSI and LQI at the end of received packet */
                      /* TODO: CRC Auto Flash bit 0x08 ??? */
  values[1] = 0x05;   /* CRC in Rx and Tx Enabled: Variable Packet mode, defined by first byte */
                      /* TODO: Enable data whitening ... */
  cc1101_access(dev, CC1101_PKTCTRL1, values, -2);

  /* Main Radio Control State Machine */

  values[0] = 0x07;   /* No time-out */
  values[1] = 0x00;   /* Clear channel if RSSI < thr && !receiving;
                       * TX -> RX, RX -> RX: 0x3F */
  values[2] = CC1101_MCSM0_VALUE;   /* Calibrate on IDLE -> RX/TX, OSC Timeout = ~500 us
                       * TODO: has XOSC_FORCE_ON */
  cc1101_access(dev, CC1101_MCSM2, values, -3);

  /* Wake-On Radio Control */
  /* Not used yet. */

  /* WOREVT1:WOREVT0 - 16-bit timeout register */
}

#define 	MSG_START		0xAA
#define 	MSG_END			0x55
#define 	CMD_READTIME    0X01
#define     CC1101_SYNCCODE_US  10


extern int GetmsgStartaddrAndLen(char *databuff,int maxlen,int **start_addr);

void modifyTimer_us(uint32_t  timer2_us)
{
    /****************************************************************/
    char 	*kp_data = NULL;
	int 	kmsg_datalen = 0;
	int 	kloop = 0;
	int 	kptr = 0;
	int 	krBytes = 0;

	krBytes = cc1101_rxtx_status.rx_len;
	do
	{
		kmsg_datalen = GetmsgStartaddrAndLen(&cc1101_rxtx_status.rxbuf[kptr],krBytes,&kp_data);
		kptr += kmsg_datalen;
		krBytes -= kmsg_datalen;
		if(MSG_START == kp_data[0])
		{
			switch(kp_data[2])
			{
				case CMD_READTIME:
						{
							_Pcc1101_timemsg_timer_rx = (_cc110x_timemsg *)kp_data;
							_Pcc1101_timemsg_timer_rx->us = (timer2_us - _Pcc1101_timemsg_timer_rx->us - CC1101_SYNCCODE_US);
				        }
					break;
			}
		}					
	}
	while(krBytes >0);			
}

/****************************************************************************
 * Callbacks
 ****************************************************************************/

/* External line triggers this callback
 *
 * The concept todo is:
 *  - GPIO provides EXTI Interrupt
 *  - It should handle EXTI Interrupts in ISR, to which chipcon can
 *    register a callback (and others). The ISR then foreach() calls a
 *    its callback, and it is up to peripheral to find, whether the cause
 *    of EXTI ISR was itself.
 **/
int cc1101_eventcb(int irq, FAR void *context,FAR void *arg)
{
    //FAR struct cc1101_dev_s *cdev = (FAR struct cc1101_dev_s *)arg;
	uint8_t nbytes;
	int status = 0;
	uint8_t crc[2];
	int temp = 0;
	static int crcerror = 0;

	cc1101_timer2_us = *(int*)TIMR2_CNT_ADDR;

	//add by liushuhe 2017.11.30
	if(cc1101_rxtx_status.workmode == CC1101_MODE_RX)
	{
			//add by liushuhe 2018.03.01
			//CC1101_pollnotify(cc1101_fd);	
#if 0
	boardctl(BOARDIOC_TIME2_PPS_UP, 0);
	 int i=0;
	 for(i=0;i<168*10;i++);
	boardctl(BOARDIOC_TIME2_PPS_DOWN, 0);
#endif	 
		cc1101_interrupt++;
		cc1101_access((FAR struct cc1101_dev_s *)arg, CC1101_RXBYTES, &status, 1);
	    if(status&0x7f)
	    {
	    	//*(int*)TIMR1_CNT_ADDR= 0;
			
			cc1101_access((FAR struct cc1101_dev_s *)arg, CC1101_RXFIFO, &nbytes, 1);
	    	//nbytes
			if(nbytes > CC1101_PACKET_MAXTOTALLEN)
			{
				cc1101_rxtx_status.rx_len = 61;
			}
			else
			{
				cc1101_rxtx_status.rx_len_drv = nbytes+2;
				cc1101_rxtx_status.rx_len = cc1101_rxtx_status.rx_len_drv - 3;
			}
			
			cc1101_access((FAR struct cc1101_dev_s *)arg, CC1101_RXFIFO, cc1101_rxtx_status.rxbuf, (cc1101_rxtx_status.rx_len_drv > sizeof(cc1101_rxtx_status.rxbuf)) ? sizeof(cc1101_rxtx_status.rxbuf) : cc1101_rxtx_status.rx_len_drv);	

			if(cc1101_rxtx_status.rxbuf[cc1101_rxtx_status.rx_len_drv-1]&0x80)
			{
				if(cc1101_rxtx_status.rx_len_drv != 61)
				{
					//input 
					modifyTimer_us(cc1101_timer2_us);
					//3:add+2crcbytes
					DatePrint(&cc1101_rxtx_status.rxbuf[1],cc1101_rxtx_status.rx_len); 
					cc1101_rxtx_status.rx_status = SUCCESS;
			        //CC1101_pollnotify(cc1101_fd);	
				}
			}
			else
			{
				crcerror++;
				//spierr("crc<%d>\n",crcerror);
				cc1101_rxtx_status.rx_len = 0;
				cc1101_rxtx_status.rx_status = FAIL;
			}
			//add by liushuhe 2018.03.01
			CC1101_pollnotify(cc1101_fd);	
		}
		else if(status&0x80)
		{
			spierr("buf overflow\n");
		}
		cc1101_strobe((FAR struct cc1101_dev_s *)arg, CC1101_SIDLE);
		cc1101_strobe((FAR struct cc1101_dev_s *)arg, CC1101_SFRX);
		cc1101_strobe((FAR struct cc1101_dev_s *)arg, CC1101_SRX);
	}
	else if(cc1101_rxtx_status.workmode == CC1101_MODE_TX)
	{		
		//wait untill txbyte ok
		cc1101_rxtx_status.tx_status = SUCCESS;
		cc1101_strobe((FAR struct cc1101_dev_s *)arg, CC1101_SIDLE);
		cc1101_strobe((FAR struct cc1101_dev_s *)arg, CC1101_SFRX);
		cc1101_strobe((FAR struct cc1101_dev_s *)arg, CC1101_SRX);
		cc1101_rxtx_status.workmode = CC1101_MODE_RX;
	}
	else
	{
		spierr("error calc\n");
	}
	
	
  return OK;
}


struct cc1101_dev_s *dev_earyinit(struct spi_dev_s *spi, uint8_t isrpin,
    uint32_t pin_int,uint32_t pin_miso,uint32_t pin_mosi,
    const struct c1101_rfsettings_s *rfsettings)
{
  struct cc1101_dev_s *dev;

  ASSERT(spi);

  if ((dev = kmm_malloc(sizeof(struct cc1101_dev_s))) == NULL)
    {
      errno = ENOMEM;
      return NULL;
    }

  dev->rfsettings = rfsettings;
  dev->spi        = spi;
  dev->isrpin     = isrpin;
  dev->pin_int     = pin_int;
  dev->pin_miso   = pin_miso;
  dev->pin_mosi   = pin_mosi;

  return dev;
}
/****************************************************************************
 * Public Functions
 ****************************************************************************/

int cc1101_init(FAR struct cc1101_dev_s *dev)
{

  ASSERT(dev->spi);

  if (cc1101_reset(dev) < 0)
    {
      kmm_free(dev);
      errno = EFAULT;

      return -errno;
    }


  if (cc1101_checkpart(dev) < 0)
    {
      kmm_free(dev);
      errno = ENODEV;
      return -errno;
    }

  
  cc1101_setrf(dev, dev->rfsettings);

  //add by liushuhe 2017.11.19
  //register isr pin  pin_int=GPIO_GDO2_C1100
  stm32_configgpio(dev->pin_int);
  //stm32_gpiosetevent(dev->pin_int, true, false, true, cc1101_eventcb, dev); 
  stm32_gpiosetevent(dev->pin_int, false, true, true, cc1101_eventcb, dev); 

  read_cc1101_setrf(dev, dev->rfsettings);
  
  cc1101_strobe(dev, CC1101_SIDLE);
  cc1101_strobe(dev, CC1101_SFRX);
  cc1101_receive(dev);


  cc1101_interrupt = 0;
  return 1;
}

int cc1101_deinit(FAR struct cc1101_dev_s *dev)
{
  ASSERT(dev);

  /* Release the external GPIO interrupt
   *
   * REVISIT:  There is no MCU-independent way to do this in this
   * context.
   */

  /* Power down chip */

  cc1101_powerdown(dev);

  /* Release external interrupt line */

  kmm_free(dev);
  return 0;
}

int cc1101_powerup(FAR struct cc1101_dev_s *dev)
{
  ASSERT(dev);
  return 0;
}

int cc1101_powerdown(FAR struct cc1101_dev_s *dev)
{
  ASSERT(dev);
  return 0;
}

int cc1101_setgdo(FAR struct cc1101_dev_s *dev, uint8_t pin, uint8_t function)
{
  ASSERT(dev);
  ASSERT(pin <= CC1101_IOCFG0);

  if (function >= CC1101_GDO_CLK_XOSC1)
    {
      /* Only one pin can be enabled at a time as XOSC/n */

      if (dev->flags & FLAGS_XOSCENABLED)
        {
          return -EPERM;
        }

      /* Force XOSC to stay active even in sleep mode */

      int value = CC1101_MCSM0_VALUE | CC1101_MCSM0_XOSC_FORCE_ON;
      cc1101_access(dev, CC1101_MCSM0, (FAR uint8_t *)&value, -1);

      dev->flags |= FLAGS_XOSCENABLED;
    }
  else if (dev->flags & FLAGS_XOSCENABLED)
    {
      /* Disable XOSC in sleep mode */

      int value = CC1101_MCSM0_VALUE;
      cc1101_access(dev, CC1101_MCSM0, (FAR uint8_t *)&value, -1);

      dev->flags &= ~FLAGS_XOSCENABLED;
    }

  return cc1101_access(dev, pin, &function, -1);
}


int cc1101_setrf(FAR struct cc1101_dev_s *dev, const struct c1101_rfsettings_s *settings)
{
	
	ASSERT(dev);
	ASSERT(settings);


    if(cc1101_access(dev, CC1101_IOCFG2, (FAR uint8_t *)&settings->IOCFG2, -39) < 0)
    //if(cc1101_access(dev, CC1101_IOCFG2, (FAR uint8_t *)&settings->IOCFG2, -47) < 0)
    {
		return ERROR;
    }


    //if(cc1101_access(dev, CC1101_FSTEST, (FAR uint8_t *)&settings->FSTEST, -1) < 0)
    {
	//	return ERROR;
    }

    //if(cc1101_access(dev, CC1101_TEST2, (FAR uint8_t *)&settings->TEST2, -3) < 0)
    {
	//	return ERROR;
    }

  // Load Power Table 
	if(cc1101_access(dev, CC1101_PATABLE, (FAR uint8_t *)PA_table, -8) < 0)
    {
      return ERROR;
    }
	
    cc1101_strobe(dev, CC1101_SFRX);
    cc1101_strobe(dev, CC1101_SFTX);
    cc1101_strobe(dev, CC1101_SIDLE);
	
}


int read_cc1101_setrf(FAR struct cc1101_dev_s *dev, const struct c1101_rfsettings_s *settings)
{
	int status=0xff;
	
	ASSERT(dev);
	ASSERT(settings);

	cc1101_access(dev, CC1101_IOCFG2, (FAR uint8_t *)&status, 1);
    spierr("CC1101_IOCFG2=%x\n",status);
	cc1101_access(dev, CC1101_IOCFG1, (FAR uint8_t *)&status, 1);
    spierr("CC1101_IOCFG1=%x\n",status);
	cc1101_access(dev, CC1101_IOCFG0, (FAR uint8_t *)&status, 1);
    spierr("CC1101_IOCFG0=%x\n",status);
	cc1101_access(dev, CC1101_FIFOTHR, (FAR uint8_t *)&status, 1);
    spierr("CC1101_FIFOTHR=%x\n",status);
	cc1101_access(dev, CC1101_SYNC1, (FAR uint8_t *)&status, 1);
    spierr("CC1101_SYNC1=%x\n",status);
	cc1101_access(dev, CC1101_SYNC0, (FAR uint8_t *)&status, 1);
    spierr("CC1101_SYNC0=%x\n",status);
	cc1101_access(dev, CC1101_PKTLEN, (FAR uint8_t *)&status, 1);
    spierr("CC1101_PKTLEN=%x\n",status);
	cc1101_access(dev, CC1101_PKTCTRL1, (FAR uint8_t *)&status, 1);
    spierr("CC1101_PKTCTRL1=%x\n",status);
	cc1101_access(dev, CC1101_PKTCTRL0, (FAR uint8_t *)&status, 1);
    spierr("CC1101_PKTCTRL0=%x\n",status);
	cc1101_access(dev, CC1101_ADDR, (FAR uint8_t *)&status, 1);
    spierr("CC1101_ADDR=%x\n",status);
	cc1101_access(dev, CC1101_CHANNR, (FAR uint8_t *)&status, 1);
    spierr("CC1101_CHANNR=%x\n",status);
	cc1101_access(dev, CC1101_FSCTRL1, (FAR uint8_t *)&status, 1);
    spierr("CC1101_FSCTRL1=%x\n",status);
	cc1101_access(dev, CC1101_FSCTRL0, (FAR uint8_t *)&status, 1);
    spierr("CC1101_FSCTRL1=%x\n",status);
	cc1101_access(dev, CC1101_FREQ2, (FAR uint8_t *)&status, 1);
    spierr("CC1101_FREQ2=%x\n",status);
	cc1101_access(dev, CC1101_FREQ1, (FAR uint8_t *)&status, 1);
    spierr("CC1101_FREQ1=%x\n",status);
	cc1101_access(dev, CC1101_FREQ0, (FAR uint8_t *)&status, 1);
    spierr("CC1101_FREQ0=%x\n",status);
	cc1101_access(dev, CC1101_MDMCFG4, (FAR uint8_t *)&status, 1);
    spierr("CC1101_MDMCFG4=%x\n",status);
	cc1101_access(dev, CC1101_MDMCFG3, (FAR uint8_t *)&status, 1);
    spierr("CC1101_MDMCFG3=%x\n",status);
	cc1101_access(dev, CC1101_MDMCFG2, (FAR uint8_t *)&status, 1);
    spierr("CC1101_MDMCFG2=%x\n",status);
	cc1101_access(dev, CC1101_MDMCFG1, (FAR uint8_t *)&status, 1);
    spierr("CC1101_MDMCFG1=%x\n",status);
	cc1101_access(dev, CC1101_MDMCFG0, (FAR uint8_t *)&status, 1);
    spierr("CC1101_MDMCFG0=%x\n",status);
	cc1101_access(dev, CC1101_DEVIATN, (FAR uint8_t *)&status, 1);
    spierr("CC1101_DEVIATN=%x\n",status);
	cc1101_access(dev, CC1101_MCSM2, (FAR uint8_t *)&status, 1);
    spierr("CC1101_MCSM2=%x\n",status);
	cc1101_access(dev, CC1101_MCSM1, (FAR uint8_t *)&status, 1);
    spierr("CC1101_MCSM1=%x\n",status);
	cc1101_access(dev, CC1101_MCSM0, (FAR uint8_t *)&status, 1);
    spierr("CC1101_MCSM0=%x\n",status);
	cc1101_access(dev, CC1101_FOCCFG, (FAR uint8_t *)&status, 1);
    spierr("CC1101_FOCCFG=%x\n",status);
	cc1101_access(dev, CC1101_BSCFG, (FAR uint8_t *)&status, 1);
    spierr("CC1101_BSCFG=%x\n",status);
	cc1101_access(dev, CC1101_AGCCTRL2, (FAR uint8_t *)&status, 1);
    spierr("CC1101_AGCCTRL2=%x\n",status);
	cc1101_access(dev, CC1101_AGCCTRL1, (FAR uint8_t *)&status, 1);
    spierr("CC1101_AGCCTRL1=%x\n",status);
	cc1101_access(dev, CC1101_AGCCTRL0, (FAR uint8_t *)&status, 1);
    spierr("CC1101_AGCCTRL0=%x\n",status);
	cc1101_access(dev, CC1101_WOREVT1, (FAR uint8_t *)&status, 1);
    spierr("CC1101_WOREVT1=%x\n",status);
	cc1101_access(dev, CC1101_WOREVT0, (FAR uint8_t *)&status, 1);
    spierr("CC1101_WOREVT0=%x\n",status);
    cc1101_access(dev, CC1101_WORCTRL, (FAR uint8_t *)&status, 1);
    spierr("CC1101_WORCTRL=%x\n",status);
	cc1101_access(dev, CC1101_FREND1, (FAR uint8_t *)&status, 1);
    spierr("CC1101_FREND1=%x\n",status);
	cc1101_access(dev, CC1101_FREND0, (FAR uint8_t *)&status, 1);
    spierr("CC1101_FREND0=%x\n",status);
	cc1101_access(dev, CC1101_FSCAL3, (FAR uint8_t *)&status, 1);
    spierr("CC1101_FSCAL3=%x\n",status);
	cc1101_access(dev, CC1101_FSCAL2, (FAR uint8_t *)&status, 1);
    spierr("CC1101_FSCAL2=%x\n",status);
	cc1101_access(dev, CC1101_FSCAL1, (FAR uint8_t *)&status, 1);
    spierr("CC1101_FSCAL1=%x\n",status);
	cc1101_access(dev, CC1101_FSCAL0, (FAR uint8_t *)&status, 1);
    spierr("CC1101_FSCAL0=%x\n",status);
	cc1101_access(dev, CC1101_RCCTRL1, (FAR uint8_t *)&status, 1);
    spierr("CC1101_RCCTRL1=%x\n",status);
	cc1101_access(dev, CC1101_RCCTRL0, (FAR uint8_t *)&status, 1);
    spierr("CC1101_RCCTRL0=%x\n",status);
    cc1101_access(dev, CC1101_FSTEST, (FAR uint8_t *)&status, 1);
    spierr("CC1101_FSTEST=%x\n",status);
    cc1101_access(dev, CC1101_PTEST, (FAR uint8_t *)&status, 1);
    spierr("CC1101_PTEST=%x\n",status);
    cc1101_access(dev, CC1101_AGCTEST, (FAR uint8_t *)&status, 1);
    spierr("CC1101_AGCTEST=%x\n",status);
	cc1101_access(dev, CC1101_TEST2, (FAR uint8_t *)&status, 1);
    spierr("CC1101_TEST2=%x\n",status);
    cc1101_access(dev, CC1101_TEST1,(FAR uint8_t *)&status, 1);
    spierr("CC1101_TEST1=%x\n",status);
	cc1101_access(dev, CC1101_TEST0, (FAR uint8_t *)&status, 1);
    spierr("CC1101_TEST0=%x\n",status);

	return 1;
}


int cc1101_calcRSSIdBm(int rssi)
{
  if (rssi >= 128)
    {
      rssi -= 256;
    }

  return (rssi >> 1) - 74;
}

int cc1101_receive(FAR struct cc1101_dev_s *dev)
{
	ASSERT(dev);

	//cc1101_strobe(dev, CC1101_SIDLE);
	
	//cc1101_strobe(dev, CC1101_SFRX);
	cc1101_strobe(dev, CC1101_SRX);
	cc1101_rxtx_status.workmode = CC1101_MODE_RX;

#if 0
	//add by liushuhe 2018.01.04
	cc1101_rxtx_status.workmode = CC1101_MODE_RX;
	rfSettings.IOCFG2 = CC1101_GDO2_RX;
    if(cc1101_access(dev, CC1101_IOCFG2, (FAR uint8_t *)&rfSettings.IOCFG2, -1) < 0)
    {
		spierr("cc1101 Rx gdo init error\n");
    }
	rfSettings.FIFOTHR = CC1101_THER_RX;
    if(cc1101_access(dev, CC1101_FIFOTHR, (FAR uint8_t *)&rfSettings.FIFOTHR, -1) < 0)
    {
		spierr("cc1101 Rx FIFOTHR init error\n");
    }
#endif

	return 0;
}

int cc1101_sendmode(FAR struct cc1101_dev_s *dev)
{
	ASSERT(dev);

    cc1101_rxtx_status.workmode = CC1101_MODE_TX;
	cc1101_rxtx_status.tx_status = FAIL;

#if 0

	cc1101_strobe(dev, CC1101_SIDLE);
	cc1101_strobe(dev, CC1101_SFTX);



	rfSettings.ADDR = _Pcc1101_timemsg_tx->src;
    if(cc1101_access(dev, CC1101_ADDR, (FAR uint8_t *)&rfSettings.ADDR, -1) < 0)
    {
		spierr("cc1101 tx ADDR init error\n");
    }


	rfSettings.IOCFG2 = CC1101_GDO2_TX;
    if(cc1101_access(dev, CC1101_IOCFG2, (FAR uint8_t *)&rfSettings.IOCFG2, -1) < 0)
    {
		spierr("cc1101 tx gdo init error\n");
    }
	rfSettings.FIFOTHR = CC1101_THER_TX;
    if(cc1101_access(dev, CC1101_FIFOTHR, (FAR uint8_t *)&rfSettings.FIFOTHR, -1) < 0)
    {
		spierr("cc1101 tx FIFOTHR init error\n");
    }
#endif
	return 0;
}



int cc1101_read(FAR struct cc1101_dev_s *dev, uint8_t * buf, size_t size)
{
	char cVal;
	int i=0;

	ASSERT(dev);

	/* 把环形缓冲区的数据取出来, 最多取255字节 */
	while ((i < size) && (0 == GetData(&cVal)))
	{
		buf[i] = cVal;
		i++;
	}
			
	return i;
	
}

int cc1101_write(FAR struct cc1101_dev_s *dev, const uint8_t *buf, size_t size)
{
	uint8_t packetlen;
	int ret;
	uint8_t ttttt = 0;
	int txbyte;

	ASSERT(dev);
	ASSERT(buf);

	/* Present limit */

	if (size > CC1101_PACKET_MAXDATALEN)
	{
	  packetlen = CC1101_PACKET_MAXDATALEN;
	}
	else
	{
	  packetlen = size;
	}
	//add by liushuhe 2018.03.02

	//goto tx mode	
    cc1101_rxtx_status.workmode  = CC1101_MODE_TX;
	cc1101_rxtx_status.tx_status = FAIL;

	cc1101_strobe(dev, CC1101_SIDLE);
	cc1101_strobe(dev, CC1101_SFTX);
	
	//len
	uint8_t len = 0;
	len = packetlen + 1;
	cc1101_access(dev, CC1101_TXFIFO, &len, -1);
	
    //addr
    uint8_t addr = 0;
	addr = _Pcc1101_timemsg_tx->dist;
	cc1101_access(dev, CC1101_TXFIFO, &addr, -1);
	
	//data
	ret = cc1101_access(dev, CC1101_TXFIFO, (FAR uint8_t *)buf, -(packetlen));
	cc1101_rxtx_status.tx_len = packetlen;
	
	return ret;
}

int cc1101_send(FAR struct cc1101_dev_s *dev)
{
	char bytes=0;
	
	
	ASSERT(dev);

	cc1101_strobe(dev, CC1101_STX);

	//wait untill txbyte ok
	do
	{
		cc1101_access(dev, CC1101_TXBYTES, &bytes, 1);
	}while((bytes &0x7F) != 0);

    //wait send ok
    //while(stm32_gpioread(dev->pin_miso));
	int i=0;
	for(i=0;i<168*10;i++);

	return cc1101_rxtx_status.tx_len;
}

int cc1101_idle(FAR struct cc1101_dev_s *dev)
{
  ASSERT(dev);
  cc1101_strobe(dev, CC1101_SIDLE);
  return 0;
}


/*********************************************************************************/
//add by liushuhe 2017.11.30
static int fs_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct cc1101_upperhalf_s *upper = inode->i_private;
  FAR struct cc1101_lowerhalf_s *lower = upper->dev;

  uint8_t tmp;
  int ret;

  /* Get exclusive access to the device structures */

  ret = sem_wait(&upper->exclsem);
  if (ret < 0)
    {
      ret = -errno;
      goto errout;
    }

  /* Increment the count of references to the device.  If this the first
   * time that the driver has been opened for this device, then initialize
   * the device.
   */

  tmp = upper->crefs + 1;
  //add by liushuhe 2017.12.02
  ret = lower->ops->init(lower->c1101_dev);
  
  if (tmp == 0)
    {
      /* More than 255 opens; uint8_t overflows to zero */

      ret = -EMFILE;
      goto errout_with_sem;
    }

  /* Save the new open count on success */

  upper->crefs = tmp;
  ret = OK;

errout_with_sem:
  sem_post(&upper->exclsem);

errout:
  return ret;
}

/*********************************************************************************/
//add by liushuhe 2017.11.30
static int fs_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct cc1101_upperhalf_s *upper = inode->i_private;
  int ret;


  /* Get exclusive access to the device structures */

  ret = sem_wait(&upper->exclsem);
  if (ret < 0)
    {
      ret = -errno;
      goto errout;
    }

  /* Decrement the references to the driver.  If the reference count will
   * decrement to 0, then uninitialize the driver.
   */

  if (upper->crefs > 1)
    {
      upper->crefs--;
	  
    }
  else
    {
      //FAR struct cc1101_lowerhalf_s *lower = upper->dev;

      /* There are no more references to the port */

      upper->crefs = 0;
    }
  ret = OK;

//errout_with_sem:
  sem_post(&upper->exclsem);

errout:
  return ret;
}

/*********************************************************************************/
//add by liushuhe 2017.11.30
static ssize_t fs_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
	ssize_t ret;
	FAR struct inode *inode = filep->f_inode;
	FAR struct cc1101_upperhalf_s *upper = inode->i_private;
	FAR struct cc1101_lowerhalf_s *lower = upper->dev;

	/* TODO: Should we check permissions here? */
	/* Audio read operations get passed directly to the lower-level */

	if (lower->ops->read != NULL)
	{
		ret = lower->ops->read(lower->c1101_dev, (_uint8_t *)buffer, buflen);
	}
	
	cc1101_interrupt -= ret;
	//auto goto rcv mode
	
	return ret;
}

/*********************************************************************************/
//add by liushuhe 2017.11.30
static ssize_t fs_write(FAR struct file *filep, FAR const char *buffer, size_t buflen)
{
	ssize_t ret_w = 0; 
	ssize_t ret_s = 0; 
	FAR struct inode *inode = filep->f_inode;
	FAR struct cc1101_upperhalf_s *upper = inode->i_private;
	FAR struct cc1101_lowerhalf_s *lower = upper->dev;
	
	_Pcc1101_timemsg_tx = (_cc110x_timemsg *)buffer;

	//cc1101_sendmode(lower->c1101_dev);
	
	//set data to fifo
	if (lower->ops->write != NULL)
	{
		ret_w = lower->ops->write(lower->c1101_dev, (_uint8_t *)buffer, buflen);
	}
	
	//start tx
	if (lower->ops->setmode_send != NULL)
	{
		ret_s=lower->ops->setmode_send(lower->c1101_dev);
	}
	
	return ret_s;
}

/*********************************************************************************/
//add by liushuhe 2018.01.08
static int fs_poll(FAR struct file *filep, FAR struct pollfd *fds,bool setup)
{
  FAR struct inode *inode;
  FAR struct cc1101_upperhalf_s *priv;
  int ret;
  int i;

  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct cc1101_upperhalf_s *)inode->i_private;
  /* Get exclusive access */
  do
    {
      ret = nxsem_wait(&priv->devsem);
      /* The only case that an error should occur here is if the wait was
       * awakened by a signal.
       */

      DEBUGASSERT(ret == OK || ret == -EINTR);
    }
  while (ret == -EINTR);
  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto out;
        }
      /* This is a request to set up the poll.  Find an available slot for
       * the poll structure reference.
       */

      for (i = 0; i < CONFIG_CC1101_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!priv->fds[i])
            {
              /* Bind the poll structure and this slot */
              priv->fds[i] = fds;
              fds->priv = &priv->fds[i];
              break;
            }
        }
      if (i >= CONFIG_CC1101_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret = -EBUSY;
          goto out;
        }
        if(cc1101_buf.g_iReadPos != cc1101_buf.g_iWritePos)
        {
            //CC1101_pollnotify(priv);
        }
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */
      struct pollfd **slot = (struct pollfd **)fds->priv;
      DEBUGASSERT(slot != NULL);
      /* Remove all memory of the poll setup */

      *slot = NULL;
      fds->priv = NULL;
    }
out:
  nxsem_post(&priv->devsem);
  return ret;
}


static int fs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  int ret = -1;
  uint32_t bytes = 0;
  uint32_t r_ptr = cc1101_buf.g_iReadPos;
  uint32_t w_ptr = cc1101_buf.g_iWritePos;
 
  
  switch (cmd)
    {
      case GETCC1101BUF_BYTES:
        {
          uint32_t* ptr = (uint32_t*)((uintptr_t)arg);

		  if(w_ptr != r_ptr)
		  {
			  do
			  {
				  r_ptr = (r_ptr + 1) % CC1101_BUF_SIZE;
				  bytes++;
			  }
			  while(w_ptr != r_ptr);
			  
			  *ptr = bytes; 	  
		  }
		  else
		  {
			  *ptr = 0; 
		  }
		  
          ret = bytes;
        }
        break;
		
      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}



/*********************************************************************************/
//add by liushuhe 2017.11.30
FAR struct cc1101_lowerhalf_s * dev_lower_init(FAR struct cc1101_dev_s *dev,struct cc1101_operations_s *c1101_ops)
{
	FAR struct cc1101_lowerhalf_s *lower;
	lower = (FAR struct cc1101_lowerhalf_s *)kmm_zalloc(sizeof(struct cc1101_lowerhalf_s));
	if (lower == NULL)
	{
		return NULL;
	}
	
	lower->ops 		 = c1101_ops; 		
	lower->c1101_dev = dev;
  return lower;
}
/*********************************************************************************/
//add by liushuhe 2017.11.30
int cc1101_register(struct cc1101_lowerhalf_s *dev)
{
  FAR struct cc1101_upperhalf_s *upper;
  
  /* Allocate the upper-half data structure */

	cc1101_buf.g_Buf = kmm_zalloc(CC1101_BUF_SIZE);

	
    cc1101_buf.g_iReadPos  = 0;
    cc1101_buf.g_iWritePos = 0;
		
  upper = (FAR struct cc1101_upperhalf_s *)kmm_zalloc(sizeof(struct cc1101_upperhalf_s));
  if (upper == NULL)
    {
      return -ENOMEM;
    }

  
  cc1101_fd = upper;
 
  /* Initialize the Audio device structure (it was already zeroed by kmm_zalloc()) */
  nxsem_init(&upper->devsem, 0, 1);
  sem_init(&upper->exclsem, 0, 1);
  upper->dev = dev;

  return register_driver("/dev/cc1101", &g_cc1101_drvrops, 0666, upper);
}

/*********************************************************************************/
//add by liushuhe 2017.11.30
int  cc1101_auto_register(FAR struct spi_dev_s *spi,uint32_t pin_int,uint32_t pin_miso,uint32_t pin_mosi)
{
	int ret; 
	FAR struct cc1101_dev_s 		* cc1101_dev;
	FAR struct cc1101_lowerhalf_s 	* cc1101_lowerhalf;
	
	cc1101_dev = dev_earyinit(spi, CC1101_PIN_GDO2,pin_int,pin_miso,pin_mosi,&rfSettings);
	if(cc1101_dev == NULL)
	{
		spierr("ERROR:cc1101_dev Allocation failed\n");
		return 0;
	}

	cc1101_lowerhalf = dev_lower_init(cc1101_dev,&g_cc1101_devops);
	if(cc1101_dev == NULL)
	{
		spierr("ERROR:cc1101_lowerhalf Allocation failed\n");
		return 0;
	}
	
	ret = cc1101_register(cc1101_lowerhalf);
	if(ret !=0)
	{
		spierr("ERROR:cc1101_register  failed\n");
		return 0;
	}
	else
	{
		return 1;
	}
	
}




