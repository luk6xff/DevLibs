/**
 *   @brief nRF24L01+ The Nordic nRF24L01+ is a highly integrated, ultra low power (ULP) 2Mbps RF transceiver IC 
 *	 for the 2.4GHz ISM (Industrial, Scientific and Medical) band -  Breakout SPI Library  
 *           
 *   @Author lukasz uszko(luszko@op.pl)
 *
 *   Tested on FRDM-KL46Z/FRDM-KL25Z/STM32L/STM32M uC
 * 
 *   Copyright (c) 2014 lukasz uszko
 *   Released under the MIT License (see http://mbed.org/license/mit)
 *
 *   Documentation regarding the nRF24L01+ might be found here: 
 *   https://www.nordicsemi.com/kor/Products/2.4GHz-RF/nRF24L01P
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __NRF24L01+_H
#define __NRF24L01+_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "stdbool.h"
#include "string.h"

typedef unsigned char BYTE;
//**************************************************************************************************************************//
// SPI(nRF24L01) COMMANDS

#define R_REGISTER 0x00  		 //read command and status register (0b000AAAAAA) where AAAAA = 5 bit Register Map Address
#define W_REGISTER 0x20          //Write command and status registers.LSB AAAAA = 5 bit Register Map Address
#define R_RX_PAYLOAD 0x61        //Read RX-payload: 1 – 32 bytes. A read operation always starts at byte 0. Payload is deleted from FIFO after it is read. Used in RX mode
#define W_TX_PAYLOAD 0xA0 	     //Write TX-payload: 1 – 32 bytes. A write operation always starts at byte 0 used in TX payload.
#define FLUSH_TX 0xE1     		 //Flush TX FIFO, used in TX mode
#define FLUSH_RX 0xE2    		 //Flush RX FIFO, used in RX mode
#define REUSE_TX_PL 0xE3 		 //Used for a PTX deviceReuse last transmitted payload.TX payload reuse is active until W_TX_PAYLOAD or FLUSH TX is executed.
#define R_RX_PL_WID 0x60 		 //Read RX payload width for the top R_RX_PAYLOAD in the RX FIFO. Note: Flush RX FIFO if the read value is larger than 32 bytes.
#define W_ACK_PAYLOAD 0xA8 		 //(0b1010 1PPP)Used in RX mode. Write Payload to be transmitted together with ACK packet on PIPE PPP. (PPP valid in the range from 000 to 101).
#define W_TX_PAYLOAD_NO_ACK 0xB0 //Used in TX mode. Disables AUTOACK on this specific packet.
#define NOP 0xFF                 //No Operation. Might be used to read the STATUS register
//**************************************************************************************************************************//
//CONSTANTS - you can change it depending on your needs
#define NUMBER_OF_BYTES_IN_FIFO 10 			// default value of how many bytes you can send/receive to/from the FIFO (the same value ought to be set in RX_PW_Px registers )
#define DATA_PIPE_LENGTH 5 						//the same value has to be set up in SETUP_AW REGISTER
static const BYTE RX_ADDR_P0[DATA_PIPE_LENGTH] =
		{ 0x35, 0x36, 0x37, 0x38, 0x01 }; 		//0x0A  // write the number of bytes defined in SETUP_AW (AW) // LSB written first
static const BYTE RX_ADDR_P1[DATA_PIPE_LENGTH] =
		{ 0x15, 0x16, 0x17, 0x18, 0x01 }; 		//0x0B
static const BYTE RX_ADDR_P2[] = { 0xA3 };
static const BYTE RX_ADDR_P3[] = { 0xA4 };
static const BYTE RX_ADDR_P4[] = { 0xA5 };
static const BYTE RX_ADDR_P5[] = { 0xA6 };
static const BYTE TX_ADDR[DATA_PIPE_LENGTH] = { 0x35, 0x36, 0x37, 0x38, 0x01 }; //0x10 // write the number of bytes defined in SETUP_AW (AW) // LSB written first
//**************************************************************************************************************************//
//OTHERS
#define SET_ 1
#define RESET_ 0

//**************************************************************************************************************************//
//SPI(nRF24L01) COMMANDS
typedef enum {
	READ = R_REGISTER, //read command and status register (0b000AAAAAA) where AAAAA = 5 bit Register Map Address
	WRITE = W_REGISTER, //Write command and status registers.LSB AAAAA = 5 bit Register Map Address
	READ_PAYLOAD = R_RX_PAYLOAD, //Read RX-payload: 1 – 32 bytes. A read operation always starts at byte 0. Payload is deleted from FIFO after it is read. Used in RX mode
	WRITE_PAYLOAD = W_TX_PAYLOAD, //Write TX-payload: 1 – 32 bytes. A write operation always starts at byte 0 used in TX payload.
	FLUSH__TX = FLUSH_TX, //Flush TX FIFO, used in TX mode
	FLUSH__RX = FLUSH_RX, //Flush RX FIFO, used in RX mode
	REUSE_LAST_TX_PAYLOAD = REUSE_TX_PL, //Used for a PTX deviceReuse last transmitted payload.TX payload reuse is active until W_TX_PAYLOAD or FLUSH TX is executed.
	READ_RX_PAYLOAD_WIDTH = R_RX_PL_WID, //Read RX payload width for the top R_RX_PAYLOAD in the RX FIFO. Note: Flush RX FIFO if the read value is larger than 32 bytes.
	WRITE_PAYLOAD_WITH_ACK = W_ACK_PAYLOAD, //(0b1010 1PPP)Used in RX mode. Write Payload to be transmitted together with ACK packet on PIPE PPP. (PPP valid in the range from 000 to 101).
	DISABLE_AUTO_ACK = W_TX_PAYLOAD_NO_ACK, //Used in TX mode. Disables AUTOACK on this specific packet.
	DO_NOTHING = NOP
} _SPI_COMMAND_TypeDef;

//**************************************************************************************************************************//
// SPI(nRF24L01) ADRESSES OF THE ALL REGISTERS OF THE nRF24L01+
typedef enum {
	CONFIG_regAdr = 0x00,
	EN_AA_ADR_regAdr,
	EN_RXADDR_regAdr,
	SETUP_AW_regAdr,
	SETUP_RETR_regAdr,
	RF_CH_regAdr,
	RF_SETUP_regAdr,
	STATUS_regAdr,
	OBSERVE_TX_regAdr,
	RPD_regAdr,
	RX_ADDR_P0_regAdr,
	RX_ADDR_P1_regAdr,
	RX_ADDR_P2_regAdr,
	RX_ADDR_P3_regAdr,
	RX_ADDR_P4_regAdr,
	RX_ADDR_P5_regAdr,
	TX_ADDR_regAdr,
	RX_PW_P0_regAdr,
	RX_PW_P1_regAdr,
	RX_PW_P2_regAdr,
	RX_PW_P3_regAdr,
	RX_PW_P4_regAdr,
	RX_PW_P5_regAdr,
	FIFO_STATUS_regAdr,
	DYNPD_regAdr = 0x1C,
	FEATURE_regAdr
} _SPI_REGISTER_ADRESS_TypeDef;

//**************************************************************************************************************************//
//  CONTENT OF ALL REGISTERS OF THE nRF24L01+

typedef struct /*__attribute__ ((__packed__))*/{

	BYTE PRIM_RX :1;
	BYTE PWR_UP :1;
	BYTE CRCO :1;
	BYTE EN_CRC :1;
	BYTE MASK_MAX_RT :1;
	BYTE MASK_TX_DS :1;
	BYTE MASK_RX_DR :1;
	const BYTE RESERVED :1;

} CONFIG_TypeDef; //0x00

////////////////////////////////////////////////////////////////////////////////
typedef struct {

	BYTE ENAA_P0 :1;
	BYTE ENAA_P1 :1;
	BYTE ENAA_P2 :1;
	BYTE ENAA_P3 :1;
	BYTE ENAA_P4 :1;
	BYTE ENAA_P5 :1;
	const BYTE RESERVED :2;

} EN_AA_TypeDef; //0x01

////////////////////////////////////////////////////////////////////////////////

typedef struct {

	BYTE ERX_P0 :1;
	BYTE ERX__P1 :1;
	BYTE ERX__P2 :1;
	BYTE ERX__P3 :1;
	BYTE ERX__P4 :1;
	BYTE ERX_ :1;
	const BYTE RESERVED :2;

} EN_RXADDR_TypeDef; //0x02

////////////////////////////////////////////////////////////////////////////////

typedef enum {

	_3BYTES = 0x01, _4BYTES, _5BYTES

} _AW_TypeDef;

typedef struct {

	_AW_TypeDef AW :2;
	const BYTE RESERVED :6;

} SETUP_AW_TypeDef; //0x03

////////////////////////////////////////////////////////////////////////////////

typedef enum {
	_DISABLED,
	_1_RETRANSMIT,
	_2_RETRANSMIT,
	_3_RETRANSMIT,
	_4_RETRANSMIT,
	_5_RETRANSMIT,
	_6_RETRANSMIT,
	_7_RETRANSMIT,
	_8_RETRANSMIT,
	_9_RETRANSMIT,
	_10_RETRANSMIT,
	_11_RETRANSMIT,
	_12_RETRANSMIT,
	_13_RETRANSMIT,
	_14_RETRANSMIT,
	_15_RETRANSMIT
} _ARC_TypeDef;

typedef enum {
	_250us,
	_500us,
	_750us,
	_1000us,
	_1250us,
	_1500us,
	_1750us,
	_2000us,
	_2250us,
	_2500us,
	_2750us,
	_3000us,
	_3250us,
	_3500us,
	_3750us,
	_4000us
} _ARD_TypeDef;

typedef struct {
	_ARC_TypeDef ARC :4;
	_ARD_TypeDef ARD :4;
} SETUP_RETR_TypeDef; //0x04

////////////////////////////////////////////////////////////////////////////////

typedef struct {
	BYTE RF_CH :7;
	const BYTE RESERVED :1;
} RF_CH_TypeDef; //0x05

////////////////////////////////////////////////////////////////////////////////

/*[RF_DR_LOW, RF_DR_HIGH]:
 ‘00’ – 1Mbps
 ‘01’ – 2Mbps
 ‘10’ – 250kbps
 ‘11’ – Reserved
 */
typedef enum {
	_18dBm, _12dBm, _6dBm, _0dBm
} _RF_PWR_TypeDef;

typedef struct {
	const BYTE RESERVED0 :1;
	_RF_PWR_TypeDef RF_PWR :2;
	BYTE RF_DR_HIGH :1;
	BYTE PLL_LOCK :1;
	BYTE RF_DR_LOW :1;
	const BYTE RESERVED1 :1;
	BYTE CONT_WAVE :1;
} RF_SETUP_TypeDef; //0x06

////////////////////////////////////////////////////////////////////////////////

typedef struct {
	BYTE TX_FULL :1;
	BYTE RX_P_NO :3;
	BYTE MAX_RT :1;
	BYTE TX_DS :1;
	BYTE RX_DR :1;
	const BYTE RESERVED1 :1;
} STATUS_TypeDef; //0x07

////////////////////////////////////////////////////////////////////////////////

typedef struct {
	BYTE ARC_CNT :4; //page 75 datasheet
	BYTE PLOS_CNT :4;
} OBSERVE_TX_TypeDef; //0x08

////////////////////////////////////////////////////////////////////////////////

typedef struct {
	BYTE RPD :1;
	const BYTE RESERVED1 :7;
} RPD_TypeDef; //0x09

////////////////////////////////////////////////////////////////////////////////
typedef enum {
	PIPE_NOT_USED,
	_1_Byte,
	_2_Bytes,
	_3_Bytes,
	_4_Bytes,
	_5_Bytes,
	_6_Bytes,
	_7_Bytes,
	_8_Bytes,
	_9_Bytes,
	_10_Bytes,
	_11_Bytes,
	_12_Bytes,
	_13_Bytes,
	_14_Bytes,
	_15_Bytes,
	_16_Bytes,
	_17_Bytes,
	_18_Bytes,
	_19_Bytes,
	_20_Bytes,
	_21_Bytes,
	_22_Bytes,
	_23_Bytes,
	_24_Bytes,
	_25_Bytes,
	_26_Bytes,
	_27_Bytes,
	_28_Bytes,
	_29_Bytes,
	_30_Bytes,
	_31_Bytes,
	_32_Bytes,
} _RX_PW_TypeDef;

typedef struct {
	_RX_PW_TypeDef RX_PW_P0 :6;
	const BYTE RESERVED :2;
} RX_PW_P0_TypeDef; //0x11

typedef struct {
	_RX_PW_TypeDef RX_PW_P1 :6;
	const BYTE RESERVED :2;
} RX_PW_P1_TypeDef; //0x12

typedef struct {
	_RX_PW_TypeDef RX_PW_P2 :6;
	const BYTE RESERVED :2;
} RX_PW_P2_TypeDef; //0x13

typedef struct {
	_RX_PW_TypeDef RX_PW_P3 :6;
	const BYTE RESERVED :2;
} RX_PW_P3_TypeDef; //0x14

typedef struct {
	_RX_PW_TypeDef RX_PW_P4 :6;
	const BYTE RESERVED :2;
} RX_PW_P4_TypeDef; //0x15

typedef struct {
	_RX_PW_TypeDef RX_PW_P5 :6;
	const BYTE RESERVED :2;
} RX_PW_P5_TypeDef; //0x16

////////////////////////////////////////////////////////////////////////////////

typedef struct {
	BYTE RX_EMPTY :1;
	BYTE RX_FULL :1;
	const BYTE RESERVED0 :2;
	BYTE TX_EMPTY :1;
	BYTE TX_FULL :1;
	BYTE TX_REUSE :1;
	const BYTE RESERVED1 :1;
} FIFO_STATUS_TypeDef; //0x17

////////////////////////////////////////////////////////////////////////////////

typedef struct {
	BYTE DPL_P0 :1;
	BYTE DPL_P1 :1;
	BYTE DPL_P2 :1;
	BYTE DPL_P3 :1;
	BYTE DPL_P4 :1;
	BYTE DPL_P5 :1;
	const BYTE RESERVED1 :2;
} DYNPD_TypeDef; //0x1C

////////////////////////////////////////////////////////////////////////////////

typedef struct {
	BYTE EN_DYN_ACK :1;
	BYTE EN_ACK_PAY :1;
	BYTE EN_DPL :1;
	const BYTE RESERVED1 :5;
} FEATURE_TypeDef; //0x1C

////////////////////////////////////////////////////////////////////////////////

//**************************************************************************************************************************//
// the all registers that are contained in nRF24L01

typedef struct {

	CONFIG_TypeDef CONFIG;
	EN_AA_TypeDef EN_AA;
	EN_RXADDR_TypeDef EN_RXADDR;
	SETUP_AW_TypeDef SETUP_AW;
	SETUP_RETR_TypeDef SETUP_RETR;
	RF_CH_TypeDef RF_CH;
	RF_SETUP_TypeDef RF_SETUP;
	STATUS_TypeDef STATUS;
	OBSERVE_TX_TypeDef OBSERVE_TX;
	RPD_TypeDef RPD;
	BYTE RX_ADDR_P0[DATA_PIPE_LENGTH]; //0x0A  // write the number of bytes defined in SETUP_AW (AW) // LSB written first
	BYTE RX_ADDR_P1[DATA_PIPE_LENGTH]; //0x0B
	BYTE RX_ADDR_P2[1];
	BYTE RX_ADDR_P3[1];
	BYTE RX_ADDR_P4[1];
	BYTE RX_ADDR_P5[1];
	BYTE TX_ADDR[DATA_PIPE_LENGTH]; //0x10 // write the number of bytes defined in SETUP_AW (AW) // LSB written first
	RX_PW_P0_TypeDef RX_PW_P0;
	RX_PW_P1_TypeDef RX_PW_P1;
	RX_PW_P2_TypeDef RX_PW_P2;
	RX_PW_P3_TypeDef RX_PW_P3;
	RX_PW_P4_TypeDef RX_PW_P4;
	RX_PW_P5_TypeDef RX_PW_P5;
	FIFO_STATUS_TypeDef FIFO_STATUS;
	DYNPD_TypeDef DYNPD;
	FEATURE_TypeDef FEATURE;
} nRF24L01;

//**************************************************************************************************************************//
//  REGISTERS AND BITFIELDS FOR nRF24L01+
//  BIT xxx_MASK can be used to be sure if all bits have been set up correctly (no reserved ones were changed)
//  To use xxx_MASK just & (AND) all previously set bits with this xxx_MASK
//
//**************************************************************************************************************************//
//CONFIG REGISTER
#define CONFIG_PRIM_RX      (BYTE)0x01
#define CONFIG_PWR_UP       (BYTE)0x02
#define CONFIG_CRCO         (BYTE)0x04
#define CONFIG_EN_CRC		(BYTE)0x08
#define CONFIG_MASK_MAX_RT  (BYTE)0x10
#define CONFIG_MASK_TX_DS   (BYTE)0x20
#define CONFIG_MASK_RX_DR   (BYTE)0x40
#define CONFIG_MASK         (BYTE)0x7F

//EN_AA REGISTER
#define EN_AA_ENAA_P0       (BYTE)0x01
#define EN_AA_ENAA_P1       (BYTE)0x02
#define EN_AA_ENAA_P2       (BYTE)0x04
#define EN_AA_ENAA_P3       (BYTE)0x08
#define EN_AA_ENAA_P4       (BYTE)0x10
#define EN_AA_ENAA_P5       (BYTE)0x20
#define EN_AA_MASK          (BYTE)0x3F

//EN_RXADDR REGISTER
#define EN_RXADDR_ERX_P0    (BYTE)0x01
#define EN_RXADDR_ERX_P1    (BYTE)0x02
#define EN_RXADDR_ERX_P2    (BYTE)0x04
#define EN_RXADDR_ERX_P3    (BYTE)0x08
#define EN_RXADDR_ERX_P4    (BYTE)0x10
#define EN_RXADDR_ERX_P5    (BYTE)0x20
#define EN_RXADDR_MASK      (BYTE)0x3F

//SETUP_AW REGISTER
#define SETUP_AW_3BYTES    (BYTE)0x01
#define SETUP_AW_4BYTES    (BYTE)0x02
#define SETUP_AW_5BYTES    (BYTE)0x03
#define SETUP_AW_MASK      (BYTE)0x03

//SETUP_RETR REGISTER
#define SETUP_RETR_ARC_RETRANSMIT_DISABLED    (BYTE)0x00
#define SETUP_RETR_ARC_1_RETRANSMIT           (BYTE)0x01
#define SETUP_RETR_ARC_2_RETRANSMIT           (BYTE)0x02
#define SETUP_RETR_ARC_3_RETRANSMIT           (BYTE)0x03
#define SETUP_RETR_ARC_4_RETRANSMIT           (BYTE)0x04
#define SETUP_RETR_ARC_5_RETRANSMIT           (BYTE)0x05
#define SETUP_RETR_ARC_6_RETRANSMIT           (BYTE)0x06
#define SETUP_RETR_ARC_7_RETRANSMIT           (BYTE)0x07
#define SETUP_RETR_ARC_8_RETRANSMIT           (BYTE)0x08
#define SETUP_RETR_ARC_9_RETRANSMIT           (BYTE)0x09
#define SETUP_RETR_ARC_10_RETRANSMIT          (BYTE)0x0A
#define SETUP_RETR_ARC_11_RETRANSMIT          (BYTE)0x0B
#define SETUP_RETR_ARC_12_RETRANSMIT          (BYTE)0x0C
#define SETUP_RETR_ARC_13_RETRANSMIT          (BYTE)0x0D
#define SETUP_RETR_ARC_14_RETRANSMIT          (BYTE)0x0E
#define SETUP_RETR_ARC_15_RETRANSMIT          (BYTE)0x0F
#define SETUP_RETR_ARD_250us                  (BYTE)0x00
#define SETUP_RETR_ARD_500us           		  (BYTE)0x10
#define SETUP_RETR_ARD_750us          		  (BYTE)0x20
#define SETUP_RETR_ARD_1000us          		  (BYTE)0x30
#define SETUP_RETR_ARD_1250us        	      (BYTE)0x40
#define SETUP_RETR_ARD_1500us       	      (BYTE)0x50
#define SETUP_RETR_ARD_1750us         		  (BYTE)0x60
#define SETUP_RETR_ARD_2000us        	      (BYTE)0x70
#define SETUP_RETR_ARD_2250us         		  (BYTE)0x80
#define SETUP_RETR_ARD_2500us         		  (BYTE)0x90
#define SETUP_RETR_ARD_2750us       	      (BYTE)0xA0
#define SETUP_RETR_ARD_3000us         		  (BYTE)0xB0
#define SETUP_RETR_ARD_3250us                 (BYTE)0xC0
#define SETUP_RETR_ARD_3500us                 (BYTE)0xD0
#define SETUP_RETR_ARD_3750us                 (BYTE)0xE0
#define SETUP_RETR_ARD_4000us                 (BYTE)0xF0

//RF_CH REGISTER
#define RF_CH_MASK  (BYTE)0x7F

//RF_SETUP REGISTER
#define RF_SETUP_RF_PWR_18dBm        		  (BYTE)0x00
#define RF_SETUP_RF_PWR_12dBm   	 		  (BYTE)0x02
#define RF_SETUP_RF_PWR_16dBm   	 		  (BYTE)0x04
#define RF_SETUP_RF_PWR_0dBm    	 		  (BYTE)0x06
#define RF_SETUP_RF_DR_HIGH_1Mbps             (BYTE)0x00
#define RF_SETUP_RF_DR_HIGH_2Mbps    		  (BYTE)0x08
#define RF_SETUP_RF_DR_HIGH_250kbps  		  (BYTE)0x20
#define RF_SETUP_RF_DR_LOW					  (BYTE)0x20
#define RF_SETUP_PLL_LOCK                     (BYTE)0x10
#define RF_SETUP_CONT_WAVE                    (BYTE)0x80
#define RF_SETUP_MASK						  (BYTE)0xBE

//STATUS REGISTER
#define STATUS_TX_FULL_MASK   		          (BYTE)0x01//R
#define STATUS_RX_P_NO_0_MASK  		          (BYTE)0x00//R
#define STATUS_RX_P_NO_1_MASK  		          (BYTE)0x02//R
#define STATUS_RX_P_NO_2_MASK  		          (BYTE)0x04//R
#define STATUS_RX_P_NO_3_MASK  		          (BYTE)0x06//R
#define STATUS_RX_P_NO_4_MASK 		          (BYTE)0x08//R
#define STATUS_RX_P_NO_5_MASK 		          (BYTE)0x09//R
#define STATUS_MAX_RT		                  (BYTE)0x10
#define STATUS_TX_DS		                  (BYTE)0x20
#define STATUS_RX_DR                          (BYTE)0x40
#define STATUS_MASK                           (BYTE)0x7F

//OBSERVE_TX REGISTER
#define OBSERVE_TX_PLOS_CNT_MASK 		      (BYTE)0xF0//R
#define OBSERVE_TX_ARC_CNT_MASK 		      (BYTE)0x0F//R
//RPD REGISTER
#define RPD_RPD_MASK 		                  (BYTE)0x01//R
#define RPD_MASK 		                      (BYTE)0xFE

//RX_PW_Px REGISTER
#define RX_PW_Px_PIPE_NOT_USED                (BYTE)0x00
#define RX_PW_Px_1_BYTE                       (BYTE)0x01
#define RX_PW_Px_2_BYTES					  (BYTE)0x02
#define RX_PW_Px_3_BYTES 					  (BYTE)0x03
#define RX_PW_Px_4_BYTES					  (BYTE)0x04
#define RX_PW_Px_5_BYTES					  (BYTE)0x05
#define RX_PW_Px_6_BYTES					  (BYTE)0x06
#define RX_PW_Px_7_BYTES					  (BYTE)0x07
#define RX_PW_Px_8_BYTES					  (BYTE)0x08
#define RX_PW_Px_9_BYTES					  (BYTE)0x09
#define RX_PW_Px_10_BYTES					  (BYTE)0x0A
#define RX_PW_Px_11_BYTES				      (BYTE)0x0B
#define RX_PW_Px_12_BYTES					  (BYTE)0x0C
#define RX_PW_Px_13_BYTES					  (BYTE)0x0D
#define RX_PW_Px_14_BYTES					  (BYTE)0x0E
#define RX_PW_Px_15_BYTES					  (BYTE)0x0F
#define RX_PW_Px_16_BYTES					  (BYTE)0x10
#define RX_PW_Px_17_BYTES					  (BYTE)0x11
#define RX_PW_Px_18_BYTES					  (BYTE)0x12
#define RX_PW_Px_19_BYTES					  (BYTE)0x13
#define RX_PW_Px_20_BYTES					  (BYTE)0x14
#define RX_PW_Px_21_BYTES					  (BYTE)0x15
#define RX_PW_Px_22_BYTES					  (BYTE)0x16
#define RX_PW_Px_23_BYTES					  (BYTE)0x17
#define RX_PW_Px_24_BYTES					  (BYTE)0x18
#define RX_PW_Px_25_BYTES					  (BYTE)0x19
#define RX_PW_Px_26_BYTES				      (BYTE)0x1A
#define RX_PW_Px_27_BYTES					  (BYTE)0x1B
#define RX_PW_Px_28_BYTES					  (BYTE)0x1C
#define RX_PW_Px_29_BYTES					  (BYTE)0x1D
#define RX_PW_Px_30_BYTES					  (BYTE)0x1E
#define RX_PW_Px_31_BYTES					  (BYTE)0x1F
#define RX_PW_Px_32_BYTES					  (BYTE)0x20
#define RX_PW_Px_MASK                         (BYTE)0x3F

//FIFO_STATUS REGISTER
#define FIFO_STATUS_RX_EMPTY_MASK  		      (BYTE)0x01//R
#define FIFO_STATUS_RX_FULL_MASK  		      (BYTE)0x02//R
#define FIFO_STATUS_TX_EMPTY_MASK  		      (BYTE)0x10//R
#define FIFO_STATUS_TX_FULL_MASK  		      (BYTE)0x20//R
#define FIFO_STATUS_TX_REUSE_MASK  		      (BYTE)0x40//R
#define FIFO_STATUS_MASK 		              (BYTE)0x73

//DYNPD REGISTER
#define DYNPD_DPL_P0  		      			  (BYTE)0x01
#define DYNPD_DPL_P1  		      			  (BYTE)0x02
#define DYNPD_DPL_P2  		      			  (BYTE)0x04
#define DYNPD_DPL_P3  		      			  (BYTE)0x08
#define DYNPD_DPL_P4  		      			  (BYTE)0x10
#define DYNPD_DPL_P5 		      			  (BYTE)0x20
#define DYNPD_MASK 		          			  (BYTE)0x3F

//FEATURE REGISTER
#define FEATURE_EN_DYN_ACK  		      	  (BYTE)0x01
#define FEATURE_EN_ACK_PAY  		      	  (BYTE)0x02
#define FEATURE_EN_DPL  		      		  (BYTE)0x04
#define FEATURE_MASK 		          		  (BYTE)0x07

//**************************************************************************************************************************//
//  FUNTIONS
//**************************************************************************************************************************//
// INLINE

void nRF24L01_En_AA_Reg(EN_AA_TypeDef *conf);
void nRF24L01_En_RxAddr_Reg(EN_RXADDR_TypeDef *conf);
void nRF24L01_Setup_AW_Reg(SETUP_AW_TypeDef *conf);
void nRF24L01_Setup_RETR_Reg(SETUP_RETR_TypeDef *conf);
void nRF24L01_Rf_CH_Reg(RF_CH_TypeDef *conf);
void nRF24L01_Rf_Setup_Reg(RF_SETUP_TypeDef *conf);
void nRF24L01_Status_Reg(STATUS_TypeDef *conf);
void nRF24L01_Observe_TX_Reg(OBSERVE_TX_TypeDef *conf);
void nRF24L01_Rpd_Reg(RPD_TypeDef *conf);
void nRF24L01_Rx_Pw_Px_Reg(void *conf);
void nRF24L01_Fifo_Status_Reg(FIFO_STATUS_TypeDef *conf);
void nRF24L01_Dynpd_Reg(DYNPD_TypeDef*conf);
void nRF24L01_Feature_Reg(FEATURE_TypeDef *conf);
void nRF24L01_Init(void);

bool nRF24L01_RegisterCallbacks(void (*SPI_ReadData)(uint8_t* data),
		void (*SPI_WriteData)(uint8_t *data), void (*CE_Pin_Set)(void),
		void (*CE_Pin_Reset)(void),void(*IRQ_PinSetup)(void));

void nRF24L01_SetTxMode(void);
void nRF24L01_SetRxMode(void);
void nRF24L01_FlushTx(void);
void nRF24L01_FlushRx(void) ;
void nRF24L01_PowerUp(void);
void nRF24L01_PowerDown(void);
void nRF24L01_SetCE_PIN(void);
void nRF24L01_ResetCE_PIN(void);
void nRF24L01_NOP(void);
void nRF24L01_ReadSTATUS(void);
void nRF24L01_FlushSTATUSReg(void);
void nRF24L01_ReadCONFIGReg(void);
void nRF24L01_SendDataToFifo(uint8_t*data);

#ifdef __cplusplus
}
#endif

#endif /*__NRF24L01+_H */
