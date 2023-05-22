/*
 * FanControl v2.0 - Fan Kitchen v1.0
 *
 * Created: 12.01.2023
 *  Author: Vadim Kulakov, vad7@yahoo.com
 *
 * ATtiny44A/84A
 *
 * Radio nRF24L01+
 * IR TL1838V
 * Relay G3MB-202P * 4
 * VCC 5V, 3.3V for nRF24
 */ 
#define F_CPU 8000000UL
// Fuses: BODLEVEL = 1.8V (BODLEVEL[2:0] = 110), RSTDISBL=0, EESAVE=0

//#define DEBUG_PROTEUS

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <util/atomic.h>

const char ProgramID[] PROGMEM = "Fan Kitchen v1";

#define KEY							(1<<PORTB3)
#define KEY_PRESSING				!(PINB & KEY)
#define KEY_SETUP					PORTB |= KEY

#define COOKER_HOOD					// есть кухонная вытяжка
#ifdef COOKER_HOOD
#define FanCooker_PORT				PORTA
#define FanCooker_DDR				DDRA
#define FanCooker_PORT_ADDR			(0x20 + FanCooker_PORT)
#define FanCooker_PIN				PINA
#define FanCooker_Out1				(1<<PORTA7)	// F1.1
#define FanCooker_Out2				(1<<PORTA0)	// F2.1
#define FanCooker_OFF				FanCooker_PORT &= ~(FanCooker_Out1 | FanCooker_Out2)
#define FanCooker_Out2_ON			FanCooker_PORT |= FanCooker_Out2
#define FanCooker_Out2_OFF			FanCooker_PORT &= ~FanCooker_Out2
#endif

#define Fan_PORT					PORTB
#define Fan_DDR						DDRB
#define Fan_PORT_ADDR				(0x20 + Fan_PORT)
#define Fan_PIN						PINB
#define Fan_Out1					(1<<PORTB1)	// F1.2
#define Fan_Out2					(1<<PORTB0)	// F2.2
#define Fan_OFF						Fan_PORT &= ~(Fan_Out1 | Fan_Out2)
#define Fan_Out2_ON					Fan_PORT |= Fan_Out2
#define Fan_Out2_OFF				Fan_PORT &= ~Fan_Out2

#define Fans_SETUP					FanCooker_DDR |= FanCooker_Out1 | FanCooker_Out2; Fan_DDR |= Fan_Out1 | Fan_Out2

#define LED1_PORT					PORTA	// Info LED(s)
#define LED1_DDR					DDRA
#define LED1_TWIN							// 2 LED - to VCC & to GND
#define LED1						(1<<PORTA1)
register uint8_t led1 asm("10");	// control var: 0 - off, 1 - on
#define LED1_ON						led1 = 1
#define LED1_OFF					led1 = 0
#ifndef LED1_TWIN					// ONE LED:
#define LED1_INIT					LED1_DDR |= LED1
#define _LED1_ON					LED1_PORT |= LED1
#define _LED1_OFF					LED1_PORT &= ~LED1
#define _LED1_STANDBY
#define LED1_ON_BEFORE_RESET		_LED1_ON
#else								// TWIN LED:		
#define LED1_INIT
#define _LED1_ON					LED1_PORT |= LED1; LED1_DDR |= LED1
#define _LED1_OFF					LED1_DDR &= ~LED1
#define _LED1_STANDBY				LED1_PORT &= ~LED1; LED1_DDR |= LED1
#define LED1_ON_BEFORE_RESET		LED1_DDR |= LED1
#endif

//const uint8_t FanPORTS[] PROGMEM = { Fan_Speed1, Fan_Speed2,  } // FanSpeed_1 - FanCookerSpeed_3

#define Fan_DamperSwitchTime		25	// sec
#define FANS_SLEEP_STEP				30	// minutes
#define FAN_IDXS					3	// How many speeds indices does each fan have

//uint8_t FanSpeed					= 0; // Current fan speed
//uint8_t FanSleep					= 0; // Current fan sleep
uint8_t SleepTimer					= 0; // *FANS_SLEEP_STEP minutes
uint8_t OffTimer					= 0; // sec
int8_t  OutPeriod;					// Fans period of regulation (number 1/100Hz halfwaves)
uint8_t* FanPortADDR;				// global address of PORTA/PORTB for current fan 
uint8_t  FanPortPIN;				// Turn on PORT value for current fan

#define	IR_MAX_CONTROLS				10 // Max different remote controls
#define	IR_PULSES_MIN				8 // IR packet minimum pulses
enum {
	IR_WAITING = 0,
	IR_START,
	IR_READING, 
	IR_DONE
};
enum {
	IR_Key_FanUp = 0,		// Fan (from OFF - speed 3)
	IR_Key_FanDown,			// Fan (from OFF - speed 1)
	IR_Key_CookerUp,		// or right - FanCooker + (from OFF - speed 3)
	IR_Key_CookerDown,		// or left  - FanCooker - (from OFF - speed 1)
	IR_Key_Off,
	IR_Key_CookerLight,
	IR_Key_Setup,
	IR_Keys_Total			// number of keys
};
uint8_t IR_Status					= IR_WAITING;
uint8_t IR_Cnt						= 0;
uint16_t IR_LastDuration;
uint16_t IR_Hash;

uint8_t IRRepeatDelay				= 0;	// *0.1 sec
uint8_t IRRepeatCnt					= 0;
uint16_t IR_Hash					= 0;
typeof(IR_Hash) IRHash_Last			= 0;
uint8_t Key1Pressed					= 0;	// Time
uint8_t Key1Pause					= 0;
volatile uint8_t Timer				= 0;	// sec
uint8_t TimerSecCnt					= 0;
uint8_t Flags;
uint8_t FanOn						= 0;	// 0 - off, 1..6 - FanSpeed_1 ... FanCookerSpeed_3
uint8_t FanSpeed					= 0;	// current speed
uint8_t FanOnNext					= 0;	// after timeouts next Fan speed
uint8_t FanOnLast					= 0;	// before FanOnNext
uint8_t FanOnNextCnt				= 0;	// sec
uint8_t SetupItem					= 0;
uint8_t Setup						= 0;	// Setup enum
uint8_t CookerLight_force_on		= 0;
uint8_t SetFanSpeed_by_CO2			= 0;	// 1 - change due to CO2 level

enum {
	fSetup_Off = 0,
	fSetup_IR,
	fSetup_Speed
};

#define REPEAT_TIMES_SETUP_SPEED	3
#define REPEAT_TIMES_SETUP_IR		5
#define IR_REPEAT_TIMEOUT			2		// *0.1 sec
#define IR_REPEAT_TIMEOUT_SETUP		50		// *0.1 sec

// EEPROM.Flags:
#define f_NRF24						(1<<0)	// Present nRF24L01
#define f_FanHiSpeed				(1<<1)	// Present Fan Hi speed - second motor coil - Port: FAN2.2 (Fan_Out2)
#define f_FanCookerHiSpeed			(1<<2)	// Present FanCooker Hi speed - second motor coil - Port: FAN2.1 (FanCooker_Out2)
#define f_CookerLamp_FanOut2		(1<<3)	// Present Cooker Lamp on Fan out 2 (FAN2.2)
#define f_CookerLamp_CookerOut2		(1<<4)	// Present Cooker Lamp on CookerFan out 2 (FAN1.2)
#define f_FanDamper					(1<<5)	// Present Fan Damper - Port: FAN2.2 (Fan_Out2). Damper is ON all time when Fan is ON
//#define f_FanCookerDamper			(1<<6)	// Present FanCooker - Port: FAN2.2 (Fan_Out2). Damper is ON all time when Fan is ON
//#define f_Damper					(1<<7)	// Central damper - one for all fans (FAN2.2)

struct _EEPROM {
/* 0*/	uint8_t _OSCCAL;
/* 1*/	uint8_t Flags;				// Flags
/* 2*/	uint8_t RF_Address;			// nRF24 address LSB
/* 3*/	uint8_t RF_Channel;			// nRF24 channel
/* 4*/	uint16_t CO2_threshold1;	// CO2 threshold to start Fan speed 1 (if FanCooker is OFF)
/* 6*/	uint16_t CO2_threshold2;	// CO2 threshold to start Fan speed 2 (if FanCooker is OFF)
/* 8*/	uint16_t CO2_threshold3;	// CO2 threshold to start Fan speed 3 (if FanCooker is OFF)
/*10*/	uint8_t FanStartupMaxSpTime;// sec forced max speed 
/*11*/	uint8_t FanShutdownTime;	// Fan turning off time (used for switch from Fan to FanCooker), sec
/*12*/	uint8_t FanCookerStartupMaxSpTime; // sec forced max speed 
/*13*/	uint8_t FanCookerShutdownTime;// FanCooker turning off time (used for switch from FanCooker to Fan), sec
/*14*/	uint8_t FanSleep;			// Auto sleep time, *FANS_SLEEP_STEP
/*15*/	uint8_t FanCookerSleep;		// Auto sleep time, *FANS_SLEEP_STEP
/*16*/	uint8_t SpeedInitIdx;	// Power up Fans speed index: 0 - all off, 1 - Fan speed1, 2 - Fan speed2, 3 - Fan speed3, 4 - FanCooker speed 1, 5 - FanCooker speed 2, 6 - FanCooker speed 3
/*17*/	uint8_t SpeedKeyIdx;		// When key pressed
/*18*/	uint8_t OutPeriod;			// Fans period of regulation (number 1/100Hz halfwaves)
// Idx: From 0 (off) to OutPeriod * 2, if > OutPeriod then second coil used if available
/*19*/	uint8_t FanSpeed_1;
/*20*/	uint8_t FanSpeed_2;
/*21*/	uint8_t FanSpeed_3;
/*22*/	uint8_t FanCookerSpeed_1;
/*23*/	uint8_t FanCookerSpeed_2;
/*24*/	uint8_t FanCookerSpeed_3;
	//
/*25*/	uint8_t FanSpeeds;			// Number of speeds (1..FAN_IDXS)
/*26*/	uint8_t FanCookerSpeeds;	// Number of speeds (1..FAN_IDXS)
/*27*/	uint8_t _reserved[5];		// -> #0x20
/*32*/	uint8_t IRRemotes; // Total active remote controls
/*33*/	uint16_t IRCommandArray[IR_MAX_CONTROLS * IR_Keys_Total]; // type like IRHash
} __attribute__ ((packed));
struct _EEPROM EEMEM EEPROM;

#define FANS_IDX_EEPROM_OFFSET		&EEPROM.FanSpeed_1
#define GET_FANS_IDX_EEPROM(a)		(a - FANS_IDX_EEPROM_OFFSET + 1) // 1..FAN_IDXS*2

#define fCMD_Write					0x80 // EEPROM[Type] = Data, "fCMD_WriteStart" must be preceded, timeout - fCMD_Write_Timeout
#define fCMD_Read					0x40 // read MEM[Data] => Data, MEM: EEPROM, MAIN, PROGRAM
#define fCMD_Set					0xC0 // Set cmd, Type = cmd id, Data = cmd value
#define fCMD_WriteStart				0x2F
// fCMD +
#define fCMD_EEPROM					0x00	// EEPROM
#define fCMD_RAM					0x10	// RAM memory
#define fCMD_PROGMEM				0x20	// Program FLASH
#define fCMD_1b						0x01
#define fCMD_2b						0x02
#define fCMD_4b						0x03
#define fCMD_8b						0x04
#define fCMD_CStr					0x05	// #0 = ProgramID

#define Type_Set_Lamp				0	// Lamp ON/OFF, bit num
#define Type_Set_Fan				1	// Set Fan idx = Data
#define Type_Set_FanSpeedUp			2	// FanSpeed +1
#define Type_Set_FanSpeedDown		3	// FanSpeed -1
#define Type_Set_FanSpeedSave		4	// Activate Setup IR mode, bit num
#define Type_Set_SetupIR			5	// Save FanSpeed to working fan idx
#define Type_Set_RESET				14	// Restart program, software reset (Data = 0xEEEE)

#define fCMD_Write_Timeout			3	// *0.1 sec

struct SETUP_DATA { // the same size as SEND_DATA!
	uint16_t Data;	// Read/Write byte or word
	uint8_t Type;	// Read data type(SetupType.*) or Write EEPROM address
	uint8_t Flags;	// Setup command: fSetup_*
} __attribute__ ((packed));

struct SEND_DATA {
	uint16_t CO2level;
	uint8_t FanSpeed;
	uint8_t Flags;
} __attribute__ ((packed));
struct SEND_DATA data;
uint8_t WriteTimeout = 0;

#if(1)
void Delay10us(uint8_t ms) {
	while(ms-- > 0) _delay_us(10); 
	wdt_reset();
}
//void Delay1ms(uint8_t ms) {
//	while(ms-- > 0) {
//		_delay_ms(1); wdt_reset();
//	}
//}
void Delay100ms(unsigned int ms) {
	while(ms-- > 0) {
		_delay_ms(100); wdt_reset();
	}
}

void FlashLED(uint8_t num, uint8_t toff, uint8_t ton) {
	while (num-- > 0) {
		LED1_OFF;
		Delay100ms(toff);
		LED1_ON;
		Delay100ms(ton);
	}
	LED1_OFF;
}

#endif

#include "nRF24L01.h"

#define SETUP_WATCHDOG WDTCSR = (1<<WDCE) | (1<<WDE); WDTCSR = (1<<WDE) | (0<<WDIE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0);	//  Watchdog 1 s

//											// 0xF0 mask - Number of long flashes, 0x0F mask - Number of short flashes
#define WRN_SETUP					0x01
#define WRN_SETUP_INFO				0x02
#define WRN_FAN_SWITCHING			0x10
#define WRN_SETUP_ERR				0x10	// +ERR
#define WRN_RF_Receive				0x01
#define WRN_RF_Send					0x10	// Send failure, after short bursts = fan offset (mask 0x0F)
#define WRN_RF_SetAddr				0x20	// Set addresses failure,
#define WRN_RF_NotResp				0x30	// RF module not response,
#define WRN_CO2Sensor				0x40	// CO2 Sensor reading failure

uint8_t LED_Warning = 0, LED_WarningOnCnt = 0, LED_WarningOffCnt = 0, LED_Warning_WorkLong = 0, LED_Warning_WorkShort = 0;
uint8_t LED_WarningPause = 0;
uint8_t TimerCnt100ms = 0;
uint8_t TimerCnt = 0;
uint8_t Timer1sec = 0;
uint16_t TimerCntFanSleepStep = 0;
int8_t  Out_accum = 0;
ISR(TIM0_OVF_vect, ISR_NOBLOCK) // 0.01 sec
{
	// Bresenham's line algorithm for power control
	if(FanOn) {
		Out_accum += FanSpeed;
		if(Out_accum >= OutPeriod){
			Out_accum -= OutPeriod;
			*FanPortADDR |= FanPortPIN;		// On
		} else *FanPortADDR &= ~FanPortPIN;	// Off
	}
	//	
	if(++TimerCnt100ms >= 10) { // 0.1 sec
		TimerCnt100ms = 0;
		if(++TimerCnt >= 10) { // 1 sec
			TimerCnt = 0;
			Timer1sec = 1;
		}
		if(LED_WarningPause) {
			if(--LED_WarningPause == 0) LED1_OFF;
		} else {
			// LED_Warning: 0xF0 mask - Number of long flashes, 0x0F mask - Number of short flashes, LED_Warning_NoRepeat = no repeat
			if(LED_WarningOnCnt) {
				LED1_ON;
				LED_WarningOnCnt--;
			} else if(LED_WarningOffCnt) {
				LED1_OFF;
				LED_WarningOffCnt--;
			} else if(LED_Warning_WorkLong) { // long flashes
				LED_Warning_WorkLong--;
				LED_WarningOnCnt = 14;			// 1.4s (*0.1s)
				if(LED_Warning_WorkLong == 0) {
					LED_WarningOffCnt = 6;		// 0.6s
					goto xSetPause;
				} else LED_WarningOffCnt = 4;	// 0.4s
			} else if(LED_Warning_WorkShort) { // short flashes
				LED_Warning_WorkShort--;
				LED_WarningOnCnt = 3;	// 0.3s
				LED_WarningOffCnt = 3;	// 0.3s
xSetPause:
				if(LED_Warning_WorkShort == 0) LED_WarningOffCnt = 25; // 2.5s
			} else if(LED_Warning) {
				LED_Warning_WorkLong = (LED_Warning & 0xF0) >> 4;
				LED_Warning_WorkShort = LED_Warning & 0x0F;
				LED_Warning = 0;
			}
		}
		if(IRRepeatDelay) IRRepeatDelay--;
		if(WriteTimeout) WriteTimeout--;
		if(Key1Pause == 0 && Key1Pressed < 255 && KEY_PRESSING) Key1Pressed++;
	}
	if(Key1Pause) Key1Pause--; else if(KEY_PRESSING && Key1Pressed == 0) Key1Pressed = 1;
	if(led1) {
		_LED1_ON;
	} else {
		if(FanOn || (FanCooker_PORT & FanCooker_Out2) || (Fan_PORT & Fan_Out2)) { _LED1_OFF; } else { _LED1_STANDBY; }
	}
}

ISR(TIM1_OVF_vect) // IR timeout
{
	if(IR_Status == IR_READING) {
		if(IR_Cnt > IR_PULSES_MIN) IR_Status = IR_DONE; else IR_Status = IR_WAITING;
	} else if(IR_Status == IR_START) IR_Status = IR_WAITING;
}

ISR(EXT_INT0_vect) // IR, PIN change
{
	if(IR_Status <= IR_READING) {
		uint16_t _TCNT = TCNT1;
		TCNT1 = 0;
		if(IR_Status == IR_WAITING) {
			IR_Status = IR_START;
		} else if(IR_Status == IR_START) {
			IR_Cnt = 0;
			IR_Hash = 5381; // hash init
			IR_LastDuration = _TCNT;
			IR_Status = IR_READING;
		} else {
			uint8_t n;
			if(_TCNT < IR_LastDuration * 3 / 4) n = 1;
			else if(IR_LastDuration < _TCNT * 3 / 4) n = 2;
			else n = 0;
			IR_Hash = ((IR_Hash << 5) + IR_Hash) ^ n;
			IR_LastDuration = _TCNT;
			if(++IR_Cnt == 0) IR_Status = IR_DONE;
		}
	}
}

void Set_LED_Warning(uint8_t d)
{
	if(LED_Warning == 0) LED_Warning = d;
}

void Set_LED_Warning_New(uint8_t d)
{
	LED_Warning_WorkLong = LED_Warning_WorkShort = LED_WarningOnCnt = LED_WarningOffCnt = 0;
	LED_Warning = d;
	LED1_OFF;
}

void ResetSettings(void)
{
	eeprom_update_byte(&EEPROM.Flags, f_NRF24 | f_FanCookerHiSpeed | f_CookerLamp_FanOut2);// Flags
 	eeprom_update_byte(&EEPROM.RF_Address, 0xC1);		// nRF24 address LSB
 	eeprom_update_byte(&EEPROM.RF_Channel, 122);		// nRF24 channel
 	eeprom_update_byte(&EEPROM.OutPeriod, 15);			// Fans period of regulation (number 1/100Hz halfwaves), max speed value
 	eeprom_update_byte(&EEPROM.SpeedInitIdx, 0);		// Power up Fans speed index: 0 - all off, 1 - Fan speed1, 2 - Fan speed2, 3 - Fan speed3, 4 - FanCooker speed 1, 5 - FanCooker speed 2, 6 - FanCooker speed 3
 	eeprom_update_byte(&EEPROM.IRRemotes, 0);
/*	 
	eeprom_update_word(&EEPROM.CO2_threshold1, 750);// CO2 threshold to start Fan speed 1 (if FanCooker is OFF)
	eeprom_update_word(&EEPROM.CO2_threshold2, 850);	// CO2 threshold to start Fan speed 2 (if FanCooker is OFF)
	eeprom_update_word(&EEPROM.CO2_threshold3, 1000);	// CO2 threshold to start Fan speed 3 (if FanCooker is OFF)
	eeprom_update_byte(&EEPROM.FanStartupMaxSpTime, 150);// sec forced max speed
	eeprom_update_byte(&EEPROM.FanShutdownTime, 100);	// Fan turning off time (used for switch from Fan to FanCooker), sec
	eeprom_update_byte(&EEPROM.FanCookerShutdownTime, 0);// FanCooker turning off time (used for switch from FanCooker to Fan), sec
	eeprom_update_byte(&EEPROM.FanCookerStartupMaxSpTime, 2); // sec forced max speed
	eeprom_update_byte(&EEPROM.FanSleep, 20);			// *30 = 10 h, Auto sleep time, *FANS_SLEEP_STEP
	eeprom_update_byte(&EEPROM.FanCookerSleep, 6);		// *30 = 3  h, Auto sleep time, *FANS_SLEEP_STEP
	eeprom_update_byte(&EEPROM.SpeedKeyIdx, 6);			// FanCooker3
	eeprom_update_byte(&EEPROM.FanSpeeds, 2);
	eeprom_update_byte(&EEPROM.FanCookerSpeeds, 3);
	eeprom_update_byte(&EEPROM.FanSpeed_1, 14);			// 1..OutPeriod, OutPeriod+1..OutPeriod*2 - second fan coil
	eeprom_update_byte(&EEPROM.FanSpeed_2, 15);
	eeprom_update_byte(&EEPROM.FanSpeed_3, 15);
	eeprom_update_byte(&EEPROM.FanCookerSpeed_1, 14);
	eeprom_update_byte(&EEPROM.FanCookerSpeed_2, 15);
	eeprom_update_byte(&EEPROM.FanCookerSpeed_3, 15 * 2);
*/
}

void GetSettings(void)
{
//	uint8_t b = eeprom_read_byte(&EEPROM._OSCCAL);
//	if(b != 0xFF) OSCCAL = b;
	Flags = eeprom_read_byte(&EEPROM.Flags);
	OutPeriod = eeprom_read_byte(&EEPROM.OutPeriod);
}

// Set speed of FanCooker/Fan, speed index: 0(off), 1..FAN_IDXS*2
void SetFanSpeed(uint8_t fidx)
{
	if(fidx > FAN_IDXS * 2) return;
	if(FanOnNext && (fidx == 0 || FanOnNext == fidx)) FanOnNext = 0;
	if(fidx != FanOn) {
		if(FanOn == 0) { // From OFF mode
			FanOnNext = fidx;
			if(fidx > FAN_IDXS) { // FanCooker
				if(FanOnNext > FAN_IDXS + eeprom_read_byte(&EEPROM.FanCookerSpeeds)) FanOnNext = FAN_IDXS + eeprom_read_byte(&EEPROM.FanCookerSpeeds);
				if(FanOnLast && FanOnLast < FAN_IDXS) { // switch from Fan and shutdown is in process
					fidx = 0;
				} else {
					fidx = FAN_IDXS + eeprom_read_byte(&EEPROM.FanCookerSpeeds);
					FanOnNextCnt = eeprom_read_byte(&EEPROM.FanCookerStartupMaxSpTime);
				}
			} else { // Fan
				if(FanOnNext > eeprom_read_byte(&EEPROM.FanSpeeds)) FanOnNext = eeprom_read_byte(&EEPROM.FanSpeeds);
				if(FanOnLast > FAN_IDXS) { // switch from FanCooker and shutdown is in process
					fidx = 0;
				} else {
					fidx = eeprom_read_byte(&EEPROM.FanSpeeds);
					FanOnNextCnt = eeprom_read_byte(&EEPROM.FanStartupMaxSpTime);
				}
			}
 			if(FanOnNextCnt == 0) {
 				fidx = FanOnNext;
 				FanOnNext = 0;
 			}
		} else if(fidx == 0) { // -> off
		} else if(fidx > FAN_IDXS && FanOn <= FAN_IDXS) { // switch from Fan to FanCooker
			FanOnNextCnt = eeprom_read_byte(&EEPROM.FanShutdownTime);
			if(FanOnNextCnt) {
				FanOnLast = FanOn;
				FanOnNext = fidx;
				fidx = 0;
			}
		} else if(fidx <= FAN_IDXS && FanOn > FAN_IDXS) { // switch from FanCooker to Fan
			FanOnNextCnt = eeprom_read_byte(&EEPROM.FanCookerShutdownTime);
			if(FanOnNextCnt) {
				FanOnLast = FanOn;
				FanOnNext = fidx;
				fidx = 0;
			}
		} else {
			if(FanOnNext) {
				FanOnNext = fidx;
				LED_Warning = fidx;
				goto xEnd;
			}
		}
		if(FanOnNext == fidx) {
			FanOnNextCnt = 0;
			FanOnNext = 0;
		}
		FanOn = 0; // disable function in ISR
		*(uint8_t*)FanPortADDR &= ~FanPortPIN;	// Off
		if(!CookerLight_force_on && !SetFanSpeed_by_CO2) {
			if(Flags & f_CookerLamp_CookerOut2) FanCooker_Out2_OFF;
			else if(Flags & f_CookerLamp_FanOut2) Fan_Out2_OFF;
		}
		if(fidx) {
			uint8_t speed = eeprom_read_byte(FANS_IDX_EEPROM_OFFSET - 1 + fidx);
			if(fidx > FAN_IDXS) { // FanCooker
				FanPortADDR = (uint8_t*)&FanCooker_PORT;
				FanPortPIN = FanCooker_Out1;
				if(speed > OutPeriod) {
					if(Flags & f_FanCookerHiSpeed) FanPortPIN = FanCooker_Out2;
					speed -= OutPeriod;
				}
				if(Flags & f_CookerLamp_CookerOut2) FanCooker_Out2_ON;
				else if(Flags & f_CookerLamp_FanOut2) Fan_Out2_ON;
//				if(Flags & f_FanCookerDamper) FanCooker_Out2_ON;
			} else { // Fan
				FanPortADDR = (uint8_t*)&Fan_PORT;
				FanPortPIN = Fan_Out1;
				if(speed > OutPeriod) {
					if(Flags & f_FanHiSpeed) FanPortPIN = Fan_Out2;
					speed -= OutPeriod;
				}
				if(Flags & (f_FanDamper /*| f_Damper*/)) Fan_Out2_ON;
			}
			LED_Warning = fidx;
			FanSpeed = speed;
			FanOn = fidx;
		} else if(FanOnNext == 0) {
			if(Flags & (f_FanDamper /*| f_Damper*/)) Fan_Out2_OFF;
//			if(Flags & f_FanCookerDamper) FanCooker_Out2_OFF;
		}
	}
xEnd:
	if(FanOn == 0 && FanOnNext == 0) SleepTimer = 0;
	else if(!SetFanSpeed_by_CO2) {
		if(fidx == 0) fidx = FanOnNext;
		SleepTimer = eeprom_read_byte(fidx > FAN_IDXS ? &EEPROM.FanCookerSleep : &EEPROM.FanSleep);
	}
}

void FanSpeedUp(void)
{
	if(FanOn < eeprom_read_byte(&EEPROM.FanSpeeds) || (FanOn > FAN_IDXS && FanOn - FAN_IDXS < eeprom_read_byte(&EEPROM.FanCookerSpeeds))) SetFanSpeed(FanOn + 1);
}

void FanSpeedDown(void)
{
	if(FanOn == FAN_IDXS + 1) SetFanSpeed(0); else SetFanSpeed(FanOn - 1);
}

int main(void)
{
	CLKPR = (1<<CLKPCE); CLKPR = (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0); // Clock prescaler division factor: 1
	MCUCR = (1<<SE) | (0<<SM1) | (0<<SM0); // Idle sleep enable
	NRF24_DDR |= NRF24_CE | NRF24_CSN | NRF24_SCK | NRF24_MOSI; // Out
	LED1_INIT;
	Fans_SETUP;
	KEY_SETUP;
	// Timer 8 bit
	TCCR0A = (0<<WGM01) | (1<<WGM00);  // Timer0: PWM, Phase Correct, ТОP OCR0A
	TCCR0B = (1<<WGM02) | (1 << CS02) | (0 << CS01) | (1 << CS00); // Timer0 prescaller: 1024
	OCR0A = 39; // = Fclk/prescaller/2/Freq = 50hz
	//OCR0B = 0; // Half Duty cycle ((TOP+1)/2-1)
	TIMSK0 |= (1<<TOIE0); // Timer/Counter Overflow Interrupt Enable
	// Timer 16 bit
	TCCR1A = (1<<WGM11) | (1<<WGM10);  // Timer1: Fast PWM, top OCR1A
	TCCR1B = (1<<WGM13) | (1<<WGM12) | (0<<CS12) | (1<<CS11) | (1<<CS10); // Timer1: /64 = 125000
	OCR1A = 2500; // Timeout value. OC0A(TOP)=Fclk/prescaller/Freq - 1; Freq=Fclk/(prescaller*(1+TOP))
	TIMSK1 |= (1<<TOIE1); // Timer/Counter Overflow Interrupt Enable
	// ADC
// 	ADMUX = (0<<REFS1) | (1<<MUX2)|(1<<MUX1)|(1<<MUX0); // ADC7 (PA7)
// 	ADCSRA = (1<<ADEN) | (0<<ADATE) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); // ADC enable, Free Running mode, Interrupt, ADC 128 divider
// 	ADCSRB = (1<<ADLAR) | (0<<ADTS2) | (0<<ADTS1) | (0<<ADTS0); // ADC Left Adjust Result
	// Pin change
	//GIMSK = (1<<PCIE1); // Pin Change Interrupt Enable
	//PCMSK1 = (1<<PCINT11); // Pin Change Mask Register - Keys
	// Prepare for IR receiving
	MCUCR |= (0<<ISC01) | (1<<ISC00); // Any logical change on INT0
	GIMSK |= (1<<INT0); // External Interrupt Request 0 Enable
	GIFR |= (1<<INTF0); // Clear INT flag
	SETUP_WATCHDOG;
	uint8_t n = eeprom_read_byte(&EEPROM.SpeedInitIdx);
	if(n > FAN_IDXS * 2) {
		ResetSettings();
		n = 0;
	}
	GetSettings();
	sei();
	SetFanSpeed(n);
	FlashLED(1, 0, 10);
	if(Flags & f_NRF24) {
		NRF24_init(eeprom_read_byte(&EEPROM.RF_Channel)); // After init transmission must be delayed
		while(!NRF24_SetAddresses(eeprom_read_byte(&EEPROM.RF_Address))) {
			FlashLED(5,1,1);
#ifdef DEBUG_PROTEUS
			break;
#endif
		}
		NRF24_SetMode(NRF24_ReceiveMode);
	}
	while(1)
	{
		__asm__ volatile ("" ::: "memory"); // Need memory barrier
		sleep_cpu();
		wdt_reset();
		if(Timer1sec) {
			Timer1sec = 0;
			if(++TimerCntFanSleepStep >= FANS_SLEEP_STEP * 60) {
				TimerCntFanSleepStep = 0;
				if(SleepTimer) if(--SleepTimer == 0 && FanOn) SetFanSpeed(0);
			}
			if(FanOnNextCnt) if(--FanOnNextCnt == 0) {
				if(FanOnNext) SetFanSpeed(FanOnNext); else FanOnLast = 0;
			}
			if(FanOnNext) Set_LED_Warning(WRN_FAN_SWITCHING);
			if(Timer) {
				if(--Timer == 0) {
 					if(Setup) {
xSetup_finish:						 
						 Setup = fSetup_Off;
						 SetupItem = 0;
						 FlashLED(5, 3, 3);
					}
				}
				if(Setup) Set_LED_Warning(SetupItem + 1);
			}
			if(IRRepeatDelay == 0) { // Setup pressed n times
				uint8_t n = IRRepeatCnt;
				IRRepeatCnt = 0;
				if(n) {
					if(Setup == fSetup_Speed) {
						eeprom_update_byte(FANS_IDX_EEPROM_OFFSET - 1 + FanOn, FanSpeed);
						goto xSetup_finish;
					} else if(n >= REPEAT_TIMES_SETUP_IR) {
						goto xStartSetupIR;
					} else if(n >= REPEAT_TIMES_SETUP_SPEED) {
						if(FanOn) {
							FlashLED(1, 5, 15);
							Setup = fSetup_Speed;
							LED_Warning = FanSpeed;
							Timer = 255;
						}
					}
				}
			}
		}
		if(IR_Status == IR_DONE) {
			IR_Status = IR_WAITING;
			uint8_t remotes_max = eeprom_read_byte(&EEPROM.IRRemotes) * IR_Keys_Total;
			if(Setup == fSetup_IR) {
				if(IRRepeatDelay) continue;
				remotes_max += SetupItem;
			}
			LED_WarningPause = 2;
			LED1_ON;
			uint8_t i = 0;
			for(; i < remotes_max; i++) {
				if(eeprom_read_word(&EEPROM.IRCommandArray[i]) == IR_Hash) {
					uint8_t key = i % IR_Keys_Total;
					if(Setup == fSetup_IR) { // already exist!
						if(key == IR_Key_Setup) { // skip if pressed "Setup" on other or the same remote
							IR_Hash = 0;
							goto xSetupIR_New;
						}
						FlashLED(5, 1, 1);
						break;
					}
					if(key == IR_Key_Setup) {
						IRRepeatCnt++;
						IRRepeatDelay = IR_REPEAT_TIMEOUT_SETUP;
					} else if(IRRepeatDelay == 0) {
						IRRepeatDelay = IR_REPEAT_TIMEOUT;
						if(key == IR_Key_Off) {
							if(Setup == fSetup_Speed) {
								goto xSetup_finish;
							} else {
								SetFanSpeed(0); // Off
							}
						} else if(key == IR_Key_CookerUp) { // FanCooker
							if(Setup == fSetup_Speed) goto xFanSpeedUp;
							else if(FanOn > FAN_IDXS) FanSpeedUp(); else SetFanSpeed(GET_FANS_IDX_EEPROM(&EEPROM.FanCookerSpeed_3));
						} else if(key == IR_Key_CookerDown) { // FanCooker
							if(Setup == fSetup_Speed) goto xFanSpeedDown;
 							else if(FanOn > FAN_IDXS) FanSpeedDown(); else SetFanSpeed(GET_FANS_IDX_EEPROM(&EEPROM.FanCookerSpeed_1));
						} else if(key == IR_Key_FanUp) { // Fan
							if(Setup == fSetup_Speed) goto xFanSpeedUp;
							else if(FanOn && FanOn <= FAN_IDXS) FanSpeedUp(); else SetFanSpeed(GET_FANS_IDX_EEPROM(&EEPROM.FanSpeed_3));
						} else if(key == IR_Key_FanDown) { // Fan
							if(Setup == fSetup_Speed) goto xFanSpeedDown;
							else if(FanOn && FanOn <= FAN_IDXS) FanSpeedDown(); else SetFanSpeed(GET_FANS_IDX_EEPROM(&EEPROM.FanSpeed_1));
						} else if(key == IR_Key_CookerLight) {
							CookerLight_force_on ^= 1;
							goto xLampForce;
						}
					}
					break;
				}
			}
			if(Setup == fSetup_IR && i == remotes_max) { // New remote command
xSetupIR_New:				
				i = eeprom_read_byte(&EEPROM.IRRemotes);
				eeprom_update_word(&EEPROM.IRCommandArray[i * IR_Keys_Total + SetupItem], IR_Hash);
				Set_LED_Warning_New(0);
				if(SetupItem < IR_Keys_Total - 1) {
					SetupItem++;
					Timer = 60;
				} else { // Finish, next remote
					eeprom_update_byte(&EEPROM.IRRemotes, ++i);
					if(i >= IR_MAX_CONTROLS) { // not enough space
						goto xSetup_finish;
					} else goto xStartSetupIR;
				}
			}
		}
		if(Key1Pressed) {
			if(Key1Pressed > 40) {
				LED1_ON;
				while(KEY_PRESSING) {
					__asm__ volatile ("" ::: "memory"); // Need memory barrier
					wdt_reset();
				}
				if(Key1Pressed >= 120) { // reset settings - pressed > 12 sec
					ResetSettings();
					goto xReset;
				} else {
xStartSetupIR:
					// Setup IR Commands
					FlashLED(3, 1, 1);
					Delay100ms(15);
					Key1Pressed = 0;
					uint8_t i = eeprom_read_byte(&EEPROM.IRRemotes);
					if(i >= IR_MAX_CONTROLS) {
						FlashLED(50, 1, 1);
						if(Key1Pressed) eeprom_update_byte(&EEPROM.IRRemotes, 0); else continue;
					}
					LED_Warning = (i + 1) << 4;
					IR_Status = IR_WAITING;
					Setup = fSetup_IR;
					SetupItem = 0;
					Timer = 60;
				}
			} else if(!KEY_PRESSING) { // Manual speed - pressed < 2 sec
				uint8_t i = eeprom_read_byte(&EEPROM.SpeedKeyIdx);
				SetFanSpeed(/* FanOn == i ? 0 : */ i);
				Key1Pause = 255;	// 2.5 sec
				Key1Pressed = 0;
			}
		}
		if(Flags & f_NRF24) {
			if(NRF24_Receive((uint8_t*)&data)) {
#ifndef DEBUG_PROTEUS
				LED_WarningPause = 2;
				LED1_ON;
#endif
				struct SETUP_DATA *p = (struct SETUP_DATA*)&data;
				register uint8_t cmd = p->Flags;
				if(cmd == fCMD_Set) { // SET command
					int8_t d = p->Data;
					register uint8_t type = p->Type;
					if(type == Type_Set_RESET) {
						if(p->Data != 0xEEEE) continue;
xReset:
						LED1_ON_BEFORE_RESET;
						cli(); while(1) ; // restart
					} else if(type == Type_Set_Fan) { // Set Fan Index
						if(d <= FAN_IDXS * 2) {
							SetFanSpeed(d);
							if(d == 0) FlashLED(3, 2, 2);
						}
					} else if(type == Type_Set_FanSpeedUp) { // FanSpeed +1
xFanSpeedUp:
						if(FanSpeed < OutPeriod) FanSpeed++;
						LED_Warning = FanSpeed;
						Timer = 255;
					} else if(type == Type_Set_FanSpeedDown) { // FanSpeed -1
xFanSpeedDown:
						if(FanSpeed > 1) FanSpeed--;
						LED_Warning = FanSpeed;
						Timer = 255;
 					} else if(type == Type_Set_FanSpeedSave) { // Save to current fan idx
 						if(FanOn) {
 							eeprom_update_byte(FANS_IDX_EEPROM_OFFSET - 1 + FanOn, FanSpeed);
 						}
 						Timer = 255;
					} else if(type == Type_Set_Lamp) { // Switch Lamp
						CookerLight_force_on = d;
xLampForce:
						if(CookerLight_force_on) {
							if(Flags & f_CookerLamp_FanOut2) Fan_Out2_ON;
							else if(Flags & f_CookerLamp_CookerOut2) FanCooker_Out2_ON;
						} else {
							if(Flags & f_CookerLamp_FanOut2) Fan_Out2_OFF;
							else if(Flags & f_CookerLamp_CookerOut2) FanCooker_Out2_OFF;
						}
					} else if(type == Type_Set_SetupIR) { // Enter Setup IR mode
						goto xStartSetupIR;
					}
				} else if(cmd == fCMD_WriteStart) {
					WriteTimeout = fCMD_Write_Timeout;
				} else if(cmd & fCMD_Read) { // READ command
					register uint8_t cmd2 = cmd & 0xF0;
					cmd &= 0x0F;
					if(cmd2 == fCMD_Read + fCMD_EEPROM) {
						if(p->Data < sizeof(struct _EEPROM)) {
							if(cmd == fCMD_2b) p->Data = eeprom_read_word((uint16_t*)((uint8_t*)&EEPROM + p->Data));
							else p->Data = eeprom_read_byte((uint8_t*)&EEPROM + p->Data);
							//p->Type = p->Flags = 0;
						} else continue;
					} else if(cmd2 == fCMD_Read + fCMD_RAM) {
						ATOMIC_BLOCK(ATOMIC_FORCEON) {
							if(cmd == fCMD_2b) p->Data = *((uint16_t *)p->Data);
							else p->Data = *((uint8_t *)p->Data);
						}
						//p->Type = p->Flags = 0;
					} else if(cmd2 == fCMD_Read + fCMD_PROGMEM) {
						if(cmd == fCMD_2b) p->Data = pgm_read_word(p->Data);
						else if(cmd == fCMD_1b) p->Data = pgm_read_byte(p->Data);
						//p->Type = p->Flags = 0;
					} else continue;
					Delay100ms(1);
					NRF24_SetMode(NRF24_TransmitMode);
					uint8_t err = 0;
					if(cmd == fCMD_CStr) {
						for(uint8_t i = 0; i < sizeof(ProgramID); i++) {
							p->Data = pgm_read_byte(&ProgramID[i]);
							err = NRF24_Transmit((uint8_t *)&data);
							if(err) break;
							Delay10us(255);
						}
					} else err = NRF24_Transmit((uint8_t *)&data);
					NRF24_SetMode(NRF24_ReceiveMode);
					if(err) Set_LED_Warning_New(WRN_SETUP_ERR + err);
					//Timer = 120; // sec
				} else if(cmd & fCMD_Write) { // WRITE command
					if(WriteTimeout) {
						register uint8_t cmd2 = cmd & 0x30;
						cmd &= 0x0F;
						if(cmd2 == fCMD_RAM) {
							ATOMIC_BLOCK(ATOMIC_FORCEON) {
								if(cmd == fCMD_2b) *((uint16_t *)(uint16_t)p->Type) = p->Data;
								else *((uint8_t *)(uint16_t)p->Type) = p->Data;
							}
						} else if(cmd2 == fCMD_EEPROM) {
							if(cmd == fCMD_2b) {
								eeprom_update_word((uint16_t*)((uint8_t*)&EEPROM + p->Type), p->Data);
							} else {
								eeprom_update_byte((uint8_t*)&EEPROM + p->Type, p->Data);
							}
							GetSettings();
							Set_LED_Warning_New(WRN_SETUP_INFO);
						}
						WriteTimeout = fCMD_Write_Timeout;
					}
					//Timer = 120; // sec
				} else { // CO2 received
					if(SleepTimer == 0 && Timer == 0) {
						SetFanSpeed_by_CO2 = 1;
						uint8_t fs;
						if(data.CO2level > eeprom_read_word(&EEPROM.CO2_threshold3)) fs = 3;
						else if(data.CO2level > eeprom_read_word(&EEPROM.CO2_threshold2)) fs = 2;
						else if(data.CO2level > eeprom_read_word(&EEPROM.CO2_threshold1)) fs = 1;
						else fs = 0;
						SetFanSpeed(fs);
						SetFanSpeed_by_CO2 = 0;
						Timer = 255;
					}
				}
			}
		}
	}
}

// Check connected SSR relay
/*	FanCooker_PORT |= FanCooker_Speed1;
	Delay10us(100);
	FanCookerPresent = (FanCooker_PIN & FanCooker_Speed1) != 0;
	FanCooker_PORT &= ~FanCooker_Speed1;
	FanCooker_PORT |= FanCooker_Speed2;
	Delay10us(100);
	FanCookerPresent |= ((FanCooker_PIN & FanCooker_Speed2) != 0) << 1;
	FanCooker_PORT &= ~FanCooker_Speed2;
	Fan_PORT |= Fan_Speed1;
	Delay10us(100);
	FanPresent = (Fan_PIN & Fan_Speed1) != 0;
	Fan_PORT &= ~Fan_Speed1;
	Fan_PORT |= Fan_Speed2;
	Delay10us(100);
	FanPresent |= ((Fan_PIN & Fan_Speed2) != 0) << 1;
	Fan_PORT &= ~Fan_Speed2;
	FlashLED((FanPresent == 3 ? 2 : FanPresent ? 1 : 0) + (FanCookerPresent == 3 ? 2 : FanCookerPresent ? 1 : 0), 3, 3);
*/
