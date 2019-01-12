/* Name: main.c
 * Project: hid-mouse, a very simple HID example
 * Author: Christian Starkjohann
 * Creation Date: 2008-04-07
 * Tabsize: 4
 * Copyright: (c) 2008 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 */

/*
This example should run on most AVRs with only little changes. No special
hardware resources except INT0 are used. You may have to change usbconfig.h for
different I/O pins for USB. Please note that USB D+ must be the INT0 pin, or
at least be connected to INT0 as well.

We use VID/PID 0x046D/0xC00E which is taken from a Logitech mouse. Don't
publish any hardware using these IDs! This is for demonstration only!
*/

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>     /* for _delay_ms() */

#include <avr/pgmspace.h>   /* required by usbdrv.h */
#include "usbdrv.h"
//#include "oddebug.h"        /* This is also an example for using debug macros */

#define cbi(addr,bit)     addr &= ~(1<<bit)
#define sbi(addr,bit)     addr |=  (1<<bit)
//#define tbi(addr,bit)	  addr ^=  (1<<bit)

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

PROGMEM const char usbHidReportDescriptor[45] = {
	0x05, 0x01,			// USAGE_PAGE (Generic Desktop)
	0x09, 0x06,			// USAGE (Keyboard)
	0xa1, 0x01,			// COLLECTION (Application)
	0x05, 0x07,			//   USAGE_PAGE (Keyboard)
	0x19, 0xe0,			//   USAGE_MINIMUM (Keyboard LeftControl)
	0x29, 0xe7,			//   USAGE_MAXIMUM (Keyboard Right GUI)
	0x15, 0x00,			//   LOGICAL_MINIMUM (0)
	0x25, 0x01,			//   LOGICAL_MAXIMUM (1)
	0x75, 0x01,			//   REPORT_SIZE (1)
	0x95, 0x08,			//   REPORT_COUNT (8)
	0x81, 0x02,			//   INPUT (Data,Var,Abs)
	0x95, 0x01,			//   REPORT_COUNT (1)
	0x75, 0x08,			//   REPORT_SIZE (8)
	0x81, 0x03,			//   INPUT (Cnst,Var,Abs)

	0x95, 0x06,			//   REPORT_COUNT (6)
	0x75, 0x08,			//   REPORT_SIZE (8)
	0x15, 0x00,			//   LOGICAL_MINIMUM (0)
	0x25, 0x65,			//   LOGICAL_MAXIMUM (101)
	0x05, 0x07,			//   USAGE_PAGE (Keyboard)
	0x19, 0x00,			//   USAGE_MINIMUM (Reserved (no event indicated))
	0x29, 0x65,			//   USAGE_MAXIMUM (Keyboard Application)
	0x81, 0x00,			//   INPUT (Data,Ary,Abs)
	0xc0				// End Collection
};
typedef struct{
    //uchar   reportID;
	uchar	modifier;
	uchar	reserved;
	uchar	keyCodes[6];
}report_t;

static report_t reportBuffer;
static uchar    idleRate;   /* repeat rate for keyboards, never used for mice */

static signed char delta = 0;
static unsigned char PIRCount = 0;

static void resetReportBuffer(report_t* r){
	//r->reportID = 0x02;
	//r->modifier = 0;
	//r->reserved = 0;
	for(int i=0;i<6;i++){
		r->keyCodes[i] = 0;
	}
}

static void addKeyCode(report_t* r, uchar keyCode){
	for(int i=0;i<6;i++){
		if( r->keyCodes[i] == 0) {
			r->keyCodes[i] = keyCode;
			return;
		}
	}
}

/* ------------------------------------------------------------------------- */

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

    /* The following requests are never used. But since they are required by
     * the specification, we implement them in this example.
     */
    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
        //DBG1(0x50, &rq->bRequest, 1);   /* debug output: print our request */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            /* we only have one report type, so don't look at wValue */
            usbMsgPtr = (void *)&reportBuffer;
            return sizeof(reportBuffer);
        }else if(rq->bRequest == USBRQ_HID_GET_IDLE){
            usbMsgPtr = &idleRate;
            return 1;
        }else if(rq->bRequest == USBRQ_HID_SET_IDLE){
            idleRate = rq->wValue.bytes[1];
        }
    }else{
        /* no vendor specific requests implemented */
    }
    return 0;   /* default for not implemented requests: return no data back to host */
}

/* ------------------------------------------------------------------------- */

ISR(TIMER0_COMPA_vect) {
	
	//ロータリーエンコーダー処理
	static const int dir[] = { 0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0 }; /* 回転方向テーブル */
	static int i;//インデックス
	int n;

	i = (i << 2) | (bit_is_clear(PIND,PD3)<<1) | bit_is_clear(PIND,PD4);   /* 前回値と今回値でインデックスとする */
	n = dir[i & 0x0F];
	delta += n;
	
	//人感センサー処理
	if(bit_is_clear(PIND,PD7)) {
		if(PIRCount < 0xFF) PIRCount++;
	}
	else PIRCount = 0;
}

int __attribute__((noreturn)) main(void)
{
	//sbi(DDRD, PD6);
	//cbi(PORTD, PD6);
	
	//Buttons
	cbi(DDRB,PB1);
	cbi(DDRB,PB2);//PB0,1入力
	sbi(PORTB,PB1);
	sbi(PORTB,PB2);//PB0,1内部プルアップ有効
	
	//Rotary init
	cbi(DDRD,PD3);//PD3入力
	cbi(DDRD,PD4);//PD5入力
	cbi(PORTD,PD3);//PD3内部プルアップ **無効**
	sbi(PORTD,PD4);//PD5内部プルアップ
	
	//PIR Sensor
	cbi(DDRD, PD7);//PD7入力
	cbi(PORTD, PD7);//PD7内部プルアップ無効
	
	/**
	 * timer interrupt
	 * CTC 16MHz / 1024 / 127 = 120Hzくらい
	 */
	TCCR0A = 0b00000010;
	TCCR0B = 0b00000011;
	OCR0A  = 144;
	//TIMSK0 = 0b00000010;
	sbi(TIMSK0,OCIE0A);
	
	//USB init
	uchar   i;

    wdt_enable(WDTO_1S);
    /* Even if you don't use the watchdog, turn it off here. On newer devices,
     * the status of the watchdog (on/off, period) is PRESERVED OVER RESET!
     */
    /* RESET status: all port bits are inputs without pull-up.
     * That's the way we need D+ and D-. Therefore we don't need any
     * additional hardware initialization.
     */
    //odDebugInit();
    //DBG1(0x00, 0, 0);       /* debug output: main starts */
    usbInit();
    usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
    i = 0;
    while(--i){             /* fake USB disconnect for > 250 ms */
        wdt_reset();
        _delay_ms(1);
    }
    usbDeviceConnect();
    sei();
    //DBG1(0x01, 0, 0);       /* debug output: main loop starts */
	//USB init end
	//unsigned char j = 0;
    for(;;){                /* main event loop */
		//USB loop start
        //DBG1(0x02, 0, 0);   /* debug output: main loop iterates */
        wdt_reset();
        usbPoll();
		
        if(usbInterruptIsReady()){
            /* called after every poll of the interrupt endpoint */
            //advanceCircleByFixedAngle();
			
            //DBG1(0x03, 0, 0);   /* debug output: interrupt report prepared */
			//j++;
			resetReportBuffer(&reportBuffer);
			if(delta > 3) {
				addKeyCode(&reportBuffer, 26);//W
				delta %= 4;
			}
			else if(delta < -3) {
				addKeyCode(&reportBuffer, 22);//S
				delta = -(-delta % 4);
			}
			if(bit_is_clear(PINB,PB1)) addKeyCode(&reportBuffer,4); //戻るボタンはA
			if(bit_is_clear(PINB,PB2)) addKeyCode(&reportBuffer,7); //決定ボタンはD
			if(PIRCount > 200 && PIRCount < 0xFF) addKeyCode(&reportBuffer,20); //人感センサーはQ
            usbSetInterrupt((void *)&reportBuffer, sizeof(reportBuffer));
			//delta = 0;
        }
		//USB loop end
    }
}

/* ------------------------------------------------------------------------- */
