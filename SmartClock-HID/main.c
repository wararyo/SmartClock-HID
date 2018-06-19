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
#include "oddebug.h"        /* This is also an example for using debug macros */

#define cbi(addr,bit)     addr &= ~(1<<bit)
#define sbi(addr,bit)     addr |=  (1<<bit)
#define tbi(addr,bit)	  addr ^=  (1<<bit)

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

PROGMEM const char usbHidReportDescriptor[118] = { /* USB report descriptor, size must match usbconfig.h */
    0x05, 0x01,        // USAGE_PAGE (Generic Desktop)
    0x09, 0x02,        // USAGE (Mouse)
    0xa1, 0x01,        // COLLECTION (Application)
    0x09, 0x02,        //   USAGE (Mouse)
    0xa1, 0x02,        //   COLLECTION (Logical)
    0x09, 0x01,        //     USAGE (Pointer)
    0xa1, 0x00,        //     COLLECTION (Physical)
    // ------------------------------  Buttons
    0x05, 0x09,        //       USAGE_PAGE (Button)
    0x19, 0x01,        //       USAGE_MINIMUM (Button 1)
    0x29, 0x05,        //       USAGE_MAXIMUM (Button 5)
    0x15, 0x00,        //       LOGICAL_MINIMUM (0)
    0x25, 0x01,        //       LOGICAL_MAXIMUM (1)
    0x75, 0x01,        //       REPORT_SIZE (1)
    0x95, 0x05,        //       REPORT_COUNT (5 Buttons)
    0x81, 0x02,        //       INPUT (Data,Var,Abs)
    // ------------------------------  Padding
    0x75, 0x03,        //       REPORT_SIZE (8-5buttons 3)
    0x95, 0x01,        //       REPORT_COUNT (1)
    0x81, 0x03,        //       INPUT (Cnst,Var,Abs)
    // ------------------------------  X,Y position
    0x05, 0x01,        //       USAGE_PAGE (Generic Desktop)
    0x09, 0x30,        //       USAGE (X)
    0x09, 0x31,        //       USAGE (Y)
    0x15, 0x81,        //       LOGICAL_MINIMUM (-127)
    0x25, 0x7f,        //       LOGICAL_MAXIMUM (127)
    0x75, 0x08,        //       REPORT_SIZE (8)
    0x95, 0x02,        //       REPORT_COUNT (2)
    0x81, 0x06,        //       INPUT (Data,Var,Rel)
    0xa1, 0x02,        //       COLLECTION (Logical)
    // ------------------------------  Vertical wheel res multiplier
    0x09, 0x48,        //         USAGE (Resolution Multiplier)
    0x15, 0x00,        //         LOGICAL_MINIMUM (0)
    0x25, 0x01,        //         LOGICAL_MAXIMUM (1)
    0x35, 0x01,        //         PHYSICAL_MINIMUM (1)
    0x45, 0x01,        //         PHYSICAL_MAXIMUM (1)
    0x75, 0x02,        //         REPORT_SIZE (2)
    0x95, 0x01,        //         REPORT_COUNT (1)
    0xa4,              //         PUSH
    0xb1, 0x02,        //         FEATURE (Data,Var,Abs)
    // ------------------------------  Vertical wheel
    0x09, 0x38,        //         USAGE (Wheel)
    0x15, 0x81,        //         LOGICAL_MINIMUM (-127)
    0x25, 0x7f,        //         LOGICAL_MAXIMUM (127)
    0x35, 0x00,        //         PHYSICAL_MINIMUM (0)        - reset physical
    0x45, 0x00,        //         PHYSICAL_MAXIMUM (0)
    0x75, 0x08,        //         REPORT_SIZE (8)
    0x81, 0x06,        //         INPUT (Data,Var,Rel)
    0xc0,              //       END_COLLECTION
    0xa1, 0x02,        //       COLLECTION (Logical)
    // ------------------------------  Horizontal wheel res multiplier
    0x09, 0x48,        //         USAGE (Resolution Multiplier)
    0xb4,              //         POP
    0xb1, 0x02,        //         FEATURE (Data,Var,Abs)
    // ------------------------------  Padding for Feature report
    0x35, 0x00,        //         PHYSICAL_MINIMUM (0)        - reset physical
    0x45, 0x00,        //         PHYSICAL_MAXIMUM (0)
    0x75, 0x04,        //         REPORT_SIZE (4)
    0xb1, 0x03,        //         FEATURE (Cnst,Var,Abs)
    // ------------------------------  Horizontal wheel
    0x05, 0x0c,        //         USAGE_PAGE (Consumer Devices)
    0x0a, 0x38, 0x02,  //         USAGE (AC Pan)
    0x15, 0x81,        //         LOGICAL_MINIMUM (-127)
    0x25, 0x7f,        //         LOGICAL_MAXIMUM (127)
    0x75, 0x08,        //         REPORT_SIZE (8)
    0x81, 0x06,        //         INPUT (Data,Var,Rel)
    0xc0,              //       END_COLLECTION
    0xc0,              //     END_COLLECTION
    0xc0,              //   END_COLLECTION
    0xc0               // END_COLLECTION
};


 //
 // Wheel Mouse - simplified version
 //
 // Input report - 5 bytes
 //
 //     Byte | D7      D6      D5      D4      D3      D2      D1      D0
 //    ------+---------------------------------------------------------------------
 //      0   |  0       0       0    Forward  Back    Middle  Right   Left (Button)
 //      1   |                             X
 //      2   |                             Y
 //      3   |                       Vertical Wheel
 //      4   |                    Horizontal (Tilt) Wheel
 //
 // Feature report - 1 byte
 //
 //     Byte | D7      D6      D5      D4   |  D3      D2  |   D1      D0
 //    ------+------------------------------+--------------+----------------
 //      0   |  0       0       0       0   |  Horizontal  |    Vertical
 //                                             (Resolution multiplier)
 //
 // Reference
 //    http://www.microchip.com/forums/m391435.aspx
 //
 
typedef struct{
    uchar   buttonMask;
    char    dx;
    char    dy;
    char    dWheel;
	char	dPan;//Horizontal Wheel
}report_t;

static report_t reportBuffer;
static int      sinus = 7 << 6, cosinus = 0;
static uchar    idleRate;   /* repeat rate for keyboards, never used for mice */

static signed char delta = 0;


/* The following function advances sin/cos by a fixed angle
 * and stores the difference to the previous coordinates in the report
 * descriptor.
 * The algorithm is the simulation of a second order differential equation.
 */
static void advanceCircleByFixedAngle(void)
{
char    d;

#define DIVIDE_BY_64(val)  (val + (val > 0 ? 32 : -32)) >> 6    /* rounding divide */
    reportBuffer.dx = d = DIVIDE_BY_64(cosinus);
    sinus += d;
    reportBuffer.dy = d = DIVIDE_BY_64(sinus);
    cosinus -= d;
}

static void resetReportBuffer(report_t* r){
	r->buttonMask = 0;
	r->dx = 0;
	r->dy = 0;
	r->dWheel = 0;
	r->dPan = 0;
}

/* ------------------------------------------------------------------------- */

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

    /* The following requests are never used. But since they are required by
     * the specification, we implement them in this example.
     */
    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
        DBG1(0x50, &rq->bRequest, 1);   /* debug output: print our request */
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

ISR(INT1_vect)//ロリコンA変化
{
	if(delta != 0) return;
	_delay_ms(1);   //チャタリング防止
	cbi(EIFR,INTF1); //チャタリング防止の後で、割り込みフラグをクリア
	if (bit_is_set(PIND,3)) { // エンコーダ出力AがやっぱりHighのとき
		if (bit_is_clear(PIND,5)) { // エンコーダ出力BがLowのとき
				delta++; //時計回り
			} else {  // エンコーダ出力BがHighのとき
				delta--; //反時計回り
		}
	}
}

int __attribute__((noreturn)) main(void)
{
	sbi(DDRD, PD6);
	cbi(PORTD, PD6);
	
	cbi(DDRB,PB0);
	cbi(DDRB,PB1);//PB0,1入力
	sbi(PORTB,PB0);
	sbi(PORTB,PB1);//PB0,1内部プルアップ有効
	
	//Rotary init
	cbi(DDRD,PD3);//PD3入力
	cbi(DDRD,PD5);//PD5入力
	sbi(PORTD,PD3);//PD3内部プルアップ
	sbi(PORTD,PD5);//PD5内部プルアップ
	
	sbi(MCUCR, ISC11);
	cbi(MCUCR, ISC11);//INT1立ち上がり
	sbi(GIMSK,INT1);//INT1割り込み許可
	
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
    odDebugInit();
    DBG1(0x00, 0, 0);       /* debug output: main starts */
    usbInit();
    usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
    i = 0;
    while(--i){             /* fake USB disconnect for > 250 ms */
        wdt_reset();
        _delay_ms(1);
    }
    usbDeviceConnect();
    sei();
    DBG1(0x01, 0, 0);       /* debug output: main loop starts */
	//USB init end
	//unsigned char j = 0;
    for(;;){                /* main event loop */
		//USB loop start
        DBG1(0x02, 0, 0);   /* debug output: main loop iterates */
        wdt_reset();
		
		cbi(GIMSK,INT1);//INT1割り込みやめて
        usbPoll();
		sbi(GIMSK,INT1);//INT1割り込み許可
		
        if(usbInterruptIsReady()){
			cbi(GIMSK,INT1);//INT1割り込みやめて
            /* called after every poll of the interrupt endpoint */
            //advanceCircleByFixedAngle();
			
			if(bit_is_set(PIND,3)) sbi(PORTD,PD6);
			else cbi(PORTD,PD6);
			
            DBG1(0x03, 0, 0);   /* debug output: interrupt report prepared */
			//j++;
			reportBuffer.dWheel = delta;
			if(bit_is_set(PINB,PB0)) cbi(reportBuffer.buttonMask,0);//戻るボタンは左クリック
			else sbi(reportBuffer.buttonMask, 0);
			if(bit_is_set(PINB,PB1)) cbi(reportBuffer.buttonMask,1);//決定ボタンは右クリック
			else sbi(reportBuffer.buttonMask, 1);
            usbSetInterrupt((void *)&reportBuffer, sizeof(reportBuffer));
			//resetReportBuffer(&reportBuffer);
			delta=0;
			//tbi(PORTB,PB0);
			sbi(GIMSK,INT1);//INT1割り込み許可
        }
		//USB loop end
    }
}

/* ------------------------------------------------------------------------- */
