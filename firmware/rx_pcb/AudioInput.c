/*
  This file implements the receiver part of the wireless guitar system
  written by Wojciech M. Zabolotny ( wzab<at>ise.pw.edu.pl )
  Copyright (C) Wojciech Zabolotny, 2011
  Please treat my modifications as PUBLIC DOMAIN, however if you
  use them, please mention it in documentation of your software
  and/or device (this is a suggestion, not the obligation).

  I do not provide any warranty on my modifications!
  You may use it only on your own risk!

  Please note, that the below code was based on the AudioInput.c
  and MIDI.c file written by Dean Camera, with licence terms described below.
*/
/*
  LUFA Library
  Copyright (C) Dean Camera, 2010.

  dean [at] fourwalledcubicle [dot] com
  www.lufa-lib.org
*/

/*
  Copyright 2010  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the AudioInput demo. This file contains the main tasks of
 *  the demo and is responsible for the initial application hardware configuration.
 */

#include "AudioInput.h"
#include "rfm70.h"
/** LUFA Audio Class driver interface configuration and state information. This structure is
 *  passed to all Audio Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_Audio_Device_t Microphone_Audio_Interface =
  {
    .Config =
    {
      .StreamingInterfaceNumber = 1,

      .DataINEndpointNumber     = AUDIO_STREAM_EPNUM,
      .DataINEndpointSize       = AUDIO_STREAM_EPSIZE,
    },
  };

//Pasted by WZab section
#include "MIDI.h"

/** LUFA MIDI Class driver interface configuration and state information. This structure is
 *  passed to all MIDI Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_MIDI_Device_t Keyboard_MIDI_Interface =
  {
    .Config =
    {
      .StreamingInterfaceNumber = 2,

      .DataINEndpointNumber      = MIDI_STREAM_IN_EPNUM,
      .DataINEndpointSize        = MIDI_STREAM_EPSIZE,
      .DataINEndpointDoubleBank  = false,

      .DataOUTEndpointNumber     = MIDI_STREAM_OUT_EPNUM,
      .DataOUTEndpointSize       = MIDI_STREAM_EPSIZE,
      .DataOUTEndpointDoubleBank = false,
    },
  };
// End of pasted by WZab section
void T4_Init(void);

#define PKT_TIMEOUT 10
volatile uint8_t timer_count=0;
// Timer routines - needed automated detection of lost connection 
// (e.g. due to channel change in the transmitter)
ISR(TIMER1_COMPA_vect, ISR_BLOCK)
{
  if(timer_count) timer_count--;
}
  
static inline void start_timer(int16_t val)
{
  OCR1A = val;
  TCCR1A = 0;
  TCCR1B = (0<<WGM13) | (1 << WGM12) | (1 << CS10);
  TIMSK1 = (1<<OCIE1A);
}

static inline void set_timer(int16_t val)
{
  cli();
  timer_count = val;
  sei();
}

#define PORT_OVR PORTD
#define PIN_NR_OVR 5
#define DDR_OVR DDRD

#define PORT_PKTLST PORTD
#define PIN_NR_PKTLST 4
#define DDR_PKTLST DDRD

#define PORT_PKTRCV PORTC
#define PIN_NR_PKTRCV 7
#define DDR_PKTRCV DDRC

#define PORT_BEEPER PORTD
#define PIN_NR_BEEPER 1
#define DDR_BEEPER DDRD

volatile uint8_t overrun_count=0;
volatile uint8_t packet_lost_count=0;
volatile uint8_t packet_rcvd_count=0;
// Timer routines - needed for servicing of LEDS signaling 
// operation of the link
ISR(TIMER3_COMPA_vect, ISR_BLOCK)
{
  if(overrun_count) {
    overrun_count--;
    if(overrun_count==0) PORT_OVR &= ~(1<<PIN_NR_OVR); //Turn off the overrun diode
  }
  if(packet_lost_count) {
    packet_lost_count--;
    if(packet_lost_count==0) PORT_PKTLST &= ~(1<<PIN_NR_PKTLST); //Turn off the packet lost diode
  }
  if(packet_rcvd_count) {
    packet_rcvd_count--;
    if(packet_rcvd_count==0) PORT_PKTRCV &= ~(1<<PIN_NR_PKTRCV); //Turn off the packet lost diode
  }
}
  
static inline void start_timer3(int16_t val)
{
  OCR3A = val;
  TCCR3A = 0;
  TCCR3B = (0<<WGM33) | (1 << WGM32) | (1 << CS30);
  TIMSK3 = (1<<OCIE3A);
}

static inline void set_timer3(int16_t val)
{
  cli();
  timer_count = val;
  sei();
}

void beep(uint8_t msg)
{
  uint8_t mask;
  DDR_BEEPER |= (1<<PIN_NR_BEEPER);
  {
    mask=0x80;
    while(mask) {
      if(mask & msg)
	PORT_BEEPER |= (1<<PIN_NR_BEEPER);
      else
        PORT_BEEPER &= ~(1<<PIN_NR_BEEPER);
      _delay_ms(200);
      mask >>= 1;
    }
    PORT_BEEPER &= ~(1<<PIN_NR_BEEPER);
    _delay_ms(400);
  }
}

void show_error(uint8_t msg)
{
  uint8_t mask;
  DDR_BEEPER |= (1<<PIN_NR_BEEPER);
  while(1) {
    mask=0x80;
    while(mask) {
      if(mask & msg)
	PORT_BEEPER |= (1<<PIN_NR_BEEPER);
      else
        PORT_BEEPER &= ~(1<<PIN_NR_BEEPER);
      _delay_ms(200);
      mask >>= 1;
    }
    PORT_BEEPER &= ~(1<<PIN_NR_BEEPER);
    _delay_ms(400);
  }
}

#include "abuf.h"
#include "byte_select.h"

volatile int32_t abuf[ABUF_LEN];
volatile uint8_t abuf_wptr = 0;
volatile uint8_t abuf_rptr = 0;
volatile uint8_t abuf_overrun = 0;

static inline bool abuf_is_empty(void)
{
  if (abuf_rptr == abuf_wptr) return true;
  return false;
}

static inline void abuf_put(int32_t smp)
{
  uint8_t new_wptr = (abuf_wptr+1) & (ABUF_LEN-1);
  if(new_wptr == abuf_rptr) {
    overrun_count = 200;
    PORT_OVR |= (1<<PIN_NR_OVR);
    return;
  }
  abuf[abuf_wptr] = smp;
  abuf_wptr = new_wptr;
}

static inline int32_t abuf_get(void)
{
  int32_t res = abuf[abuf_rptr];
  cli();
  abuf_rptr = (abuf_rptr+1) & (ABUF_LEN-1);
  sei();
  return res; //(res + res);
}

uint8_t rf_channel = 0;
uint8_t adc_batt = 0;

#define WGS_NUM_OF_SWITCHES 4
volatile uint8_t switches = 0;
uint8_t oldswitches = 0;
static unsigned char switch_on_msg[WGS_NUM_OF_SWITCHES][4] = {{0x0c,0xc0,0x01,0x00},
							      {0x0c,0xc0,0x02,0x00},
							      {0x0c,0xc0,0x03,0x00},
							      {0x0c,0xc0,0x04,0x00}};
static unsigned char switch_off_msg[WGS_NUM_OF_SWITCHES][4] = {[0 ... WGS_NUM_OF_SWITCHES-1]={0x0,0x0,0x0,0}};

#define WGS_NUM_OF_POTS 3
volatile uint8_t adc_val[WGS_NUM_OF_POTS] = {0,0,0};
int8_t old_adc_val[WGS_NUM_OF_POTS] = {0,0,0};
static unsigned char pot_msg[WGS_NUM_OF_POTS][4] = {{0x0b,0xb0,0x50,0},{0x0b,0xb0,0x51,0},{0x0b,0xb0,0x52,0}};
//Structure used to generate a message when controller changes its state


ISR(INT3_vect, ISR_BLOCK)
{
  static uint8_t old_pkt_nr =0;
  //Receiver's interrupt
  //Mark reception of the packet
  timer_count = 4*PKT_TIMEOUT; //This value should not be higher than 255!
  //Disable the INT3 interrupt
  EIMSK &= ~(1<<INT3);
  EIFR  = (1<<INTF3);
  PORT_PKTRCV |= (1<<PIN_NR_PKTRCV) ; //Show that packet arrived
  packet_rcvd_count = 100;
  sei(); //Enable interrupts
  while(1) {
    uint8_t i, pkt_nr;
    union{
      int32_t i;
      uint8_t b[4];
    } smp1,smp2;
    //The whole payload is received, cancel the interrupt
    write_reg(0x7,0x40);
    if((read_reg(0x17) & 0x01) == 1) {
      break;
    }
    set_cs(0);
    SPI_Transfer(0x61);
    pkt_nr=SPI_Transfer(0x00); //read packet number
    // Check ifany packet is lost and signal it
    // Please note that "pkt_nr-old_pkt_nr !=1" 
    // generates wrong code. You have to cast result to uint8_t
    if((uint8_t)(pkt_nr-old_pkt_nr) != 1) {
      packet_lost_count = 100;
      PORT_PKTLST |= (1<<PIN_NR_PKTLST);
    }
    old_pkt_nr = pkt_nr;
    // Copy the data!
    for(i=0;i<6;i++) {
      smp1.b[3]= SPI_Transfer(0x00); 
      smp1.b[2]= SPI_Transfer(0x00);
      smp2.b[3]= SPI_Transfer(0x00);
      smp2.b[2]= SPI_Transfer(0x00);
      smp1.b[1]= SPI_Transfer(0x00);
      smp1.b[0]= 0x00;
      smp2.b[0]= 0x00;
      //Unpack nibbles
      smp2.b[1]= (smp1.b[1] & 0x0f) << 4;
      smp1.b[1] &= 0xf0;
      abuf_put(smp1.i);
      abuf_put(smp2.i);
    }
    i = SPI_Transfer(0x00);
    switch (pkt_nr & 0x07) {
    case 0x01:
      switches = i;
      break;
    case 0x02:
      adc_batt = i; // battery state
      break;
    case 0x03:
      adc_val[0] = i;
      break;
    case 0x04:
      adc_val[1] = i;
      break;
    case 0x05:
      adc_val[2] = i;
      break;
    }
    set_cs(1);
  }
  cli(); // Avoid nesting of the interrupt!
  EIMSK |= (1<<INT3);
}

static inline void WZab_Device_WriteSample24(USB_ClassInfo_Audio_Device_t* const AudioInterfaceInfo,
					     uint32_t Sample) ATTR_NON_NULL_PTR_ARG(1) ATTR_ALWAYS_INLINE;

										       static inline void WZab_Device_WriteSample24(USB_ClassInfo_Audio_Device_t* const AudioInterfaceInfo,
																    uint32_t Sample)
{
  Endpoint_Write_Byte(uint32_b1(Sample));
  Endpoint_Write_Byte(uint32_b2(Sample));
  Endpoint_Write_Byte(uint32_b3(Sample));
  
  if (Endpoint_BytesInEndpoint() == AudioInterfaceInfo->Config.DataINEndpointSize)
    Endpoint_ClearIN();
}

#ifdef WGS_EMULATE_48KHZ
int32_t smp_old = 0;
#endif

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
  uint8_t rfm_state = 0;
  uint16_t sample_nr = 0;
  SetupHardware();

  //LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
  sei();
  set_timer(PKT_TIMEOUT);
  start_timer(3199); //400us timer 
  start_timer3(31999); //2ms timer for overrun detection
  for (;;)
    { 
      // Main loop
      {
#ifdef WGS_EMULATE_48KHZ
	register int32_t smp1b = 0;
#endif
	//Highest priority - receive audio samples and pass them to the isochronous endpoint
	uint8_t PrevEndpoint = Endpoint_GetCurrentEndpoint();
	/* Check if the sample reload timer period has elapsed, and that the USB bus is ready for a new sample */
	if ((abuf_is_empty()==false) && Audio_Device_IsReadyForNextSample(&Microphone_Audio_Interface))
	  {
	    int32_t AudioSample;
			
	    /* Only generate audio if the board button is being pressed */
	    //AudioSample = (Buttons_GetStatus() & BUTTONS_BUTTON1) ? CurrentWaveValue : 0;
	    AudioSample = abuf_get();
#ifdef WGS_EMULATE_48KHZ
	    //Calculate and send interpolated sample 
	    smp1b = (AudioSample>>1) + smp_old;
	    smp_old = AudioSample>>1; //store the second sample
	    WZab_Device_WriteSample24(&Microphone_Audio_Interface, (uint32_t)smp1b);
#endif		
	    WZab_Device_WriteSample24(&Microphone_Audio_Interface, (uint32_t)AudioSample);
	  }

	Endpoint_SelectEndpoint(PrevEndpoint);
      }
      //check if connection is working 
      if(timer_count==0) {
	//No packet received for long time. Probably someone has changed the channel
	//in the transmitter
	//Maximum search time: 80 channels * 4ms/channel = 320ms! (quite acceptable)
	set_ce(0);
	cli(); //Avoid collision with the SPI access in ISRs
	if((++rf_channel) == 83) rf_channel = 0; //Change channel, don't use channels above 83!
	set_channel(rf_channel);
	set_timer(PKT_TIMEOUT);
	sei();
	set_ce(1);
      }
      //Process MIDI every 1024 loops!
      if(((++sample_nr) % 1024) == 0)
	{
	  uint8_t tmp1, tmp2, i;
	  //Check switches
	  tmp1 = switches;
	  tmp2 = oldswitches ^ switches;
	  if(tmp2) {
	    // Something has changed!
	    oldswitches = tmp1;
	    for(i=0;i<WGS_NUM_OF_SWITCHES;i++) {
	      if (tmp2 & 1) {
		// Send the new value of the i-th switch
		if (tmp1 & 1) { //bit 0 in tmp1 was 1 - switch is ON
		  if(switch_on_msg[i][0]) {
		    MIDI_EventPacket_t MIDIEvent = (MIDI_EventPacket_t)
		      {
			.CableNumber = (switch_on_msg[i][0]>>4) & 0x0f,
			.Command     = switch_on_msg[i][0] & 0x0f,
	      
			.Data1       = switch_on_msg[i][1],
			.Data2       = switch_on_msg[i][2],
			.Data3       = switch_on_msg[i][3],
		      };
		    MIDI_Device_SendEventPacket(&Keyboard_MIDI_Interface, &MIDIEvent);
		  }
		} else { // bit0 in tmp1 was 0 - switch is OFF
		  if(switch_off_msg[i][0]) {
		    MIDI_EventPacket_t MIDIEvent = (MIDI_EventPacket_t)
		      {
			.CableNumber = (switch_off_msg[i][0]>>4) & 0x0f,
			.Command     = switch_off_msg[i][0] & 0x0f,
	      
			.Data1       = switch_off_msg[i][1],
			.Data2       = switch_off_msg[i][2],
			.Data3       = switch_off_msg[i][3],
		      };
		    MIDI_Device_SendEventPacket(&Keyboard_MIDI_Interface, &MIDIEvent);	          
		  }
		}
	      }
	      tmp2 >>= 1;
	      tmp1 >>= 1;
	    }
	  }
	  //Check analog controllers
	  for(i=0;i<WGS_NUM_OF_POTS;i++) { 
	    tmp1 = adc_val[i]>>1; 
	    // Send the state of controller after the change, or every 65536 samples
	    if (tmp1 != old_adc_val[i]) {
	      // I tried to send ADC values also when (sample_nr == 0) but it caused 
	      // problems with MIDI Learn function in different programs
	      old_adc_val[i]=tmp1;
	      MIDI_EventPacket_t MIDIEvent = (MIDI_EventPacket_t)
		{
		  .CableNumber = (pot_msg[i][0]>>4) & 0x0f,
		  .Command     = pot_msg[i][0] & 0x0f,
	      
		  .Data1       = pot_msg[i][1],
		  .Data2       = pot_msg[i][2],
		  .Data3       = pot_msg[3] ? (127 - tmp1) :  tmp1,
		};
	      MIDI_Device_SendEventPacket(&Keyboard_MIDI_Interface, &MIDIEvent);
	    }
	  }
	  MIDI_Device_Flush(&Keyboard_MIDI_Interface);
	  MIDI_Device_USBTask(&Keyboard_MIDI_Interface);
	}
      Audio_Device_USBTask(&Microphone_Audio_Interface);
      USB_USBTask();
    }
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
  /* Disable watchdog if enabled by bootloader/fuses */
  MCUSR &= ~(1 << WDRF);
  wdt_disable();

  /* Disable clock division */
  /* cli();
     CLKPR=0x80;
     CLKPR=0x00;
     sei() */;
  clock_prescale_set(clock_div_1);

  /* Hardware Initialization */
  //LEDs_Init();
  //Buttons_Init();
  //ADC_Init(ADC_FREE_RUNNING | ADC_PRESCALE_32);
  //ADC_SetupChannel(MIC_IN_ADC_CHANNEL);
  rfm70_hw_setup();
  SPI_MasterInit();
  rfm70_init();
  USB_Init();
  //Set INT3 as input and switch on the pullup
  DDRD  &= ~(1 << 3);
  PORTD |= (1 << 3);
  //Prepare packet receive LED
  DDR_PKTRCV |= (1<<PIN_NR_PKTRCV);
  PORT_PKTRCV &= ~(1<<PIN_NR_PKTRCV);
  //Prepare the overrun and packet lost detection LEDs
  DDR_OVR |= (1<<PIN_NR_OVR);
  PORT_OVR &= ~(1<<PIN_NR_OVR);
  DDR_PKTLST |= (1<<PIN_NR_PKTLST);
  PORT_PKTLST &= ~(1<<PIN_NR_PKTLST);
  //Prepare the message LED (or beeper)
  DDR_BEEPER |= (1<<PIN_NR_BEEPER);
  PORT_BEEPER &= ~(1<<PIN_NR_BEEPER);  
}

static PROGMEM uint8_t cmd_flush_rx[]={0xe2,0x00};
static PROGMEM uint8_t cmd_flush_tx[]={0xe1,0x00};

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
  //LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
  EIFR = (1<<INTF3);
  EIMSK |= (1<<INT3); //Turn on the receiver's interrupts
  set_ce(0);
  _delay_us(10);
  write_pcmd(cmd_flush_tx,sizeof(cmd_flush_tx));
  write_pcmd(cmd_flush_rx,sizeof(cmd_flush_rx));
  write_reg(7,0x70); //cancel all possible interrupts
  switch_to_rx_mode();
  return;

}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
  EIMSK &= ~(1<<INT3);
  set_ce(0);
  //LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
  bool ConfigSuccess = true;

  ConfigSuccess &= Audio_Device_ConfigureEndpoints(&Microphone_Audio_Interface);
  ConfigSuccess &= MIDI_Device_ConfigureEndpoints(&Keyboard_MIDI_Interface);
  //LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/* Function for handling of our private USB Control Requests */
void WGS_Device_ProcessControlRequest(void)
{
  if (!(Endpoint_IsSETUPReceived()))
    return;
  //Handle only vendor requests
  if ((USB_ControlRequest.bmRequestType & CONTROL_REQTYPE_TYPE)==REQTYPE_VENDOR)
    {
      if ((USB_ControlRequest.bmRequestType & CONTROL_REQTYPE_DIRECTION)==REQDIR_HOSTTODEVICE)
	{
	  uint8_t ctrl_num = USB_ControlRequest.bRequest & 0x0f;
	  switch (USB_ControlRequest.bRequest & 0xf0) {
	  case 0xb0:
	    //It is a message setting the function of potentiometer
	    //Check if the number of potentiometer is correct
	    if (ctrl_num >= WGS_NUM_OF_POTS)
	      return;
	    pot_msg[ctrl_num][0]= (USB_ControlRequest.wValue >> 8) & 0xff;
	    pot_msg[ctrl_num][1]= USB_ControlRequest.wValue & 0xff;
	    pot_msg[ctrl_num][2]= (USB_ControlRequest.wIndex >> 8) & 0xff;
	    pot_msg[ctrl_num][3]= USB_ControlRequest.wIndex & 0xff;
	    break;
	  case 0x80:
	    //It is a message setting the "ON" message for a switch
	    //Check if the number of switch is correct
	    if (ctrl_num >= WGS_NUM_OF_SWITCHES)
	      return;
	    switch_on_msg[ctrl_num][0]= (USB_ControlRequest.wValue >> 8) & 0xff;
	    switch_on_msg[ctrl_num][1]= USB_ControlRequest.wValue & 0xff;
	    switch_on_msg[ctrl_num][2]= (USB_ControlRequest.wIndex >> 8) & 0xff;
	    switch_on_msg[ctrl_num][3]= USB_ControlRequest.wIndex & 0xff;
	    break;
	  case 0xc0:
	    //It is a message setting the "OFF" message for a switch
	    //Check if the number of switch is correct
	    if (ctrl_num >= WGS_NUM_OF_SWITCHES)
	      return;
	    switch_off_msg[ctrl_num][0]= (USB_ControlRequest.wValue >> 8) & 0xff;
	    switch_off_msg[ctrl_num][1]= USB_ControlRequest.wValue & 0xff;
	    switch_off_msg[ctrl_num][2]= (USB_ControlRequest.wIndex >> 8) & 0xff;
	    switch_off_msg[ctrl_num][3]= USB_ControlRequest.wIndex & 0xff;
	    break;
	  case 0x10:
	    //This message will set other parameters.
	    //I hope to add possibility to receive data from another transmitter...
	    //but currently it does nothing
            return;
	  default:
	    //Illegal request - don't handle it
	    return;
	  }
	  //Request handled
	  Endpoint_ClearSETUP();
	  Endpoint_ClearStatusStage();
	}
    }
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
  Audio_Device_ProcessControlRequest(&Microphone_Audio_Interface);
  MIDI_Device_ProcessControlRequest(&Keyboard_MIDI_Interface);
  //Now we should process our control requests, used to program
  //functionality of our system  
  WGS_Device_ProcessControlRequest();
}

