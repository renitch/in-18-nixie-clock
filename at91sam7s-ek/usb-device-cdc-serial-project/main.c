#include <board.h>
#include <pio/pio.h>
#include <pio/pio_it.h>
#include <irq/irq.h>
#include <tc/tc.h>
#include <usart/usart.h>
#include <utility/trace.h>
#include <utility/led.h>
#include <usb/device/cdc-serial/CDCDSerialDriver.h>
#include <usb/device/cdc-serial/CDCDSerialDriverDescriptors.h>
#include <pmc/pmc.h>

#define PIN_USART1_RXD  {1 << 21, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}
#define PIN_USART1_TXD  {1 << 22, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}


#define PINS_USART1      PIN_USART1_TXD, PIN_USART1_RXD
#define BASE_USART1      AT91C_BASE_US1
#define ID_USART1        AT91C_ID_US1     

static const Pin pinsUsart1[] = {PINS_USART1};


#define PIN_DATA  {1 << 26, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_PULLUP}
#define PIN_CLK   {1 << 24, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_PULLUP}
#define PIN_LATCH {1 << 25, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_PULLUP}

static const Pin pinData = PIN_DATA;
static const Pin pinClk  = PIN_CLK;
static const Pin pinLatch = PIN_LATCH;

char hoursHI[] = {13,  5, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13};
char hoursLO[] = { 1,  4,  0,  2,  8, 12, 13,  5,  3,  9,  1,  1,  1,  1,  1,  1};

char minutesHI[] = { 8,  9,  1,  0, 12, 13,  8,  8,  8,  8,  8,  8,  8,  8,  8,  8};
char minutesLO[] = { 2, 13, 12,  9,  4,  5,  0,  1,  8,  3,  2,  2,  2,  2,  2,  2};

char secondsHI[] = { 5, 12,  8,  9,  0,  1,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5};
char secondsLO[] = { 2,  4,  5, 12,  0,  1,  9,  8, 13,  3,  2,  2,  2,  2,  2,  2};

int globalYear = 0;
int globalMonth = 0;
int globalDay = 0;
int globalHours = 0;
int globalMinutes = 0;
int globalSeconds = 0;

static int isDaylightSavingTime = 0;

void delay(unsigned long a) {
  for (int x=0;x<a;x++) asm ("nop");
}

//------------------------------------------------------------------------------
/// Handles interrupts coming from USART1
//------------------------------------------------------------------------------
void USART2_IrqHandler(void)
{
    unsigned int status = BASE_USART1->US_CSR;
    unsigned short serialState;

    printf("USART2_IrqHandler ENTERING\n\r");

    // Buffer has been read successfully
    if ((status & AT91C_US_ENDRX) != 0) {
        printf("US Rx\n\r");
    }

    // Buffer has been sent
    if ((status & AT91C_US_TXBUFE) != 0) {

      printf("USART2_IrqHandler Restart USB read\n\r");
        // Restart USB read
        //CDCDSerialDriver_Read(usbBuffer, DATABUFFERSIZE, (TransferCallback) UsbDataReceived, 0);
        //BASE_USART1->US_IDR = AT91C_US_TXBUFE;
    }

    // Errors
    //serialState = CDCDSerialDriver_GetSerialState();
    printf("USART2_IrqHandler serialState=%i\n\r", serialState);

    // Overrun
    if ((status & AT91C_US_OVER) != 0) {

        TRACE_WARNING( "USART1_IrqHandler: Overrun\n\r");
    }

    // Framing error
    if ((status & AT91C_US_FRAME) != 0) {

        TRACE_WARNING( "USART1_IrqHandler: Framing error\n\r");
    }
}

//------------------------------------------------------------------------------
//          Main
//------------------------------------------------------------------------------

void InitUSART1(void)
{
    PIO_Configure(pinsUsart1, PIO_LISTSIZE(pinsUsart1));
    AT91C_BASE_PMC->PMC_PCER = 1 << ID_USART1;
    BASE_USART1->US_IDR = 0xFFFFFFFF;
    USART_Configure(BASE_USART1,
                    USART_MODE_ASYNCHRONOUS,
                    9600,//115200,
                    BOARD_MCK);

    USART_SetTransmitterEnabled(BASE_USART1, 1);
    USART_SetReceiverEnabled(BASE_USART1, 1);
    //IRQ_ConfigureIT(ID_USART1, 0, USART2_IrqHandler);
    //IRQ_EnableIT(ID_USART1);
    
    BASE_USART1->US_IER = AT91C_US_ENDRX | AT91C_US_FRAME | AT91C_US_OVER;
}

unsigned char read_char_USART1_nonstop(void)
{
  if ((BASE_USART1->US_CSR & AT91C_US_RXRDY) == 1)
    return ((BASE_USART1->US_RHR) & 0x1FF);
  else
    return 0;
}

#define BUFFERSIZE 1024

static int UTC_OFFSET=2;

static char gpsBuffer[BUFFERSIZE];
static int currentBufferIndex = 0;
static int isGPSCommand = 0;

void extractTime(char * rmc_data);
void showTime();

void writeByte(unsigned char data) {
  int delayValue = 10;
  for (int i = 0; i < 8; i++) {
    int b = (data >> i) & 0x1;

    //delay(5000);
    delay(delayValue);
    PIO_Clear(&pinClk);
    delay(delayValue);
    if (b>0) {
      delay(delayValue);
      PIO_Set(&pinData);
      delay(delayValue);
    } else {
      delay(delayValue);
      PIO_Clear(&pinData);
      delay(delayValue);
    }
    delay(delayValue);
    PIO_Set(&pinClk);
    delay(delayValue);
  }
}

//Summer time functions----
int DayOfWeek(int day, int month, int year) {
  int a = (14 - month) / 12;
  int y = year - a;
  int m = month + 12 * a - 2;
  return (7000 + (day + y + y / 4 - y / 100 + y / 400 + (31 * m) / 12)) % 7;
}

int IsLeapYear (int year) {
  //return ((year % 400) == 0) || (((year % 4) == 0) && ((year % 100) ~= 0));
  if ((year & 3) == 0 && ((year % 25) != 0 || (year & 15) == 0)) {
    return 1;
  } else {
    return 0;
  }
}

int MonthDay(int Month, int Year)
{
  if(Month == 1) {
    return 31;
  } else if (Month == 2 && IsLeapYear(Year) == 1) {
    return 29;
  } else if (Month == 2 && IsLeapYear(Year) == 0) {
    return 28;
  } else if (Month == 3) {
    return 31;
  } else if (Month == 4) {
    return 30;
  } else if (Month == 5) {
    return 31;
  } else if (Month == 6) {
    return 30;
  } else if (Month == 7) {
    return 31;
  } else if (Month == 8) {
    return 31;
  } else if (Month == 9) {
    return 30;
  } else if (Month == 10) {
    return 31;
  } else if (Month == 11) {
    return 30;
  } else if (Month == 12) {
    return 31;
  } else return 0;
}

void correctSummertime(/*int Year, int Month, int Day, int Hour, int Minute, int Second*/)
{
  //int zone = UTC_OFFSET;
    
  //Ќаходим последнее воскресенье марта и последнее воскресенье окт€бр€
  int M = 31 - DayOfWeek(31,3,globalYear);
  int O = 31 - DayOfWeek(31,10,globalYear);
  
  //”чЄт летнего времни (с последнего воскресень€ марта до последнего воскресень€ окт€бр€)
  //incorrect old implementation
  /*
  if (((globalMonth > 3) && (globalMonth < 10)) || 
      ((globalMonth == 3) && (globalDay >= M) && globalHours >= 0) || 
        ((globalMonth == 10) && (globalDay <= O) && globalHours <= 1 ))
{
          zone = zone + 1;
        }
  if ((globalHours + zone) < 24)
{
    globalHours = globalHours + zone;
  } else
{
    globalHours = globalHours + zone - 24;
    if ((globalDay + 1) <= MonthDay(globalMonth, globalYear))
{
      globalDay = globalDay + 1;
    } else
{
      globalDay = 1;
      if ((globalMonth + 1) <= 12)
{
        globalMonth = globalMonth + 1;
      }
else
{
        globalMonth = 1;
        globalYear = globalYear + 1;
      }
    }
  }*/
  int summertimeOffset = 0;
  
  if (((globalMonth > 3) && (globalMonth < 10)) || 
      ((globalMonth == 3) && (globalDay >= M)) || 
      ((globalMonth == 10) && (globalDay <= O))) {

      isDaylightSavingTime = 1;

      if ((globalMonth == 3) && (globalDay == M) && globalHours < 3 ) {
        isDaylightSavingTime = 0;
      }

      if ((globalMonth == 10) && (globalDay >= O) && globalHours >=3 ) {
        isDaylightSavingTime = 0;
      }
      
      if (isDaylightSavingTime > 0) {
        summertimeOffset = 1;
      }
  }

  if ((globalHours + summertimeOffset) < 24) {
    globalHours = globalHours + summertimeOffset;
  } else {
    globalHours = globalHours + summertimeOffset - 24;
    if ((globalDay + 1) <= MonthDay(globalMonth, globalYear)) {
      globalDay = globalDay + 1;
    } else {
      globalDay = 1;
      if ((globalMonth + 1) <= 12) {
        globalMonth = globalMonth + 1;
      } else {
        globalMonth = 1;
        globalYear = globalYear + 1;
      }
    }
  }
}

void correctTimezone() {
  int zone = UTC_OFFSET;
  
  if ((globalHours + zone) < 24) {
    globalHours = globalHours + zone;
  } else {
    globalHours = globalHours + zone - 24;
    if ((globalDay + 1) <= MonthDay(globalMonth, globalYear)) {
      globalDay = globalDay + 1;
    } else {
      globalDay = 1;
      if ((globalMonth + 1) <= 12) {
        globalMonth = globalMonth + 1;
      } else {
        globalMonth = 1;
        globalYear = globalYear + 1;
      }
    }
  }
}
//-------------------------

void processChar(char c) {
  if (isGPSCommand == 0) {
    if (c == '$') {
      isGPSCommand = 1;
      currentBufferIndex = 0;
      memset(gpsBuffer, 0, BUFFERSIZE);
      //printf("command start\n\r");
    }
  } else if (c == 10 || c == 13 || c == '$') {
      isGPSCommand = 0;
      //printf("time to extract\n\r");
      extractTime(gpsBuffer);
  }
  
  if (isGPSCommand == 1) {
    gpsBuffer[currentBufferIndex++] = c;
  }
}

void extractTime(char * rmc_data) {
  if(strncmp(rmc_data, "$GPZDA", 6) == 0){
    int i = 0;
    int got_hour;
    int got_min;
    int got_sec;
    int got_day;
    int got_month;
    int got_year;

    while(rmc_data[i++] != ','){ if (i > 768) return; };
        got_hour = (rmc_data[i]-'0')*10 + (rmc_data[i+1]-'0');
	got_min = ((rmc_data[i+2]-'0')*10)+(rmc_data[i+3]-'0');
	got_sec = ((rmc_data[i+4]-'0')*10)+(rmc_data[i+5]-'0');
	i+=6;
	while(rmc_data[i++] != ','){ if (i > 768) return; };
	got_day = ((rmc_data[i]-'0')*10)+(rmc_data[i+1]-'0');
	while(rmc_data[i++] != ','){ if (i > 768) return; };
	got_month = ((rmc_data[i]-'0')*10)+(rmc_data[i+1]-'0');
	while(rmc_data[i++] != ','){ if (i > 768) return; };
	got_year = ((rmc_data[i]-'0')*1000)+(rmc_data[i+1]-'0')*100 + (rmc_data[i+2]-'0')*10+(rmc_data[i+3]-'0');
        
        printf("time: %i:%i:%i %i.%i.%i\n\r", got_hour, got_min, got_sec, got_day, got_month, got_year);

        globalYear = got_year;
        globalMonth = got_month;
        globalDay = got_day;
        globalHours = got_hour;
        globalMinutes = got_min;
        globalSeconds = got_sec;

	/*got_hour += UTC_OFFSET;
	if(got_hour >= 24){
		got_hour = got_hour - 24;
		got_day++;
	}*/
        correctTimezone();
        //Correct hours due to summertime
        correctSummertime();
        //-------------------------------
        printf("corrected time: %i:%i:%i %i.%i.%i\n\r", globalHours, globalMinutes, globalSeconds, globalDay, globalMonth, globalYear);
        //SHOW TIME
        showTime();
        //END SHOW TIME
    }
}

void showTime() {
  
  int hourHi = globalHours / 10;
  int hourLo = globalHours % 10;
  int minHi = globalMinutes / 10;
  int minLo = globalMinutes % 10;
  int secHi = globalSeconds / 10;
  int secLo = globalSeconds % 10;
  
  PIO_Clear(&pinLatch);

  writeByte(hoursLO[hourLo] + hoursHI[hourHi]*16);
  writeByte(minutesLO[minLo]*16 + minutesHI[minHi]);
  writeByte(secondsLO[secLo]*16 + secondsHI[secHi]);
  
  PIO_Set(&pinLatch);
}

//------------------------------------------------------------------------------
/// Initializes drivers and start the USB <-> Serial bridge.
//------------------------------------------------------------------------------
int main()
{
    TRACE_CONFIGURE(DBGU_STANDARD, 115200, BOARD_MCK);
    printf("-- USB Device CDC Serial Project %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

    // If they are present, configure Vbus & Wake-up pins
    PIO_InitializeInterrupts(0);

    LED_Configure(0);
  
    PIO_Configure(&pinData, 1);
    PIO_Configure(&pinClk, 1);
    PIO_Configure(&pinLatch, 1);

    InitUSART1();

    isGPSCommand = 0;
    
    int iii = 0;
    
    while (1) {
        unsigned char token = read_char_USART1_nonstop();
        putchar(token);
        if (token != 0) {
            processChar(token);
        }
    }
/*
    PIO_Configure(&pinData, 1);
    PIO_Configure(&pinClk, 1);
    PIO_Configure(&pinLatch, 1);
  for (int i =0; i < 16;i++) {
    
    delay(5000);
    PIO_Clear(&pinLatch);
    delay(5000);

    writeByte(hoursLO[i] + hoursHI[i]*16);
    writeByte(minutesLO[i]*16 + minutesHI[i]);
    writeByte(secondsLO[i]*16 + secondsHI[i]);

    delay(5000000);
    
        PIO_Set(&pinLatch);
  }
    */
}

