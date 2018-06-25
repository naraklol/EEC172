//EEC 172
//Neil Arakkal
//Daivik Dinesh

// Standard includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// simplelink includes
#include "device.h"

// driverlib includes
#include "hw_memmap.h"
#include "hw_types.h"
#include "hw_ints.h"
#include "interrupt.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "pinmux.h"
#include "hw_common_reg.h"
#include "spi.h"
#include "gpio.h"
#include "gpio_if.h"
#include "interrupt.h"
#include "Adafruit_GFX.h"
#include "Adafruit_OLED.h"
#include "Adafruit_SSD1351.h"
#include "glcdfont.h"
#include "test.h"
#include "timer.h"
#include "uart.h"

//Free_rtos/ti-rtos includes
#include "osi.h"

// common interface includes
#ifndef NOTERM
#include "uart_if.h"
#endif
#include "timer_if.h"
#include "network_if.h"
#include "udma_if.h"
#include "common.h"

// HTTP Client lib
#include <http/client/httpcli.h>
#include <http/client/common.h>

#define APPLICATION_VERSION     "1.1.1"
#define APP_NAME                "Get Weather"

#define SLEEP_TIME              8000000
#define SUCCESS                 0
#define OSI_STACK_SIZE          3000

#define PREFIX_BUFFER "/data/2.5/weather?q="
#define POST_BUFFER "&mode=xml&units=metric&appid=50b7eab9f2d6c71f2b36bfd33017876d"

#define HOST_NAME       "api.openweathermap.org"
#define HOST_PORT       (80)

#define SPI_IF_BIT_RATE  1000000
#define TR_BUFF_SIZE     100

#define BUTTON_ONE      49000 //
#define BUTTON_TWO      50000 //
#define BUTTON_THREE    51000 //
#define BUTTON_FOUR     52000 //
#define BUTTON_FIVE     53000 //
#define BUTTON_SIX      54000 //
#define BUTTON_SEVEN    55000 //
#define BUTTON_EIGHT    56000 //
#define BUTTON_NINE     57000 //
#define BUTTON_ZERO     48000 //
#define BUTTON_CHUP     35000 //pound
#define BUTTON_CHDOWN   42000 //star
#define BUTTON_DELAY    80000000

#define X_CHAR 6
#define Y_CHAR 8

// Application specific status/error codes
typedef enum
{
    // Choosing -0x7D0 to avoid overlap w/ host-driver's error codes
    SERVER_GET_WEATHER_FAILED = -0x7D0,
    WRONG_CITY_NAME = SERVER_GET_WEATHER_FAILED - 1,
    NO_WEATHER_DATA = WRONG_CITY_NAME - 1,
    DNS_LOOPUP_FAILED = WRONG_CITY_NAME - 1,

    STATUS_CODE_MAX = -0xBB8
} e_AppStatusCodes;

unsigned long g_ulTimerInts;   //  Variable used in Timer Interrupt Handler
SlSecParams_t SecurityParams = { 0 };  // AP Security Parameters

char acSendBuff[512];   // Buffer to send data
char acRecvbuff[1460];  // Buffer to receive data

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

static long GetWeather(HTTPCli_Handle cli, int iSockID, char *pcCityName);
void GetWeatherTask(void *pvParameters);
static void BoardInit();
static void DisplayBanner(char * AppName);
static int HandleXMLData(char *acRecvbuff);
static int FlushHTTPResponse(HTTPCli_Handle cli);

//DTMF Global Variables

typedef struct PinSetting
{
    unsigned long port;
    unsigned int pin;
} PinSetting;
static PinSetting adc_cs = { .port = GPIOA3_BASE, .pin = 0x80 };

int timerflag = 0;
int ADC_value = 0;
int i = 0;
//int textMode = 0;
long int button_pressed = 0;
int N = 410;                       // block size
int samples[410];       // buffer to store N samples
long int power_all[8];       // array to store calculated power of 8 frequencies
long int coeff[8] = { 31548, 31281, 30951, 30556, 29143, 28360, 27408, 26258 }; // array to store the calculated coefficients
int f_tone[8] = { 697, 770, 852, 941, 1209, 1336, 1477, 1633 }; // frequencies of rows & columns
int new_dig; // flag set when inter-digit interval (pause) is detected
char decoded_letter = '$';
int sampleIndex = 0;
int sampleFull = 0;
int sampleReady = 0;

int lastPressed = 0;
int timeAfterPress = 0;

unsigned int txComplete = 0;

char txBuffer[128];
int buffIndex = 0;
void processADC(void);

//DTMF Functions

int getADC(void)
{
    int value;

    unsigned char c1 = 0, c2 = 0;
    GPIOPinWrite(adc_cs.port, adc_cs.pin, 0x00);

    MAP_SPICSEnable(GSPI_BASE);

    SPITransfer(GSPI_BASE, 0, &c1, 1, (SPI_CS_ENABLE | SPI_CS_DISABLE));
    SPITransfer(GSPI_BASE, 0, &c2, 1, (SPI_CS_ENABLE | SPI_CS_DISABLE));
    MAP_SPICSDisable(GSPI_BASE);
    GPIOPinWrite(adc_cs.port, adc_cs.pin, 0xff);

    c1 = c1 << 3;
    c2 = c2 >> 3;
    unsigned int temp1 = c1 << 2;
    unsigned int temp2 = c2;
    value = temp1 | temp2;
    value -= 388; // removing dc bias

    return value;
}

void TimerBaseIntHandler(void)
{
    sampleFull = 0;
    Timer_IF_InterruptClear(TIMERA0_BASE);

    sampleReady = 1;
    sampleIndex++;

    if (sampleIndex == N)
    {
        sampleFull = 1;
        sampleReady = 0;
        sampleIndex = 0;
        Timer_IF_DeInit(TIMERA0_BASE, TIMER_A);
    }
}

void T9Int(void)
{
    Timer_IF_InterruptClear(TIMERA1_BASE);
    lastPressed = 0;
}

long int goertzel(int sample[], long int coeff, int N)
{
    int Q, Q_prev, Q_prev2;
    long prod1, prod2, prod3, power;

    Q_prev = 0;
    Q_prev2 = 0;
    power = 0;

    for (i = 0; i < N; i++)
    {
        Q = (sample[i]) + ((coeff * Q_prev) >> 14) - (Q_prev2);
        Q_prev2 = Q_prev;
        Q_prev = Q;
    }

    prod1 = ((long) Q_prev * Q_prev);
    prod2 = ((long) Q_prev2 * Q_prev2);
    prod3 = ((long) Q_prev * coeff) >> 14;
    prod3 = (prod3 * Q_prev2);

    power = ((prod1 + prod2 - prod3)) >> 8;

    return power;
}

char analyzeGoertzel(void)
{
    int row, col, max_power;

    max_power = 0;

    char row_col[4][4] = { { '1', '2', '3', 'A' }, { '4', '5', '6', 'B' }, {
            '7', '8', '9', 'C' },
                           { '*', '0', '#', 'D' } };

    for (i = 0; i < 4; i++)
    {
        if (power_all[i] > max_power)
        {
            max_power = power_all[i];
            row = i;
        }
    }

    max_power = 0;

    for (i = 4; i < 8; i++)
    {
        if (power_all[i] > max_power)
        {
            max_power = power_all[i];
            col = i;
        }
    }

    int noiseCheckRow = 0;
    int noiseCheckCol = 0;

    for (i = 0; i < 4; i++)
        if (power_all[i] > 100000)
            noiseCheckRow++;

    for (i = 4; i < 8; i++)
        if (power_all[i] > 100000)
            noiseCheckCol++;

    if (power_all[col] < 1000 && power_all[row] < 1000)
        new_dig = 1;

    if ((power_all[col] > 1000 && power_all[row] > 1000) && (new_dig == 1)
            && (noiseCheckRow == 1) && (noiseCheckCol == 1))
    {
        new_dig = 0;
        return row_col[row][col - 4];
    }

    return '$';

}

void processADC(void)
{
    int j;

    for (j = 0; j < 8; j++)
        power_all[j] = goertzel(samples, coeff[j], N);

    decoded_letter = analyzeGoertzel();
}

void TimerPeriodicIntHandler(void)
{
    unsigned long ulInts;

    //
    // Clear all pending interrupts from the timer we are
    // currently using.
    //
    ulInts = MAP_TimerIntStatus(TIMERA0_BASE, true);
    MAP_TimerIntClear(TIMERA0_BASE, ulInts);

    //
    // Increment our interrupt counter.
    //
    g_ulTimerInts++;
    if (!(g_ulTimerInts & 0x1))
    {
        //
        // Off Led
        //
        GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    }
    else
    {
        //
        // On Led
        //
        GPIO_IF_LedOn(MCU_RED_LED_GPIO);
    }
}

char* textMode()
{
    int x = 0;
    int y = Y_CHAR;

    Outstr("Enter city name: ");

    while (1)
        {

            while (!sampleFull && !sampleReady)
             ;

             if (sampleReady)
             {
             sampleReady = 0;
             samples[sampleIndex] = getADC();
             }

             if (sampleFull)
             {  // clear flag
             sampleFull = 0;
             processADC();

             if (decoded_letter != '$')
             {
             timeAfterPress = MAP_TimerValueGet(TIMERA1_BASE, TIMER_A);
             timeAfterPress = -timeAfterPress;

             int letter_code = decoded_letter * 1000;

             switch (letter_code)
             {

             case (BUTTON_ONE):
             printf("1");
             break;

             case (BUTTON_TWO):

             printf("2");

             if (lastPressed == BUTTON_TWO + 1)
             {
             printf("C");
             x -= X_CHAR;
             txBuffer[--buffIndex] = 'C';
             buffIndex++;
             drawChar(x, y, 'B', BLACK, BLACK, 1);
             drawChar(x, y, 'C', WHITE, BLACK, 1);
             lastPressed = BUTTON_TWO + 2;
             x += X_CHAR;
             Timer_IF_ReLoad(TIMERA1_BASE, TIMER_A, BUTTON_DELAY);
             }
             if (lastPressed == BUTTON_TWO)
             {
             printf("B");
             x -= X_CHAR;
             txBuffer[--buffIndex] = 'B';
             buffIndex++;
             drawChar(x, y, 'A', BLACK, BLACK, 1);
             drawChar(x, y, 'B', WHITE, BLACK, 1);
             lastPressed = BUTTON_TWO + 1;
             x += X_CHAR;
             Timer_IF_ReLoad(TIMERA1_BASE, TIMER_A, BUTTON_DELAY);
             }

             if ((lastPressed != BUTTON_TWO + 1
             && lastPressed != BUTTON_TWO + 2))
             {
             printf("A");
             drawChar(x, y, 'A', WHITE, BLACK, 1);
             txBuffer[buffIndex++] = 'A';
             lastPressed = BUTTON_TWO;
             x += X_CHAR;
             Timer_IF_ReLoad(TIMERA1_BASE, TIMER_A, BUTTON_DELAY);
             }
             break;

             case (BUTTON_THREE):
             printf("3");

             if (lastPressed == BUTTON_THREE + 1)
             {
             printf("F");
             x -= X_CHAR;
             txBuffer[--buffIndex] = 'F';
             buffIndex++;
             drawChar(x, y, 'E', BLACK, BLACK, 1);
             drawChar(x, y, 'F', WHITE, BLACK, 1);
             lastPressed = BUTTON_THREE + 2;
             x += X_CHAR;
             Timer_IF_ReLoad(TIMERA1_BASE, TIMER_A, BUTTON_DELAY);
             }
             if (lastPressed == BUTTON_THREE)
             {
             printf("E");
             x -= X_CHAR;
             txBuffer[--buffIndex] = 'E';
             buffIndex++;
             drawChar(x, y, 'D', BLACK, BLACK, 1);
             drawChar(x, y, 'E', WHITE, BLACK, 1);
             lastPressed = BUTTON_THREE + 1;
             x += X_CHAR;
             Timer_IF_ReLoad(TIMERA1_BASE, TIMER_A, BUTTON_DELAY);
             }

             if ((lastPressed != BUTTON_THREE + 1
             && lastPressed != BUTTON_THREE + 2))
             {
             printf("D");
             drawChar(x, y, 'D', WHITE, BLACK, 1);
             txBuffer[buffIndex++] = 'D';
             lastPressed = BUTTON_THREE;
             x += X_CHAR;
             Timer_IF_ReLoad(TIMERA1_BASE, TIMER_A, BUTTON_DELAY);
             }
             break;

             case (BUTTON_FOUR):
             printf("4");

             if (lastPressed == BUTTON_FOUR + 1)
             {
             printf("I");
             x -= X_CHAR;
             txBuffer[--buffIndex] = 'I';
             buffIndex++;
             drawChar(x, y, 'H', BLACK, BLACK, 1);
             drawChar(x, y, 'I', WHITE, BLACK, 1);
             lastPressed = BUTTON_FOUR + 2;
             x += X_CHAR;
             Timer_IF_ReLoad(TIMERA1_BASE, TIMER_A, BUTTON_DELAY);
             }
             if (lastPressed == BUTTON_FOUR)
             {
             printf("H");
             x -= X_CHAR;
             txBuffer[--buffIndex] = 'H';
             buffIndex++;
             drawChar(x, y, 'G', BLACK, BLACK, 1);
             drawChar(x, y, 'H', WHITE, BLACK, 1);
             lastPressed = BUTTON_FOUR + 1;
             x += X_CHAR;
             Timer_IF_ReLoad(TIMERA1_BASE, TIMER_A, BUTTON_DELAY);
             }

             if ((lastPressed != BUTTON_FOUR + 1
             && lastPressed != BUTTON_FOUR + 2))
             {
             printf("G");
             drawChar(x, y, 'G', WHITE, BLACK, 1);
             txBuffer[buffIndex++] = 'G';
             lastPressed = BUTTON_FOUR;
             x += X_CHAR;
             Timer_IF_ReLoad(TIMERA1_BASE, TIMER_A, BUTTON_DELAY);
             }
             break;

             case (BUTTON_FIVE):
             printf("5");

             if (lastPressed == BUTTON_FIVE + 1)
             {
             printf("L");
             x -= X_CHAR;
             txBuffer[--buffIndex] = 'L';
             buffIndex++;
             drawChar(x, y, 'K', BLACK, BLACK, 1);
             drawChar(x, y, 'L', WHITE, BLACK, 1);
             lastPressed = BUTTON_FIVE + 2;
             x += X_CHAR;
             Timer_IF_ReLoad(TIMERA1_BASE, TIMER_A, BUTTON_DELAY);
             }
             if (lastPressed == BUTTON_FIVE)
             {
             printf("K");
             x -= X_CHAR;
             txBuffer[--buffIndex] = 'K';
             buffIndex++;
             drawChar(x, y, 'J', BLACK, BLACK, 1);
             drawChar(x, y, 'K', WHITE, BLACK, 1);
             lastPressed = BUTTON_FIVE + 1;
             x += X_CHAR;
             Timer_IF_ReLoad(TIMERA1_BASE, TIMER_A, BUTTON_DELAY);
             }

             if ((lastPressed != BUTTON_FIVE + 1
             && lastPressed != BUTTON_FIVE + 2))
             {
             printf("J");
             drawChar(x, y, 'J', WHITE, BLACK, 1);
             txBuffer[buffIndex++] = 'J';
             lastPressed = BUTTON_FIVE;
             x += X_CHAR;
             Timer_IF_ReLoad(TIMERA1_BASE, TIMER_A, BUTTON_DELAY);
             }
             break;

             case (BUTTON_SIX):
             printf("6");

             if (lastPressed == BUTTON_SIX + 1)
             {
             printf("O");
             x -= X_CHAR;
             txBuffer[--buffIndex] = 'O';
             buffIndex++;
             drawChar(x, y, 'N', BLACK, BLACK, 1);
             drawChar(x, y, 'O', WHITE, BLACK, 1);
             lastPressed = BUTTON_SIX + 2;
             x += X_CHAR;
             Timer_IF_ReLoad(TIMERA1_BASE, TIMER_A, BUTTON_DELAY);
             }
             if (lastPressed == BUTTON_SIX)
             {
             printf("N");
             x -= X_CHAR;
             txBuffer[--buffIndex] = 'N';
             buffIndex++;
             drawChar(x, y, 'M', BLACK, BLACK, 1);
             drawChar(x, y, 'N', WHITE, BLACK, 1);
             lastPressed = BUTTON_SIX + 1;
             x += X_CHAR;
             Timer_IF_ReLoad(TIMERA1_BASE, TIMER_A, BUTTON_DELAY);
             }

             if ((lastPressed != BUTTON_SIX + 1
             && lastPressed != BUTTON_SIX + 2))
             {
             printf("M");
             drawChar(x, y, 'M', WHITE, BLACK, 1);
             txBuffer[buffIndex++] = 'M';
             lastPressed = BUTTON_SIX;
             x += X_CHAR;
             Timer_IF_ReLoad(TIMERA1_BASE, TIMER_A, BUTTON_DELAY);
             }

             break;

             case (BUTTON_SEVEN):
             printf("7");

             if (lastPressed == BUTTON_SEVEN + 2)
             {
             printf("S");
             x -= X_CHAR;
             txBuffer[--buffIndex] = 'S';
             buffIndex++;
             drawChar(x, y, 'R', BLACK, BLACK, 1);
             drawChar(x, y, 'S', WHITE, BLACK, 1);
             lastPressed = BUTTON_SEVEN + 3;
             x += X_CHAR;
             Timer_IF_ReLoad(TIMERA1_BASE, TIMER_A, BUTTON_DELAY);
             }
             if (lastPressed == BUTTON_SEVEN + 1)
             {
             printf("R");
             x -= X_CHAR;
             txBuffer[--buffIndex] = 'R';
             buffIndex++;
             drawChar(x, y, 'Q', BLACK, BLACK, 1);
             drawChar(x, y, 'R', WHITE, BLACK, 1);
             lastPressed = BUTTON_SEVEN + 2;
             x += X_CHAR;
             Timer_IF_ReLoad(TIMERA1_BASE, TIMER_A, BUTTON_DELAY);
             }
             if (lastPressed == BUTTON_SEVEN)
             {
             printf("Q");
             x -= X_CHAR;
             txBuffer[--buffIndex] = 'Q';
             buffIndex++;
             drawChar(x, y, 'P', BLACK, BLACK, 1);
             drawChar(x, y, 'Q', WHITE, BLACK, 1);
             lastPressed = BUTTON_SEVEN + 1;
             x += X_CHAR;
             Timer_IF_ReLoad(TIMERA1_BASE, TIMER_A, BUTTON_DELAY);
             }

             if ((lastPressed != BUTTON_SEVEN + 1
             && lastPressed != BUTTON_SEVEN + 2
             && lastPressed != BUTTON_SEVEN + 3))
             {
             printf("P");
             drawChar(x, y, 'P', WHITE, BLACK, 1);
             txBuffer[buffIndex++] = 'P';
             lastPressed = BUTTON_SEVEN;
             x += X_CHAR;
             Timer_IF_ReLoad(TIMERA1_BASE, TIMER_A, BUTTON_DELAY);
             }
             break;

             case (BUTTON_EIGHT):
             printf("8");

             if (lastPressed == BUTTON_EIGHT + 1)
             {
             printf("V");
             x -= X_CHAR;
             txBuffer[--buffIndex] = 'V';
             buffIndex++;
             drawChar(x, y, 'U', BLACK, BLACK, 1);
             drawChar(x, y, 'V', WHITE, BLACK, 1);
             lastPressed = BUTTON_EIGHT + 2;
             x += X_CHAR;
             Timer_IF_ReLoad(TIMERA1_BASE, TIMER_A, BUTTON_DELAY);
             }
             if (lastPressed == BUTTON_EIGHT)
             {
             printf("U");
             x -= X_CHAR;
             txBuffer[--buffIndex] = 'U';
             buffIndex++;
             drawChar(x, y, 'T', BLACK, BLACK, 1);
             drawChar(x, y, 'U', WHITE, BLACK, 1);
             lastPressed = BUTTON_EIGHT + 1;
             x += X_CHAR;
             Timer_IF_ReLoad(TIMERA1_BASE, TIMER_A, BUTTON_DELAY);
             }
             if ((lastPressed != BUTTON_EIGHT + 1
             && lastPressed != BUTTON_EIGHT + 2))
             {
             printf("T");
             drawChar(x, y, 'T', WHITE, BLACK, 1);
             txBuffer[buffIndex++] = 'T';
             lastPressed = BUTTON_EIGHT;
             x += X_CHAR;
             Timer_IF_ReLoad(TIMERA1_BASE, TIMER_A, BUTTON_DELAY);
             }
             break;

             case (BUTTON_NINE):
             printf("9");

             if (lastPressed == BUTTON_NINE + 2)
             {
             printf("Z");
             x -= X_CHAR;
             txBuffer[--buffIndex] = 'Z';
             buffIndex++;
             drawChar(x, y, 'Y', BLACK, BLACK, 1);
             drawChar(x, y, 'Z', WHITE, BLACK, 1);
             lastPressed = BUTTON_NINE + 3;
             x += X_CHAR;
             Timer_IF_ReLoad(TIMERA1_BASE, TIMER_A, BUTTON_DELAY);
             }
             if (lastPressed == BUTTON_NINE + 1)
             {
             printf("Y");
             x -= X_CHAR;
             txBuffer[--buffIndex] = 'Y';
             buffIndex++;
             drawChar(x, y, 'X', BLACK, BLACK, 1);
             drawChar(x, y, 'Y', WHITE, BLACK, 1);
             lastPressed = BUTTON_NINE + 2;
             x += X_CHAR;
             Timer_IF_ReLoad(TIMERA1_BASE, TIMER_A, BUTTON_DELAY);
             }
             if (lastPressed == BUTTON_NINE)
             {
             printf("X");
             x -= X_CHAR;
             txBuffer[--buffIndex] = 'X';
             buffIndex++;
             drawChar(x, y, 'W', BLACK, BLACK, 1);
             drawChar(x, y, 'X', WHITE, BLACK, 1);
             lastPressed = BUTTON_NINE + 1;
             x += X_CHAR;
             Timer_IF_ReLoad(TIMERA1_BASE, TIMER_A, BUTTON_DELAY);
             }
             if ((lastPressed != BUTTON_NINE + 1
             && lastPressed != BUTTON_NINE + 2
             && lastPressed != BUTTON_NINE + 3))
             {
             printf("W");
             drawChar(x, y, 'W', WHITE, BLACK, 1);
             txBuffer[buffIndex++] = 'W';
             lastPressed = BUTTON_NINE;
             x += X_CHAR;
             Timer_IF_ReLoad(TIMERA1_BASE, TIMER_A, BUTTON_DELAY);
             }
             break;

             case (BUTTON_ZERO): // SPACE

             printf("0");
             txBuffer[buffIndex++] = ' ';
             x += X_CHAR;
             lastPressed = 0;
             Timer_IF_ReLoad(TIMERA1_BASE, TIMER_A, BUTTON_DELAY);
             break;

             case (BUTTON_CHUP): //SEND MESSAGE

             printf("#");
             //lastPressed = 675;

             txBuffer[buffIndex] = '\0';
             Report("\r\n\r\n\r\n\r\nNEW TRANSMISSION\r\n\r\n\r\n\r\n");
             char* sendBuffer = &txBuffer[0];


             printf("\r\n");
             buffIndex = 0;
             x = 0;
             y = 0;
             fillRect(0, 0, 128, 58, BLACK);
             Timer_IF_ReLoad(TIMERA1_BASE, TIMER_A, BUTTON_DELAY);
             return sendBuffer;
             break;

             case (BUTTON_CHDOWN): //BACKSPACE
             printf("*");

             if (buffIndex != 0)
             buffIndex--;

             lastPressed = 0;

             if (x > 0)
             {
             x -= X_CHAR;
             fillRect(x, y, X_CHAR, Y_CHAR, BLACK);
             }
             Timer_IF_ReLoad(TIMERA1_BASE, TIMER_A, BUTTON_DELAY);
             break;

             default:
             printf("\r\nNot supposed to go here\r\n");
             break;
             }
             }

             Timer_IF_Init(PRCM_TIMERA0, TIMERA0_BASE, TIMER_CFG_PERIODIC,
             TIMER_A,
             0);
             Timer_IF_IntSetup(TIMERA0_BASE, TIMER_A, TimerBaseIntHandler);
             Timer_IF_Start(TIMERA0_BASE, TIMER_A, 5000);
             }
        }

    return "notworking";


}

void LedTimerConfigNStart()
{
    //
    // Configure Timer for blinking the LED for IP acquisition
    //
    Timer_IF_Init(PRCM_TIMERA0, TIMERA0_BASE, TIMER_CFG_PERIODIC, TIMER_A, 0);
    Timer_IF_IntSetup(TIMERA0_BASE, TIMER_A, TimerPeriodicIntHandler);
    Timer_IF_Start(TIMERA0_BASE, TIMER_A, 100);  // Time value is in mSec
}

void LedTimerDeinitStop()
{
    //
    // Disable the LED blinking Timer as Device is connected to AP
    //
    Timer_IF_Stop(TIMERA0_BASE, TIMER_A);
    Timer_IF_DeInit(TIMERA0_BASE, TIMER_A);
}

char city[100];
char temp[100];
char condition[100];

static int HandleXMLData(char *acRecvbuff)
{
    char *pcIndxPtr;
    char *pcEndPtr;

    //
    // Get city name
    //
    pcIndxPtr = strstr(acRecvbuff, "name=");
    if (pcIndxPtr == NULL)
    {
        ASSERT_ON_ERROR(WRONG_CITY_NAME);
    }

    DBG_PRINT("\n\r****************************** \n\r\n\r");
    DBG_PRINT("City: ");
    if ( NULL != pcIndxPtr)
    {
        pcIndxPtr = pcIndxPtr + strlen("name=") + 1;
        pcEndPtr = strstr(pcIndxPtr, "\">");
        if ( NULL != pcEndPtr)
        {
            *pcEndPtr = 0;
        }
        DBG_PRINT("%s\n\r", pcIndxPtr);
        strcpy(city, pcIndxPtr);
    }
    else
    {
        DBG_PRINT("N/A\n\r");
        return NO_WEATHER_DATA;
    }

    //
    // Get temperature value
    //
    pcIndxPtr = strstr(pcEndPtr + 1, "temperature value");
    DBG_PRINT("Temperature: ");
    if ( NULL != pcIndxPtr)
    {
        pcIndxPtr = pcIndxPtr + strlen("temperature value") + 2;
        pcEndPtr = strstr(pcIndxPtr, "\" ");
        if ( NULL != pcEndPtr)
        {
            *pcEndPtr = 0;
        }
        DBG_PRINT("%s\n\r", pcIndxPtr);
        strcpy(temp, pcIndxPtr);
    }
    else
    {
        DBG_PRINT("N/A\n\r");
        return NO_WEATHER_DATA;
    }

    //
    // Get weather condition
    //
    pcIndxPtr = strstr(pcEndPtr + 1, "weather number");
    DBG_PRINT("Weather Condition: ");
    if ( NULL != pcIndxPtr)
    {
        pcIndxPtr = pcIndxPtr + strlen("weather number") + 14;
        pcEndPtr = strstr(pcIndxPtr, "\" ");
        if ( NULL != pcEndPtr)
        {
            *pcEndPtr = 0;
        }
        DBG_PRINT("%s\n\r", pcIndxPtr);
        strcpy(condition, pcIndxPtr);
    }
    else
    {
        DBG_PRINT("N/A\n\r");
        return NO_WEATHER_DATA;
    }

    return SUCCESS;
}

static int FlushHTTPResponse(HTTPCli_Handle cli)
{
    const char *ids[2] = {
    HTTPCli_FIELD_NAME_CONNECTION, /* App will get connection header value. all others will skip by lib */
                           NULL };
    char buf[128];
    int id;
    int len = 1;
    bool moreFlag = 0;
    char ** prevRespFilelds = NULL;

    prevRespFilelds = HTTPCli_setResponseFields(cli, ids);

    //
    // Read response headers
    //
    while ((id = HTTPCli_getResponseField(cli, buf, sizeof(buf), &moreFlag))
            != HTTPCli_FIELD_ID_END)
    {

        if (id == 0)
        {
            if (!strncmp(buf, "close", sizeof("close")))
            {
                UART_PRINT("Connection terminated by server\n\r");
            }
        }

    }

    HTTPCli_setResponseFields(cli, (const char **) prevRespFilelds);

    while (1)
    {
        len = HTTPCli_readResponseBody(cli, buf, sizeof(buf) - 1, &moreFlag);
        ASSERT_ON_ERROR(len);

        if ((len - 2) >= 0 && buf[len - 2] == '\r' && buf[len - 1] == '\n')
        {

        }

        if (!moreFlag)
        {
            break;
        }
    }
    return SUCCESS;
}

static long GetWeather(HTTPCli_Handle cli, int iSockID, char *pcCityName)
{

    char* pcBufLocation;
    long lRetVal = 0;
    int id;
    int len = 1;
    bool moreFlag = 0;
    char ** prevRespFilelds = NULL;
    HTTPCli_Field fields[2] =
            { { HTTPCli_FIELD_NAME_HOST, HOST_NAME }, { NULL, NULL }, };
    const char *ids[3] = {
    HTTPCli_FIELD_NAME_CONNECTION,
                           HTTPCli_FIELD_NAME_CONTENT_TYPE,
                           NULL };
    //
    // Set request fields
    //
    HTTPCli_setRequestFields(cli, fields);

    pcBufLocation = acSendBuff;
    strcpy(pcBufLocation, PREFIX_BUFFER);
    pcBufLocation += strlen(PREFIX_BUFFER);
    strcpy(pcBufLocation, pcCityName);
    pcBufLocation += strlen(pcCityName);
    strcpy(pcBufLocation, POST_BUFFER);
    pcBufLocation += strlen(POST_BUFFER);

    //
    // Make HTTP 1.1 GET request
    //
    lRetVal = HTTPCli_sendRequest(cli, HTTPCli_METHOD_GET, acSendBuff, 0);
    if (lRetVal < 0)
    {
        UART_PRINT("Failed to send HTTP 1.1 GET request.\n\r");
        return SERVER_GET_WEATHER_FAILED;
    }

    //
    // Test getResponseStatus: handle
    //
    lRetVal = HTTPCli_getResponseStatus(cli);
    if (lRetVal != 200)
    {
        UART_PRINT("HTTP Status Code: %d\r\n", lRetVal);
        FlushHTTPResponse(cli);
        return SERVER_GET_WEATHER_FAILED;
    }

    prevRespFilelds = HTTPCli_setResponseFields(cli, ids);

    //
    // Read response headers
    //
    while ((id = HTTPCli_getResponseField(cli, acRecvbuff, sizeof(acRecvbuff),
                                          &moreFlag)) != HTTPCli_FIELD_ID_END)
    {

        if (id == 0) // HTTPCli_FIELD_NAME_CONNECTION
        {
            if (!strncmp(acRecvbuff, "close", sizeof("close")))
            {
                UART_PRINT("Connection terminated by server.\n\r");
            }
        }
        else if (id == 1) // HTTPCli_FIELD_NAME_CONTENT_TYPE
        {
            UART_PRINT("Content Type: %s\r\n", acRecvbuff);
        }
    }

    HTTPCli_setResponseFields(cli, (const char **) prevRespFilelds);

    //
    // Read body
    //
    while (1)
    {
        len = HTTPCli_readResponseBody(cli, acRecvbuff, sizeof(acRecvbuff) - 1,
                                       &moreFlag);
        if (len < 0)
        {
            return SERVER_GET_WEATHER_FAILED;
        }
        acRecvbuff[len] = 0;

        if ((len - 2) >= 0 && acRecvbuff[len - 2] == '\r'
                && acRecvbuff[len - 1] == '\n')
        {
            break;
        }

        lRetVal = HandleXMLData(acRecvbuff);
        ASSERT_ON_ERROR(lRetVal);

        if (!moreFlag)
            break;

    }

    DBG_PRINT("\n\r****************************** \n\r");
    return SUCCESS;
}


int getAnalog(void)
{
    int value;

    unsigned char c1 = 0, c2 = 0;
    GPIOPinWrite(adc_cs.port, adc_cs.pin, 0x00);

    MAP_SPICSEnable(GSPI_BASE);

    SPITransfer(GSPI_BASE, 0, &c1, 1, (SPI_CS_ENABLE | SPI_CS_DISABLE));
    SPITransfer(GSPI_BASE, 0, &c2, 1, (SPI_CS_ENABLE | SPI_CS_DISABLE));
    MAP_SPICSDisable(GSPI_BASE);
    GPIOPinWrite(adc_cs.port, adc_cs.pin, 0xff);

    c1 = c1 << 3;
    c2 = c2 >> 3;
    unsigned int temp1 = c1 << 2;
    unsigned int temp2 = c2;
    value = temp1 | temp2;
    //value -= 388; // removing dc bias

    return value;
}

void GetWeatherTask(void *pvParameters)
{
    int iSocketDesc;
    int iRetVal;
    char acCityName[32];
    long lRetVal = -1;
    unsigned long ulDestinationIP;
    struct sockaddr_in addr;
    HTTPCli_Struct cli;
    int updateWeather = 1;

    DBG_PRINT("GET_WEATHER: Test Begin\n\r");

    //
    // Configure LED
    //
    GPIO_IF_LedConfigure(LED1 | LED3);

    GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);

    //
    // Reset The state of the machine
    //
    Network_IF_ResetMCUStateMachine();

    //
    // Start the driver
    //
    lRetVal = Network_IF_InitDriver(ROLE_STA);
    if (lRetVal < 0)
    {
        UART_PRINT("Failed to start SimpleLink Device\n\r");
        return;
    }

    // Switch on Green LED to indicate Simplelink is properly UP
    GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);

    //
    // Configure Timer for blinking the LED for IP acquisition
    //
    LedTimerConfigNStart();

    // Initialize AP security params
    SecurityParams.Key = (signed char *) SECURITY_KEY;
    SecurityParams.KeyLen = strlen(SECURITY_KEY);
    SecurityParams.Type = SECURITY_TYPE;

    //
    // Connect to the Access Point
    //
    lRetVal = Network_IF_ConnectAP(SSID_NAME, SecurityParams);
    if (lRetVal < 0)
    {
        UART_PRINT("Connection to an AP failed\n\r");
        LOOP_FOREVER()
        ;
    }

    //
    // Disable the LED blinking Timer as Device is connected to AP
    //
    LedTimerDeinitStop();

    //
    // Switch ON RED LED to indicate that Device acquired an IP
    //
    GPIO_IF_LedOn(MCU_IP_ALLOC_IND);

    //
    // Get the serverhost IP address using the DNS lookup
    //
    lRetVal = Network_IF_GetHostIP((char*) HOST_NAME, &ulDestinationIP);
    if (lRetVal < 0)
    {
        UART_PRINT("DNS lookup failed. \n\r", lRetVal);
        goto end;
    }

    //
    // Set up the input parameters for HTTP Connection
    //
    addr.sin_family = AF_INET;
    addr.sin_port = htons(HOST_PORT);
    addr.sin_addr.s_addr = sl_Htonl(ulDestinationIP);

    //
    // Testing HTTPCli open call: handle, address params only
    //
    HTTPCli_construct(&cli);
    lRetVal = HTTPCli_connect(&cli, (struct sockaddr *) &addr, 0, NULL);
    if (lRetVal < 0)
    {
        UART_PRINT("Failed to create instance of HTTP Client.\n\r");
        goto end;
    }
    int cityPtr = 0;

    //Enabling Interrupts for BUTTON 1 Input Mode

    Timer_IF_Init(PRCM_TIMERA0, TIMERA0_BASE, TIMER_CFG_PERIODIC, TIMER_A, 0);
    Timer_IF_IntSetup(TIMERA0_BASE, TIMER_A, TimerBaseIntHandler);
    Timer_IF_Start(TIMERA0_BASE, TIMER_A, 5000);

    Timer_IF_Init(PRCM_TIMERA1, TIMERA1_BASE, TIMER_CFG_PERIODIC, TIMER_A, 0);
    Timer_IF_IntSetup(TIMERA1_BASE, TIMER_A, T9Int);
    Timer_IF_Start(TIMERA1_BASE, TIMER_A, BUTTON_DELAY);

    setCursor(0, 64);

    while (1)
    {
        //
        // Get the city name over UART to get the weather info
        //

        char cities[5][20] = { "Davis", "San Francisco", "Tokyo", "London",
                               "Mumbai" };

//        UART_PRINT("\n\rEnter city name, or QUIT to quit: ");
//        iRetVal = GetCmd(acCityName, sizeof(acCityName));
        //int analog = getAnalog();
//        if (analog < 200 && cityPtr != 0)
//        {
//            cityPtr--;
//            fillScreen(RED);
//            updateWeather = 1;
//
//        }
//        if (analog > 900 && cityPtr != 4)
//        {
//            cityPtr++;
//            fillScreen(BLUE);
//            updateWeather = 1;
//        }

        if (sampleReady)
        {
            sampleReady = 0;
            samples[sampleIndex] = getADC();
        }

        if (sampleFull)
        {  // clear flag
            sampleFull = 0;
            processADC();

            if(decoded_letter == 1)
            {
                textMode();
            }

        }
        if (updateWeather)
        {

            //
            // Get the weather info and display the same
            //
            lRetVal = GetWeather(&cli, iSocketDesc, &(cities[cityPtr][0]));
            if (lRetVal == SERVER_GET_WEATHER_FAILED)
            {
                UART_PRINT("Server Get Weather failed \n\r");
                LOOP_FOREVER()
                ;
            }
            else if (lRetVal == WRONG_CITY_NAME)
            {
                UART_PRINT("Wrong input\n\r");

            }
            else if (lRetVal == NO_WEATHER_DATA)
            {
                UART_PRINT("Weather data not available\n\r");

            }
            else
            {
                Outstr((char*) cityPtr);
                Outstr(city);
                Outstr(temp);
                Outstr(condition);
                Outstr("$");

            }

            //
            // Wait a while before resuming
            //
            updateWeather = 0;

        }
    }

    HTTPCli_destruct(&cli);
    end:
    //
    // Stop the driver
    //
    lRetVal = Network_IF_DeInitDriver();
    if (lRetVal < 0)
    {
        UART_PRINT("Failed to stop SimpleLink Device\n\r");
        LOOP_FOREVER()
        ;
    }

    //
    // Switch Off RED & Green LEDs to indicate that Device is
    // disconnected from AP and Simplelink is shutdown
    //
    GPIO_IF_LedOff(MCU_IP_ALLOC_IND);
    GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);

    DBG_PRINT("GET_WEATHER: Test Complete\n\r");

    //
    // Loop here
    //
    LOOP_FOREVER()
    ;
}

static void DisplayBanner(char * AppName)
{
    UART_PRINT("\n\n\n\r");
    UART_PRINT("\t\t *************************************************\n\r");
    UART_PRINT("\t\t      CC3200 %s Application       \n\r", AppName);
    UART_PRINT("\t\t *************************************************\n\r");
    UART_PRINT("\n\n\n\r");
}

static void BoardInit(void)
{
    /* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
    //
    // Set vector table base
    //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long) &g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

void main()
{
    long lRetVal = -1;

    //
    // Initialize Board configurations
    //
    BoardInit();

    //
    // Pinmux for UART
    //
    PinMuxConfig();

    //
    // Initializing DMA
    //
    UDMAInit();
#ifndef NOTERM
    //
    // Configuring UART
    //
    InitTerm();
    ClearTerm();
#endif
    //
    // Display Application Banner
    //
    DisplayBanner(APP_NAME);

    //
    // Start the SimpleLink Host
    //

    MAP_PRCMPeripheralClkEnable(PRCM_GSPI, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralReset(PRCM_GSPI);

    MAP_SPIReset(GSPI_BASE);
    MAP_SPIConfigSetExpClk(GSPI_BASE, MAP_PRCMPeripheralClockGet(PRCM_GSPI),
    SPI_IF_BIT_RATE,
                           SPI_MODE_MASTER,
                           SPI_SUB_MODE_0,
                           (SPI_SW_CTRL_CS |
                           SPI_4PIN_MODE |
                           SPI_TURBO_OFF |
                           SPI_CS_ACTIVEHIGH |
                           SPI_WL_8));
    MAP_SPIEnable(GSPI_BASE);

    Adafruit_Init();
    fillScreen(GREEN);
    Report("Screen should start here\r\n");

    lRetVal = VStartSimpleLinkSpawnTask(SPAWN_TASK_PRIORITY);
    if (lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER()
        ;
    }

    //
    // Start the GetWeather task
    //
    lRetVal = osi_TaskCreate(GetWeatherTask,
                             (const signed char *) "Get Weather",
                             OSI_STACK_SIZE,
                             NULL, 1,
                             NULL);
    if (lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER()
        ;
    }

    //
    // Start the task scheduler
    //
    osi_start();

}
