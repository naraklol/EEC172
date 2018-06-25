//EEC 172 
//Neil Arakkal
//Daivik Dinesh

#include <string.h>

#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "prcm.h"
#include "uart.h"
#include "gpio.h"
#include "interrupt.h"
#include "Adafruit_GFX.h"
#include "Adafruit_OLED.h"
#include "Adafruit_SSD1351.h"
#include "glcdfont.h"
#include "test.h"
#include "timer.h"
#include "uart_if.h"
#include "timer_if.h"
#include "pinmux.h"

#define APPLICATION_VERSION     "1.1.1"

#define SPI_IF_BIT_RATE  1000000
#define TR_BUFF_SIZE     100

#define BUTTON_ONE      33468487 //
#define BUTTON_TWO      50180167 //
#define BUTTON_THREE    66891847 //
#define BUTTON_FOUR     83603527 //
#define BUTTON_FIVE     100315207 //
#define BUTTON_SIX      117026887 //
#define BUTTON_SEVEN    133738567 //
#define BUTTON_EIGHT    150450247 //
#define BUTTON_NINE     167161927 //
#define BUTTON_ZERO     16756807 //
#define BUTTON_CHUP     200585287 //
#define BUTTON_CHDOWN   183873607
#define BUTTON_DELAY    66666666
#define X_CHAR 6
#define Y_CHAR 9

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

typedef struct PinSetting
{
    unsigned long port;
    unsigned int pin;
} PinSetting;
static PinSetting ir = { .port = GPIOA3_BASE, .pin = 0x80 };

static void BoardInit(void)
{

#ifndef USE_TIRTOS

#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long) &g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif

    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

unsigned int ir_intflag = 0;
int count = 0;
int index = 0;
unsigned int num[100];
char txBuffer[128];
int buffIndex = 0;
int numCode = 0;
unsigned int txComplete = 0;
int lastPressed = 0;
int timeAfterPress = 0;
int rxCount = 0;

char rxBuffer[100];
char rxTemp;

static void UARTInt(void)
{
    unsigned long ulStatus;
    ulStatus = MAP_UARTIntStatus(UARTA1_BASE, true);
    MAP_UARTIntClear(UARTA1_BASE, ulStatus);

    if(UARTCharsAvail(UARTA1_BASE))
        rxTemp = (char) UARTCharGet(UARTA1_BASE);

    rxBuffer[rxCount] = rxTemp;
    rxCount++;

    if (rxTemp == '#')
    {
        rxCount = 0;
    }

    if (rxTemp == '$')
    {
        fillRect(0, 64, 128, 58, BLACK);
        int i = 0, x = 0, y = 64;
        for (i = 0; i < rxCount - 1; i++)
        {
            if (x > 124)
            {
                x = 0;
                y += Y_CHAR;
            } // when edge of screen, start at new line

            drawChar(x, y, rxBuffer[i], WHITE, BLACK, 1);
            x += 6;

        }
    }

}

static void IRInt(void)
{

    txComplete = 0;
    unsigned long ulStatus;

    ulStatus = MAP_GPIOIntStatus(ir.port, true);
    MAP_GPIOIntClear(ir.port, ulStatus);
    ir_intflag = 1;

    count = MAP_TimerValueGet(TIMERA0_BASE, TIMER_A);

    Timer_IF_Start(TIMERA0_BASE, TIMER_A, 0xffffffff);

    if (count < -160000 && count > -200000) // one
    {
        num[index] = 1;
        index++;
    }

    if (count > -120000 && count < -70000) // zero
    {
        num[index] = 0;
        index++;
    }

    if (count < -1200000)
    {
        index = 0;
        timeAfterPress = MAP_TimerValueGet(TIMERA1_BASE, TIMER_A);
        timeAfterPress = -timeAfterPress;
    }

    if (index == 32)
    {
        index = 0;
        txComplete = 1;
        Timer_IF_Start(TIMERA1_BASE, TIMER_A, 0xffffffff);
    }

}

void main()
{
    unsigned long ulStatus;

    BoardInit();
    PinMuxConfig();

    InitTerm();
    ClearTerm();

    MAP_GPIOIntRegister(ir.port, IRInt);
    MAP_GPIOIntTypeSet(ir.port, ir.pin, GPIO_FALLING_EDGE);
    ulStatus = MAP_GPIOIntStatus(ir.port, false);
    MAP_GPIOIntClear(ir.port, ulStatus);

    MAP_PRCMPeripheralClkEnable(PRCM_GSPI, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralReset(PRCM_GSPI);

    MAP_SPIReset(GSPI_BASE);
    MAP_SPIConfigSetExpClk(GSPI_BASE, MAP_PRCMPeripheralClockGet(PRCM_GSPI),
    SPI_IF_BIT_RATE,
                           SPI_MODE_MASTER, SPI_SUB_MODE_0, (SPI_SW_CTRL_CS |
                           SPI_4PIN_MODE |
                           SPI_TURBO_OFF |
                           SPI_CS_ACTIVEHIGH |
                           SPI_WL_8));
    MAP_SPIEnable(GSPI_BASE);

    Adafruit_Init();
    fillScreen(BLUE);

    MAP_GPIOIntEnable(ir.port, ir.pin);

    MAP_UARTEnable(UARTA1_BASE);
    MAP_UARTFIFOEnable(UARTA1_BASE);
    MAP_UARTConfigSetExpClk(UARTA1_BASE,
    MAP_PRCMPeripheralClockGet(PRCM_UARTA1),
                            UART_BAUD_RATE,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                            UART_CONFIG_PAR_NONE));
    MAP_UARTIntRegister(UARTA1_BASE, UARTInt);
    ulStatus = MAP_UARTIntStatus(UARTA1_BASE, false);
    MAP_GPIOIntClear(UARTA1_BASE, ulStatus);
    MAP_UARTIntEnable(UARTA1_BASE, (UART_INT_RX | UART_INT_RT));

    Timer_IF_Init(PRCM_TIMERA0, TIMERA0_BASE, TIMER_CFG_PERIODIC, TIMER_A, 0);
    Timer_IF_Init(PRCM_TIMERA1, TIMERA1_BASE, TIMER_CFG_PERIODIC, TIMER_A, 0);

    setCursor(0, 64);

    ir_intflag = 0;

    int x = 0;
    int y = 0;

    while (1)
    {/*
     unsigned int color[6] = { RED, BLUE, GREEN, YELLOW, MAGENTA, CYAN };
     int u;
     while (1)
     {
     for (u = 0; u < 6; u++)
     fillScreen(color[u]);
     }
     */
        while (ir_intflag == 0)
        {
            ;
        }

        if (lastPressed == 675)
        {
            fillRect(0, 0, 128, 58, BLACK);
            lastPressed = 0;
        }

        if (txComplete && ir_intflag)
        {  // clear flag

            ir_intflag = 0;
            txComplete = 0;

            int k = 0;
            numCode = 0;
            for (k = 31; k >= 0; k--)
            {
                numCode *= 2;
                numCode += num[k];
            }

            numCode = -numCode;

            switch (numCode)
            {

            case (BUTTON_ONE):
                Report("1");
                break;

            case (BUTTON_TWO):

                Report("2");

                if (timeAfterPress < BUTTON_DELAY)
                {
                    if (lastPressed == BUTTON_TWO + 1)
                    {

                        x -= X_CHAR;
                        txBuffer[--buffIndex] = 'C';
                        buffIndex++;
                        drawChar(x, y, 'B', BLACK, BLACK, 1);
                        drawChar(x, y, 'C', WHITE, BLACK, 1);
                        lastPressed = BUTTON_TWO + 2;
                        x += X_CHAR;
                    }
                    if (lastPressed == BUTTON_TWO)
                    {
                        x -= X_CHAR;
                        txBuffer[--buffIndex] = 'B';
                        buffIndex++;
                        drawChar(x, y, 'A', BLACK, BLACK, 1);
                        drawChar(x, y, 'B', WHITE, BLACK, 1);
                        lastPressed = BUTTON_TWO + 1;
                        x += X_CHAR;
                    }
                }
                if (timeAfterPress > BUTTON_DELAY
                        || (lastPressed != BUTTON_TWO + 1
                                && lastPressed != BUTTON_TWO + 2))
                {
                    drawChar(x, y, 'A', WHITE, BLACK, 1);
                    txBuffer[buffIndex++] = 'A';
                    lastPressed = BUTTON_TWO;
                    x += X_CHAR;
                }
                break;

            case (BUTTON_THREE):
                Report("3");

                if (timeAfterPress < BUTTON_DELAY)
                {
                    if (lastPressed == BUTTON_THREE + 1)
                    {
                        x -= X_CHAR;
                        txBuffer[--buffIndex] = 'F';
                        buffIndex++;
                        drawChar(x, y, 'E', BLACK, BLACK, 1);
                        drawChar(x, y, 'F', WHITE, BLACK, 1);
                        lastPressed = BUTTON_THREE + 2;
                        x += X_CHAR;
                    }
                    if (lastPressed == BUTTON_THREE)
                    {
                        x -= X_CHAR;
                        txBuffer[--buffIndex] = 'E';
                        buffIndex++;
                        drawChar(x, y, 'D', BLACK, BLACK, 1);
                        drawChar(x, y, 'E', WHITE, BLACK, 1);
                        lastPressed = BUTTON_THREE + 1;
                        x += X_CHAR;
                    }
                }
                if (timeAfterPress > BUTTON_DELAY
                        || (lastPressed != BUTTON_THREE + 1
                                && lastPressed != BUTTON_THREE + 2))
                {
                    drawChar(x, y, 'D', WHITE, BLACK, 1);
                    txBuffer[buffIndex++] = 'D';
                    lastPressed = BUTTON_THREE;
                    x += X_CHAR;
                }
                break;

            case (BUTTON_FOUR):
                Report("4");

                if (timeAfterPress < BUTTON_DELAY)
                {
                    if (lastPressed == BUTTON_FOUR + 1)
                    {
                        x -= X_CHAR;
                        txBuffer[--buffIndex] = 'I';
                        buffIndex++;
                        drawChar(x, y, 'H', BLACK, BLACK, 1);
                        drawChar(x, y, 'I', WHITE, BLACK, 1);
                        lastPressed = BUTTON_FOUR + 2;
                        x += X_CHAR;
                    }
                    if (lastPressed == BUTTON_FOUR)
                    {
                        x -= X_CHAR;
                        txBuffer[--buffIndex] = 'H';
                        buffIndex++;
                        drawChar(x, y, 'G', BLACK, BLACK, 1);
                        drawChar(x, y, 'H', WHITE, BLACK, 1);
                        lastPressed = BUTTON_FOUR + 1;
                        x += X_CHAR;
                    }
                }
                if (timeAfterPress > BUTTON_DELAY
                        || (lastPressed != BUTTON_FOUR + 1
                                && lastPressed != BUTTON_FOUR + 2))
                {
                    drawChar(x, y, 'G', WHITE, BLACK, 1);
                    txBuffer[buffIndex++] = 'G';
                    lastPressed = BUTTON_FOUR;
                    x += X_CHAR;
                }
                break;

            case (BUTTON_FIVE):
                Report("5");

                if (timeAfterPress < BUTTON_DELAY)
                {
                    if (lastPressed == BUTTON_FIVE + 1)
                    {
                        x -= X_CHAR;
                        txBuffer[--buffIndex] = 'L';
                        buffIndex++;
                        drawChar(x, y, 'K', BLACK, BLACK, 1);
                        drawChar(x, y, 'L', WHITE, BLACK, 1);
                        lastPressed = BUTTON_FIVE + 2;
                        x += X_CHAR;
                    }
                    if (lastPressed == BUTTON_FIVE)
                    {
                        x -= X_CHAR;
                        txBuffer[--buffIndex] = 'K';
                        buffIndex++;
                        drawChar(x, y, 'J', BLACK, BLACK, 1);
                        drawChar(x, y, 'K', WHITE, BLACK, 1);
                        lastPressed = BUTTON_FIVE + 1;
                        x += X_CHAR;
                    }
                }
                if (timeAfterPress > BUTTON_DELAY
                        || (lastPressed != BUTTON_FIVE + 1
                                && lastPressed != BUTTON_FIVE + 2))
                {
                    drawChar(x, y, 'J', WHITE, BLACK, 1);
                    txBuffer[buffIndex++] = 'J';
                    lastPressed = BUTTON_FIVE;
                    x += X_CHAR;
                }
                break;

            case (BUTTON_SIX):
                Report("6");

                if (timeAfterPress < BUTTON_DELAY)
                {
                    if (lastPressed == BUTTON_SIX + 1)
                    {
                        x -= X_CHAR;
                        txBuffer[--buffIndex] = 'O';
                        buffIndex++;
                        drawChar(x, y, 'N', BLACK, BLACK, 1);
                        drawChar(x, y, 'O', WHITE, BLACK, 1);
                        lastPressed = BUTTON_SIX + 2;
                        x += X_CHAR;
                    }
                    if (lastPressed == BUTTON_SIX)
                    {
                        x -= X_CHAR;
                        txBuffer[--buffIndex] = 'N';
                        buffIndex++;
                        drawChar(x, y, 'M', BLACK, BLACK, 1);
                        drawChar(x, y, 'N', WHITE, BLACK, 1);
                        lastPressed = BUTTON_SIX + 1;
                        x += X_CHAR;
                    }
                }
                if (timeAfterPress > BUTTON_DELAY
                        || (lastPressed != BUTTON_SIX + 1
                                && lastPressed != BUTTON_SIX + 2))
                {
                    drawChar(x, y, 'M', WHITE, BLACK, 1);
                    txBuffer[buffIndex++] = 'M';
                    lastPressed = BUTTON_SIX;
                    x += X_CHAR;
                }

                break;

            case (BUTTON_SEVEN):
                Report("7");

                if (timeAfterPress < BUTTON_DELAY)
                {
                    if (lastPressed == BUTTON_SEVEN + 2)
                    {
                        x -= X_CHAR;
                        txBuffer[--buffIndex] = 'S';
                        buffIndex++;
                        drawChar(x, y, 'R', BLACK, BLACK, 1);
                        drawChar(x, y, 'S', WHITE, BLACK, 1);
                        lastPressed = BUTTON_SEVEN + 3;
                        x += X_CHAR;
                    }
                    if (lastPressed == BUTTON_SEVEN + 1)
                    {
                        x -= X_CHAR;
                        txBuffer[--buffIndex] = 'R';
                        buffIndex++;
                        drawChar(x, y, 'Q', BLACK, BLACK, 1);
                        drawChar(x, y, 'R', WHITE, BLACK, 1);
                        lastPressed = BUTTON_SEVEN + 2;
                        x += X_CHAR;
                    }
                    if (lastPressed == BUTTON_SEVEN)
                    {
                        x -= X_CHAR;
                        txBuffer[--buffIndex] = 'Q';
                        buffIndex++;
                        drawChar(x, y, 'P', BLACK, BLACK, 1);
                        drawChar(x, y, 'Q', WHITE, BLACK, 1);
                        lastPressed = BUTTON_SEVEN + 1;
                        x += X_CHAR;
                    }
                }
                if (timeAfterPress > BUTTON_DELAY
                        || (lastPressed != BUTTON_SEVEN + 1
                                && lastPressed != BUTTON_SEVEN + 2
                                && lastPressed != BUTTON_SEVEN + 3))
                {
                    drawChar(x, y, 'P', WHITE, BLACK, 1);
                    txBuffer[buffIndex++] = 'P';
                    lastPressed = BUTTON_SEVEN;
                    x += X_CHAR;
                }
                break;

            case (BUTTON_EIGHT):
                Report("8");

                if (timeAfterPress < BUTTON_DELAY)
                {
                    if (lastPressed == BUTTON_EIGHT + 1)
                    {
                        x -= X_CHAR;
                        txBuffer[--buffIndex] = 'V';
                        buffIndex++;
                        drawChar(x, y, 'U', BLACK, BLACK, 1);
                        drawChar(x, y, 'V', WHITE, BLACK, 1);
                        lastPressed = BUTTON_EIGHT + 2;
                        x += X_CHAR;
                    }
                    if (lastPressed == BUTTON_EIGHT)
                    {
                        x -= X_CHAR;
                        txBuffer[--buffIndex] = 'U';
                        buffIndex++;
                        drawChar(x, y, 'T', BLACK, BLACK, 1);
                        drawChar(x, y, 'U', WHITE, BLACK, 1);
                        lastPressed = BUTTON_EIGHT + 1;
                        x += X_CHAR;
                    }
                }
                if (timeAfterPress > BUTTON_DELAY
                        || (lastPressed != BUTTON_EIGHT + 1
                                && lastPressed != BUTTON_EIGHT + 2))
                {
                    drawChar(x, y, 'T', WHITE, BLACK, 1);
                    txBuffer[buffIndex++] = 'T';
                    lastPressed = BUTTON_EIGHT;
                    x += X_CHAR;
                }
                break;

            case (BUTTON_NINE):
                Report("9");

                if (timeAfterPress < BUTTON_DELAY)
                {
                    if (lastPressed == BUTTON_NINE + 2)
                    {
                        x -= X_CHAR;
                        txBuffer[--buffIndex] = 'Z';
                        buffIndex++;
                        drawChar(x, y, 'Y', BLACK, BLACK, 1);
                        drawChar(x, y, 'Z', WHITE, BLACK, 1);
                        lastPressed = BUTTON_NINE + 3;
                        x += X_CHAR;
                    }
                    if (lastPressed == BUTTON_NINE + 1)
                    {
                        x -= X_CHAR;
                        txBuffer[--buffIndex] = 'Y';
                        buffIndex++;
                        drawChar(x, y, 'X', BLACK, BLACK, 1);
                        drawChar(x, y, 'Y', WHITE, BLACK, 1);
                        lastPressed = BUTTON_NINE + 2;
                        x += X_CHAR;
                    }
                    if (lastPressed == BUTTON_NINE)
                    {
                        x -= X_CHAR;
                        txBuffer[--buffIndex] = 'X';
                        buffIndex++;
                        drawChar(x, y, 'W', BLACK, BLACK, 1);
                        drawChar(x, y, 'X', WHITE, BLACK, 1);
                        lastPressed = BUTTON_NINE + 1;
                        x += X_CHAR;
                    }
                }
                if (timeAfterPress > BUTTON_DELAY
                        || (lastPressed != BUTTON_NINE + 1
                                && lastPressed != BUTTON_NINE + 2
                                && lastPressed != BUTTON_NINE + 3))
                {
                    drawChar(x, y, 'W', WHITE, BLACK, 1);
                    txBuffer[buffIndex++] = 'W';
                    lastPressed = BUTTON_NINE;
                    x += X_CHAR;
                }
                break;

            case (BUTTON_ZERO): // SPACE

                Report("0");
                txBuffer[buffIndex++] = ' ';
                x += X_CHAR;
                lastPressed = 0;
                break;

            case (BUTTON_CHUP): //SEND MESSAGE

                Report("CHANNEL UP");
                lastPressed = 675;

                int q;
                char tempBuffer[100];

                tempBuffer[0] = '#';
                for (q = 0; q < buffIndex; q++)
                    tempBuffer[q + 1] = txBuffer[q];
                tempBuffer[buffIndex + 1] = '$';

                for (q = 0; q < buffIndex + 2; q++)
                {
                    UARTCharPut(UARTA1_BASE, tempBuffer[q]);
                }

                Report("\r\n");
                buffIndex = 0;
                x = 0;
                y = 0;
                fillRect(0, 0, 128, 58, BLACK);

                break;

            case (BUTTON_CHDOWN): //BACKSPACE
                Report("CHANNEL DOWN");

                if (buffIndex != 0)
                    buffIndex--;

                lastPressed = 0;

                if (x > 0)
                {
                    x -= X_CHAR;
                    fillRect(x, y, X_CHAR, Y_CHAR, BLACK);
                }

                break;
            default:
                Report("Unknown code received %d", numCode);
                break;

            }
            Report("\r\n");
            if (x > 124)
            {
                x = 0;
                y += Y_CHAR;
            }
        }
    }

}
