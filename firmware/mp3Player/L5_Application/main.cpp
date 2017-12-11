/*
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */

/**
 * @file
 * @brief This is the application entry point.
 * 			FreeRTOS and stdio printf is pre-configured to use uart0_min.h before main() enters.
 * 			@see L0_LowLevel/lpc_sys.h if you wish to override printf/scanf functions.
 *
 */
#include "tasks.hpp"
#include "examples/examples.hpp"
#include "periodic_scheduler/periodic_callback.h"
#include "uart2.hpp"
#include "uart3.hpp"
#include "utilities.h"



/* BEGIN RJ's Includes */
#include "FreeRTOS.h"   // FreeRTOS constructs
#include "LPC17xx.h"    // LPC_GPIOx, PINSELX
#include "stdint.h"     // uintX_t
#include "printf_lib.h" // u0_dbg_printf()
#include "adc0.h"       // ADC0 api
#include "keypad/keypad.h"
/* END RJ's Includes */

//Delwin's includes
#include "ssp0.h"       //sends codec song data
#include "ssp1.h"       //maybe to speak to lcd?
#include "ff.h"         //file access fatfs
#include <stdio.h>      //printf

/* BEGIN RJ's Defines */
// VS1053 Audio Codec-Specific Defines
#define VS_READ 0x03
#define VS_WRITE 0X02

// Volume Control Defines
#define VOL_STEP_SIZE 10   // the amount of volume points to increase/decrease by (every 1 step = 0.5db)
#define VOL_LESS 0
#define VOL_MORE 1

// Control Unit Defines
#define CU_MIN_RPT_COUNT 4  // the minimum value of repeatCnt for the CU to accept last_key as a valid input character

// Misc Debug Controls
#define DBG_KEYPAD 0        // enables keypadRead() debug messages
#define DBG_CU 1            // enables controlUnit() debug messages
/* END RJ's Defines */

FIL file;

TaskHandle_t xinitialize = NULL;
TaskHandle_t xplay_music = NULL;
TaskHandle_t xbuttons = NULL;

/* BEGIN RJ's Globals */
TaskHandle_t xHandleKeypadRead = NULL;
TaskHandle_t xHandleControlUnit = NULL;
char last_key = '\0';
uint8_t repeatCnt = 0;
uint8_t currentVolume = 0x2F;    // the volume val for both channels; half-volume = 0x7F = 127; max = 0x00 = 0; min = 0xFE = 254

typedef enum {
    PLAYING,    // playing a song
    PAUSED,     // a song is paused (somewhere in the middle)
    HALTED,     // the VS is busy, or we need to do some temporary manipulation to it for a short while
    IDLE        // doing nothing (i.e. a song was stopped and must be restarted from 0:00, or a song has yet to be loaded)
} mp3_state;
mp3_state currentState = PLAYING;
mp3_state prevState = IDLE;
/* END RJ's Globals */

/* BEGIN RJ's Mutexes */
SemaphoreHandle_t xKeypadValueMutex = NULL; // for protecting last_key and repeatCnt
SemaphoreHandle_t xStateMutex = NULL;       // for protecting currentState and prevState
SemaphoreHandle_t xVolumeMutex = NULL;      // for protecting currentVolume
/* END RJ's Mutexes */

/* BEGIN RJ's Function Declarations */
bool volumeSet (uint8_t val);
/* END RJ's Function Declarations */

void initialize(void *p)
{
    // Create mutexes for specific parameters
    xKeypadValueMutex = xSemaphoreCreateMutex();
    xStateMutex = xSemaphoreCreateMutex();
    xVolumeMutex = xSemaphoreCreateMutex();

    //XDCS
    LPC_GPIO1->FIODIR |= (1 << 19);

    //XCS
    LPC_GPIO1->FIODIR |= (1 << 20);
    LPC_GPIO1->FIOSET = (1 << 20);

    //RST
    LPC_GPIO1->FIODIR |= (1 << 22);

    //DREQ
    LPC_GPIO1->FIODIR &= ~(1 << 23);

    LPC_GPIO1->FIOCLR = (1 << 22);
    LPC_GPIO1->FIOSET = (1 << 22);

    ssp0_init(0);

    static bool stop_run = false;

    char* pFilename = "1:Despasito.mp3";
    unsigned int bytesRead = 0;

    if(!stop_run)
    {
        if(LPC_GPIO1->FIOPIN & (1 << 20))
        {
            // printf("XCS is high.\n");
        }
        else 
        {
            // printf("XCS is low.\n");
        }

        if(LPC_GPIO1->FIOPIN & (1 << 23))
        {
            LPC_GPIO1->FIOCLR = (1 << 20);

            // printf("Reading from MODE register");

            ssp0_exchange_byte(0x02);
            ssp0_exchange_byte(0x03);
            char b2 = ssp0_exchange_byte(0xf8);
            char b3 = ssp0_exchange_byte(0x00);

            // printf("%X%X\n", b2, b3);
            LPC_GPIO1->FIOSET = (1 << 20);
        }


        if(LPC_GPIO1->FIOPIN & (1 << 23))
        {
            LPC_GPIO1->FIOCLR = (1 << 20);
            // printf("Writing to MODE register\n");
            
            ssp0_exchange_byte(0x02);
            ssp0_exchange_byte(0x00);
            ssp0_exchange_byte(0x48);
            ssp0_exchange_byte(0x40);

            LPC_GPIO1->FIOSET = (1 << 20);
        }

        // RJ: Set an initial volume
        while (!volumeSet(currentVolume)) {
            // try again
        }

        stop_run = true;

        //SD read
        

        f_open(&file, pFilename, FA_OPEN_EXISTING | FA_READ);
    }

    vTaskSuspend(NULL);

}

void play_music(void *p)
{
    while(1)
    {
        if (xStateMutex != NULL) {
            if (xSemaphoreTake(xStateMutex, portMAX_DELAY) == pdTRUE) {
                if (currentState != PLAYING) {
                    xSemaphoreGive(xStateMutex);
                    vTaskSuspend(NULL); // have this task suspend itself (in an effort to prevent a case where the CU pauses this task while p1.19 is still LOW)
                }

                if(LPC_GPIO1->FIOPIN & (1 << 23))
                {
                    static uint32_t offset = 0;
                    char buffer[32] = {0};
                    static int send_this = 0;
                    static int put_here = 0;
                    unsigned int bytesRead = 0;
            
                    f_read(&file, &buffer, 32, &bytesRead);
                    if(bytesRead < 32)
                    {
                        f_close(&file);
                    }
                    
                    offset = offset + 32;

                    LPC_GPIO1->FIOCLR = (1 << 19);
            
                    for(int i = 0; i < 32; i++)
                    {
                    ssp0_exchange_byte(buffer[i]);
                    }
            
                    LPC_GPIO1->FIOSET = (1 << 19);
                }
                xSemaphoreGive(xStateMutex);
            }
        }
    }
}

/* BEGIN RJ's Code */
// @function    changeState
// @parameter   new_state - the state to change to
// @returns     n/a
// @details     This function updates the current state with the value of new_state, while also updating prevState with the last known state from currentState
void changeState (mp3_state new_state) {
    // Use mutex to acquire access to guarded resources
    if (xStateMutex != NULL) {
        if (xSemaphoreTake(xStateMutex, portMAX_DELAY) == pdTRUE) {
            prevState = currentState;
            currentState = new_state;
            xSemaphoreGive(xStateMutex);
        }
    }
    return;
}

// @function    keypadRead
// @parameter   n/a
// @returns     n/a
// @details     This function comprises the keypad reader task, whose purpose is to determine the pressed key using the ADC value taken from the keypad circuit
void keypadRead (void* p) {
    // Init P1.30 as ADC0.4
    LPC_PINCON->PINSEL3 |= (3 << 28);   // sets PINSEL3[29:28] bits to 11
    uint8_t cnt = 0;
    uint16_t avg = 0;
    uint16_t running_total = 0;
    keypad kp;

    // Main task loop
    while (1) {
        vTaskDelay(5);
        switch (cnt) {
            case 5: {  // every 10 measurements, print the avg and use that to determine which key is pressed
                avg = running_total/cnt;
                char temp = kp.key(avg);

                // Use mutex to acquire access to guarded resources (i.e. last_key and repeatCnt)
                if (xKeypadValueMutex != NULL) {
                    if (xSemaphoreTake(xKeypadValueMutex, portMAX_DELAY) == pdTRUE) {
                        // Key Noise Resolution: If current read matches the previous key, increment the key counter (a larger key count means this character is most likely the intended button being pressed). A sufficiently high repeatCnt will invoke the Control Unit to process the current key
                        switch (temp != '\0' && temp == last_key) {
                            case 1: {
                                repeatCnt++;
                                break;
                            }
                            default: {  // either too much button noise, or we are getting the '\0' character
                                repeatCnt = 0;
                                break;
                            }
                        }
                        last_key = temp;
                        #if DBG_KEYPAD
                        u0_dbg_printf("key:%d %c\n", avg, (last_key == '\0') ? 'x' : last_key);
                        #endif
                        avg = 0;
                        cnt = 0;
                        running_total = 0;
                        xSemaphoreGive(xKeypadValueMutex);
                    }
                }
                break;
            }
            default: {
                running_total += adc0_get_reading(4);  // reads from adc0 channel 3 (i.e. P0.26)
                cnt++;
            }
        }
    }
}

// @function    volumeControl
// @parameter   dir - a uin8_t acting as a bool to control whether to increase (i.e. 1), or decrease (i.e. 0) volume by a preset amount
// @returns     False, if the operation couldn't be completed. True, otherwise
// @details     This function comprises the task that increases or decreases the volume (depending on the value dir) by steps the size of VOL_STEP_SIZE
bool volumeControl (uint8_t dir) {
    bool success = false;

    // Take volume mutex
    if (xVolumeMutex != NULL) {
        if (xSemaphoreTake(xVolumeMutex, portMAX_DELAY) == pdTRUE) {
            // Only assert control commands to SCI if DREQ is high
            if (LPC_GPIO1->FIOPIN & (1 << 23)) {
                LPC_GPIO1->FIOCLR = (1 << 20);  // drive XCS LOW

                ssp0_exchange_byte(VS_WRITE);
                ssp0_exchange_byte(0x0B);   // VOL register
                switch (dir) {
                    // Increase volume
                    case 1: {
                        currentVolume = (currentVolume - VOL_STEP_SIZE < 0) ? 0 : (currentVolume - VOL_STEP_SIZE);
                        break;
                    }

                    // Decrease volume
                    default: {
                        currentVolume = (currentVolume + VOL_STEP_SIZE > 0xFE) ? 0xFE : (currentVolume + VOL_STEP_SIZE);
                        break;
                    }
                }
                ssp0_exchange_byte(currentVolume);  // control volume of first speaker (L or R? Which is modified first?)
                ssp0_exchange_byte(currentVolume);  // control volume of other speaker
                success = true;
                
                LPC_GPIO1->FIOSET = (1 << 20);  // pull XCS HIGH
            }
            xSemaphoreGive(xVolumeMutex);
        }
    }

    return success;
}

// @function    volumeSet
// @parameter   val - the uint8_t value to set the volume register to
// @returns     False, if the operation couldn't be completed. True, otherwise
// @details     This function allows the volume to be set manually to a specific value in contrast to volumeControl(), which only allows relative volume changes (i.e. either louder or quieter)
// @note        EXERCISE CAUTION, since this function DOES NOT CHECK if val is within bounds (i.e. 0 <= val <= 254)
//              Also, this function updates currentVolume with the new value
bool volumeSet (uint8_t val) {
    bool success = false;

    // Take volume mutex
    if (xVolumeMutex != NULL) {
        if (xSemaphoreTake(xVolumeMutex, portMAX_DELAY) == pdTRUE) {
            // Only assert control commands to SCI if DREQ is high
            if (LPC_GPIO1->FIOPIN & (1 << 23)) {
                LPC_GPIO1->FIOCLR = (1 << 20);  // drive XCS LOW

                ssp0_exchange_byte(VS_WRITE);
                ssp0_exchange_byte(0x0B);   // VOL register
                ssp0_exchange_byte(val);    // control volume of first speaker (L or R? Which is modified first?)
                ssp0_exchange_byte(val);    // control volume of other speaker
                currentVolume = val;        // update current volume level
                success = true;
                
                LPC_GPIO1->FIOSET = (1 << 20);  // pull XCS HIGH
            }
            xSemaphoreGive(xVolumeMutex);
        }
    }

    return success;
}

// @function    controlUnit
// @parameter   n/a
// @returns     n/a
// @details     This function manages state changes to control the operation of the mp3 player
void controlUnit (void* p) {
    // Init CU here...

    // Main control loop
    while (1) {
        vTaskDelay(50);

        if (xKeypadValueMutex != NULL) {
            if (xSemaphoreTake(xKeypadValueMutex, portMAX_DELAY) == pdTRUE) {
                // Only do something if key value wasn't noisy
                if (repeatCnt >= CU_MIN_RPT_COUNT) {
                    // Determine action based on accepted key value
                    switch (last_key) {
                        case '1': {
                            // Do something
                            #if DBG_CU
                            u0_dbg_printf("CU: Playing\n");
                            #endif
                            changeState(PLAYING);
                            vTaskResume(xplay_music);
                            break;
                        }
                        case '2': {
                            // Do something
                            #if DBG_CU
                            u0_dbg_printf("CU: Pausing\n");
                            #endif
                            changeState(PAUSED);
                            break;
                        }
                        case '3': { // Decrease Volume
                            #if DBG_CU
                            u0_dbg_printf("CU: V-\n");
                            #endif
                            changeState(HALTED);
                            while (!volumeControl(VOL_LESS)) {
                                // Redo volume control
                                #if DBG_CU
                                u0_dbg_printf("...\n");
                                #endif
                            }
                            #if DBG_CU
                            u0_dbg_printf("vol:-%5fdB\n", ((float) currentVolume) * 0.5);
                            #endif
                            changeState(PLAYING);
                            break;
                        }
                        case 'A': { // Increase Volume
                            #if DBG_CU
                            u0_dbg_printf("CU: V+\n");
                            #endif
                            changeState(HALTED);
                            while (!volumeControl(VOL_MORE)) {
                                // Redo volume control
                                #if DBG_CU
                                u0_dbg_printf("...\n");
                                #endif
                            }
                            #if DBG_CU
                            u0_dbg_printf("vol:-%5fdB\n", ((float) currentVolume) * 0.5);
                            #endif
                            changeState(PLAYING);
                            break;
                        }
                        default: {
                            // If last_key == '\0' or is unknown, do nothing
                            break;
                        }
                    }

                    // Reset key repeatCnt
                    repeatCnt = 0;
                }
                xSemaphoreGive(xKeypadValueMutex);
            }
        }
    }
}
/* END RJ's Code */



/**
 * The main() creates tasks or "threads".  See the documentation of scheduler_task class at scheduler_task.hpp
 * for details.  There is a very simple example towards the beginning of this class's declaration.
 *
 * @warning SPI #1 bus usage notes (interfaced to SD & Flash):
 *      - You can read/write files from multiple tasks because it automatically goes through SPI semaphore.
 *      - If you are going to use the SPI Bus in a FreeRTOS task, you need to use the API at L4_IO/fat/spi_sem.h
 *
 * @warning SPI #0 usage notes (Nordic wireless)
 *      - This bus is more tricky to use because if FreeRTOS is not running, the RIT interrupt may use the bus.
 *      - If FreeRTOS is running, then wireless task may use it.
 *        In either case, you should avoid using this bus or interfacing to external components because
 *        there is no semaphore configured for this bus and it should be used exclusively by nordic wireless.
 */
int main(void)
{
    /**
     * A few basic tasks for this bare-bone system :
     *      1.  Terminal task provides gateway to interact with the board through UART terminal.
     *      2.  Remote task allows you to use remote control to interact with the board.
     *      3.  Wireless task responsible to receive, retry, and handle mesh network.
     *
     * Disable remote task if you are not using it.  Also, it needs SYS_CFG_ENABLE_TLM
     * such that it can save remote control codes to non-volatile memory.  IR remote
     * control codes can be learned by typing the "learn" terminal command.
     */
    scheduler_add_task(new terminalTask(PRIORITY_HIGH));

    /* Consumes very little CPU, but need highest priority to handle mesh network ACKs */
    scheduler_add_task(new wirelessTask(PRIORITY_CRITICAL));

    xTaskCreate(initialize, "initialize", STACK_BYTES(2048), NULL, PRIORITY_CRITICAL, &xinitialize);
    xTaskCreate(play_music, "play_music", STACK_BYTES(2048), NULL, PRIORITY_MEDIUM, &xplay_music);

    /* Change "#if 0" to "#if 1" to run period tasks; @see period_callbacks.cpp */
    #if 0
    const bool run_1Khz = false;
    scheduler_add_task(new periodicSchedulerTask(run_1Khz));
    #endif

    /* The task for the IR receiver to "learn" IR codes */
    // scheduler_add_task(new remoteTask  (PRIORITY_LOW));

    /* Your tasks should probably used PRIORITY_MEDIUM or PRIORITY_LOW because you want the terminal
     * task to always be responsive so you can poke around in case something goes wrong.
     */

    /**
     * This is a the board demonstration task that can be used to test the board.
     * This also shows you how to send a wireless packets to other boards.
     */
    #if 0
        scheduler_add_task(new example_io_demo());
    #endif

    /**
     * Change "#if 0" to "#if 1" to enable examples.
     * Try these examples one at a time.
     */
    #if 0
        scheduler_add_task(new example_task());
        scheduler_add_task(new example_alarm());
        scheduler_add_task(new example_logger_qset());
        scheduler_add_task(new example_nv_vars());
    #endif

    /**
	 * Try the rx / tx tasks together to see how they queue data to each other.
	 */
    #if 0
        scheduler_add_task(new queue_tx());
        scheduler_add_task(new queue_rx());
    #endif

    /**
     * Another example of shared handles and producer/consumer using a queue.
     * In this example, producer will produce as fast as the consumer can consume.
     */
    #if 0
        scheduler_add_task(new producer());
        scheduler_add_task(new consumer());
    #endif

    /**
     * If you have RN-XV on your board, you can connect to Wifi using this task.
     * This does two things for us:
     *   1.  The task allows us to perform HTTP web requests (@see wifiTask)
     *   2.  Terminal task can accept commands from TCP/IP through Wifly module.
     *
     * To add terminal command channel, add this at terminal.cpp :: taskEntry() function:
     * @code
     *     // Assuming Wifly is on Uart3
     *     addCommandChannel(Uart3::getInstance(), false);
     * @endcode
     */
    #if 0
        Uart3 &u3 = Uart3::getInstance();
        u3.init(WIFI_BAUD_RATE, WIFI_RXQ_SIZE, WIFI_TXQ_SIZE);
        scheduler_add_task(new wifiTask(Uart3::getInstance(), PRIORITY_LOW));
    #endif


    /* BEGIN RJ's Code */
    xTaskCreate(keypadRead, "keypad", STACK_BYTES(1024), 0, PRIORITY_MEDIUM, &xHandleKeypadRead);
    xTaskCreate(controlUnit, "mp3_cu", STACK_BYTES(1024), 0, PRIORITY_HIGH, &xHandleControlUnit);
    /* END RJ's Code */

    scheduler_start(); ///< This shouldn't return
    return -1;
}
