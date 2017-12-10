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



/* BEGIN RJ's Code */
#include "FreeRTOS.h"   // FreeRTOS constructs
#include "LPC17xx.h"    // LPC_GPIOx, PINSELX
#include "stdint.h"     // uintX_t
#include "printf_lib.h" // u0_dbg_printf()
#include "adc0.h"       // ADC0 api
#include "keypad/keypad.h"

//Delwin's includes
#include "ssp0.h"       //sends codec song data
#include "ssp1.h"       //maybe to speak to lcd?
#include "ff.h"         //file access fatfs
#include <stdio.h>      //printf

#define DBG_KEYPAD 1

FIL file;

TaskHandle_t xinitialize = NULL;
TaskHandle_t xplay_music = NULL;
TaskHandle_t xbuttons = NULL;

TaskHandle_t xHandleKeypadRead = NULL;
char last_key = '\0';

void initialize(void *p)
{
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
    }
}


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
                last_key = kp.key(avg);
                #if DBG_KEYPAD
                u0_dbg_printf("avg:%d %c\n", avg, (last_key == '\0') ? 'x' : last_key);
                #endif
                avg = 0;
                cnt = 0;
                running_total = 0;
                break;
            }
            default: {
                running_total += adc0_get_reading(4);  // reads from adc0 channel 3 (i.e. P0.26)
                cnt++;
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

    xTaskCreate(initialize, "initialize", STACK_BYTES(2048), NULL, PRIORITY_HIGH, &xinitialize);
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
    xTaskCreate(keypadRead, "keypad", STACK_BYTES(1024), 0, PRIORITY_HIGH, &xHandleKeypadRead);
    /* END RJ's Code */

    scheduler_start(); ///< This shouldn't return
    return -1;
}
