/*
 *     SocialLedge.com - Copyright (C) 201
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
#include "wireless.h"
#include "stdio.h"
#include "utilities.h"

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

void u2_init(int baud = 9600)
{
	LPC_SC -> PCONP |= (1 << 24); //sets UART2 power/clock control bit
	LPC_SC -> PCLKSEL1 &= ~(1 << 17); //PCLK divider = 01 {17,16}
	LPC_SC -> PCLKSEL1 |= (1 << 16); //PCLK divider = 01 {17,16}

	//TX
	LPC_PINCON -> PINSEL4 &= ~(3 << 16); //Clear P2.8
	LPC_PINCON -> PINSEL4 |= (2 << 16); //Initialize P2.8
	//RX
	LPC_PINCON -> PINSEL4 &= ~(3 << 18); //Clear P2.9
	LPC_PINCON -> PINSEL4 |= (2 << 18); //Initialize P2.9

	LPC_UART2 -> LCR |= (3); //8-bit length
	LPC_UART2 -> LCR |= (1 << 7); //DLAB1 = 1
	uint16_t div = (48 * 1000 * 1000)/(16 * baud);
	LPC_UART2 -> DLM = (div >> 8);
	LPC_UART2 -> DLL = (div >> 0);
	LPC_UART2 -> LCR &= ~(1 << 7); //DLAB1 = 0

}

void u2_send (char out)
{
	LPC_UART2 -> THR = out;
	//		printf("sent char = %c \n\n", out);
	while (1) {
		if (LPC_UART2 -> LSR & (1 << 5)) {
			break;
		}
	}
}

char u2_receive (void)
{
	while (1) {
		if (LPC_UART2 -> LSR & (1 << 0)) {
			break;
		}
	}
	char in = LPC_UART2 -> RBR;
	return in;
}

class gpio_task: public scheduler_task {
public:
    gpio_task(uint8_t priority) :
        scheduler_task("GPIO", 2000, priority) {
    }
    bool run(void*p) {
        if(LPC_GPIO2->FIOPIN & ( 1 << 0)){
            LPC_GPIO1->FIOPIN &= ~(1 << 0); //SET
//            printf("No mail\n");
            vTaskDelay(1000);
        }else{
            LPC_GPIO1->FIOPIN |= (1 << 0); //RESET/CLEAR
            printf("You have mail!!!.\n");
            nordic_sender();
            vTaskDelay(1000);
        }

        return true;
    }

    bool init(void) {
        //Switch
        LPC_GPIO2->FIODIR &= ~( 1 << 0 );

        //LED
        LPC_GPIO1->FIODIR |= ( 1 << 0 ); //Direction
        LPC_GPIO1->FIOPIN |= ( 1 << 0 ); //Selection

        return true;
    }
    bool nordic_receiver(void) {
        char var1;
        char var2;
        char var3;
        char var4;
        char var5;
        int count = 0;
        while(1) {
            mesh_packet_t pkt;
            if(wireless_get_rx_pkt(&pkt, 100)) {
                wireless_deform_pkt(&pkt, 5,
                                    &var1, sizeof(var1),
                                    &var2, sizeof(var2),
                                    &var3, sizeof(var3),
                                    &var4, sizeof(var4),
                                    &var5, sizeof(var5));
            printf("receiver: "); //Used for debugging
            printf("%c", var1); //Used for debugging
            printf("%c", var2); //Used for debugging
            printf("%c", var3); //Used for debugging
            printf("%c", var4); //Used for debugging
            printf("%c\n", var5); //Used for debugging

            //Send to phone that is connected to the device.
            u2_send(var1);
            u2_send(var2);
            u2_send(var3);
            u2_send(var4);
            u2_send(var5);
            //printf("count: %i\n\n", count); //used for debugging.
            return true;
            }
            //count++; //used for debugging
            delay_ms(100);
            return false;
        }
    }

    void nordic_sender(void) {
    	char hops = 0;
    	char ir_addr = 100;
    	char bt_addr = 200;
    	mesh_packet_t pkt;

    	char var1 = 'm';
    	char var2 = 'a';
    	char var3 = 'i';
    	char var4 = 'l';
    	char var5 = '!';
    	wireless_form_pkt(&pkt, bt_addr, mesh_pkt_nack, hops,
    			5,
    			&var1, sizeof(var1),
    			&var2, sizeof(var2),
    			&var3, sizeof(var3),
    			&var4, sizeof(var4),
    			&var5, sizeof(var5));
    	wireless_send_formed_pkt(&pkt);
    	printf("Sent %c%c%c%c%c\n",var1,var2,var3,var4,var5);
    	delay_ms(500);
    }
};



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

	//sending mail status over nordic wireless
	scheduler_add_task(new gpio_task(PRIORITY_HIGH));

	//for receiver to send through bluetooth
//	u2_init();
//	while(1){
//		if(nordic_receiver()){
//			printf("sent to phone.\n\n"); //Used to make sure phone receives signal from UART/Bluetooth.
//		}
//	}

    /* Consumes very little CPU, but need highest priority to handle mesh network ACKs */
    scheduler_add_task(new wirelessTask(PRIORITY_CRITICAL));

    /* Change "#if 0" to "#if 1" to run period tasks; @see period_callbacks.cpp */
    #if 0
    scheduler_add_task(new periodicSchedulerTask());
    #endif

    /* The task for the IR receiver */
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

    scheduler_start(); ///< This shouldn't return
    return -1;
}
