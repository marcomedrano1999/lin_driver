/*
 * lin_driver_test_main.c
 * Created on: Sep 15, 2018
 *     Author: Nico
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "fsl_uart_freertos.h"
#include "fsl_uart.h"

#include "pin_mux.h"
#include "clock_config.h"
#include <lin1d3_driver.h>
#include "FreeRTOSConfig.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define xJUST_MASTER

/* UART instance and clock */
#define MASTER_UART UART3
#define MASTER_UART_CLKSRC UART3_CLK_SRC
#define MASTER_UART_CLK_FREQ CLOCK_GetFreq(UART3_CLK_SRC)
#define MASTER_UART_RX_TX_IRQn UART3_RX_TX_IRQn

/* UART instance and clock */
#define LOCAL_SLAVE_UART UART3
#define LOCAL_SLAVE_UART_CLKSRC UART3_CLK_SRC
#define LOCAL_SLAVE_UART_CLK_FREQ CLOCK_GetFreq(UART3_CLK_SRC)
#define LOCAL_SLAVE_UART_RX_TX_IRQn UART3_RX_TX_IRQn

/* UART instance and clock */
#define SLAVE_UART UART4
#define SLAVE_UART_CLKSRC UART4_CLK_SRC
#define SLAVE_UART_CLK_FREQ CLOCK_GetFreq(UART4_CLK_SRC)
#define SLAVE_UART_RX_TX_IRQn UART4_RX_TX_IRQn

/* Task priorities. */
#define init_task_PRIORITY (configMAX_PRIORITIES - 2)
#define test_task_heap_size_d	(192)

#define Node_A_task_PRIORITY (configMAX_PRIORITIES - 2)
#define Node_B_task_PRIORITY (configMAX_PRIORITIES - 2)
#define Node_C_task_PRIORITY (configMAX_PRIORITIES - 2)
#define Node_D_task_PRIORITY (configMAX_PRIORITIES - 2)

// Macro to swap bits in the ID
#define SWAP_ID(x)		((x >> 3) | ((x >> 1) & 2) | ((x << 1) & 4) | ((x << 3) & 8))

#define message_id_1_d (SWAP_ID(0x01)<<2|message_size_2_bytes_d)
#define message_id_2_d (SWAP_ID(0x02)<<2|message_size_2_bytes_d)
#define message_id_3_d (SWAP_ID(0x03)<<2|message_size_2_bytes_d)
#define message_id_4_d (SWAP_ID(0x04)<<2|message_size_2_bytes_d)

#define app_message_id_1_d (0x01<<2|message_size_2_bytes_d)
#define app_message_id_2_d (0x02<<2|message_size_4_bytes_d)
#define app_message_id_3_d (0x03<<2|message_size_8_bytes_d)
#define app_message_id_4_d (0x04<<2|message_size_2_bytes_d)
#define app_message_id_5_d (0x05<<2|message_size_4_bytes_d)
#define app_message_id_6_d (0x06<<2|message_size_8_bytes_d)


typedef enum
{
	LED_STATE_OFF,
	LED_STATE_RED,
	LED_STATE_GREEN,
	LED_STATE_BLUE
}LED_State_t;



/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void test_task(void *pvParameters);
static void Node_A_task(void *pvParameters);
static void Node_B_task(void *pvParameters);
static void Node_C_task(void *pvParameters);
static void Node_D_task(void *pvParameters);

static void	message_4_callback_local_slave(void* message);
static void	message_5_callback_local_slave(void* message);
static void	message_6_callback_local_slave(void* message);
static void	message_1_callback_local_slave(void* message);

static void	message_1_callback_slave(void* message);
static void	message_2_callback_slave(void* message);
static void	message_3_callback_slave(void* message);


static void Node_A_message_1_callback_slave(void* message);
static void Node_A_message_2_callback_slave(void* message);
static void Node_A_message_3_callback_slave(void* message);
static void Node_A_message_4_callback_slave(void* message);

static void Node_BCD_message_1_callback_slave(void* message);
static void Node_BCD_message_234_callback_slave(void* message);


/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Application entry point.
 */
int main(void)
{
	/* Define the init structure for the output LED pin*/
	gpio_pin_config_t led_config = {
		kGPIO_DigitalOutput,
		0,
	};

	/* Define the init structure for the input button pin */
	gpio_pin_config_t button_config = {
		kGPIO_DigitalInput,
		0,
	};

    /* Init board hardware. */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    NVIC_SetPriority(MASTER_UART_RX_TX_IRQn, 5);
    NVIC_SetPriority(SLAVE_UART_RX_TX_IRQn, 5);

    // Init LEDs
    GPIO_PinInit(BOARD_LED_RED_GPIO, BOARD_LED_RED_GPIO_PIN, &led_config);
    GPIO_PinInit(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_GPIO_PIN, &led_config);
    GPIO_PinInit(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_GPIO_PIN, &led_config);

    // Turn off all leds
	GPIO_PortSet(BOARD_LED_RED_GPIO, 1u << BOARD_LED_RED_GPIO_PIN);
	GPIO_PortSet(BOARD_LED_BLUE_GPIO, 1u << BOARD_LED_BLUE_GPIO_PIN);
	GPIO_PortSet(BOARD_LED_GREEN_GPIO, 1u << BOARD_LED_GREEN_GPIO_PIN);

    // Init buttons
    GPIO_PinInit(BOARD_SW2_GPIO, BOARD_SW2_GPIO_PIN, &button_config);
    GPIO_PinInit(BOARD_SW3_GPIO, BOARD_SW3_GPIO_PIN, &button_config);

    /*if (xTaskCreate(test_task, "test_task", test_task_heap_size_d, NULL, init_task_PRIORITY, NULL) != pdPASS)
    {
        PRINTF("Init Task creation failed!.\r\n");
        while (1)
            ;
    }*/

    // Node A
    if (xTaskCreate(Node_A_task, "Node_A_task", test_task_heap_size_d, NULL, init_task_PRIORITY, NULL) != pdPASS)
	{
		PRINTF("Node A Task creation failed!.\r\n");
		while (1)
			;
	}

    // Node B
    /*if (xTaskCreate(Node_B_task, "Node_B_task", test_task_heap_size_d, NULL, init_task_PRIORITY, NULL) != pdPASS)
	{
		PRINTF("Node B Task creation failed!.\r\n");
		while (1)
			;
	}*/

    // Node C
    /*if (xTaskCreate(Node_C_task, "Node_C_task", test_task_heap_size_d, NULL, init_task_PRIORITY, NULL) != pdPASS)
	{
		PRINTF("Node C Task creation failed!.\r\n");
		while (1)
			;
	}*/

    // Node D
    /*if (xTaskCreate(Node_D_task, "Node_D_task", test_task_heap_size_d, NULL, init_task_PRIORITY, NULL) != pdPASS)
	{
		PRINTF("Node D Task creation failed!.\r\n");
		while (1)
			;
	}*/


    PRINTF(" *** LIN driver demo ***\r\n");
    vTaskStartScheduler();
    for (;;)
        ;
}

/*!
 * @brief Task responsible for loopback.
 */
static void test_task(void *pvParameters)
{
	int error;
	lin1d3_nodeConfig_t node_config;
	lin1d3_handle_t* master_handle;
	lin1d3_handle_t* slave_handle;
	lin1d3_handle_t* local_slave_handle;
	/* Set Master Config */
	node_config.type = lin1d3_master_nodeType;
	node_config.bitrate = 9600;
	node_config.uartBase = MASTER_UART;
	node_config.srcclk = MASTER_UART_CLK_FREQ;
	node_config.skip_uart_init = 0;
	//memset(node_config.messageTable,0, (sizeof(node_config.messageTable[0])*lin1d3_max_supported_messages_per_node_cfg_d));
	/* Init Master node */
	master_handle = lin1d3_InitNode(node_config);
#if !defined(JUST_MASTER)
	/* Set Slave Config */
	node_config.type = lin1d3_slave_nodeType;
	node_config.bitrate = 9600;
	node_config.uartBase = SLAVE_UART;
	node_config.srcclk = SLAVE_UART_CLK_FREQ;
	node_config.skip_uart_init = 0;
	node_config.messageTableSize = 3;
	node_config.messageTable = pvPortMalloc(sizeof(lin1d3_messageConfig_t) * 3);
	memset(node_config.messageTable,0, (sizeof(node_config.messageTable[0])*3));
	node_config.messageTable[0].ID = app_message_id_1_d;
	node_config.messageTable[0].rx = 0;
	node_config.messageTable[0].handler = message_1_callback_slave;
	node_config.messageTable[1].ID = app_message_id_2_d;
	node_config.messageTable[1].rx = 0;
	node_config.messageTable[1].handler = message_2_callback_slave;
	node_config.messageTable[2].ID = app_message_id_3_d;
	node_config.messageTable[2].rx = 0;
	node_config.messageTable[2].handler = message_3_callback_slave;
	/* Init Slave Node*/
	slave_handle = lin1d3_InitNode(node_config);

	/* Set local Slave Config */
	node_config.type = lin1d3_slave_nodeType;
	node_config.bitrate = 9600;
	node_config.uartBase = LOCAL_SLAVE_UART;
	node_config.srcclk = LOCAL_SLAVE_UART_CLK_FREQ;
	node_config.messageTableSize = 4;
	node_config.messageTable = pvPortMalloc(sizeof(lin1d3_messageConfig_t) * 4);
	memset(node_config.messageTable,0, (sizeof(node_config.messageTable[0])*4));
	node_config.messageTable[0].ID = app_message_id_4_d;
	node_config.messageTable[0].rx = 0;
	node_config.messageTable[0].handler = message_4_callback_local_slave;
	node_config.messageTable[1].ID = app_message_id_5_d;
	node_config.messageTable[1].rx = 0;
	node_config.messageTable[1].handler = message_5_callback_local_slave;
	node_config.messageTable[2].ID = app_message_id_6_d;
	node_config.messageTable[2].rx = 0;
	node_config.messageTable[2].handler = message_6_callback_local_slave;
	node_config.messageTable[3].ID = app_message_id_1_d;
	node_config.messageTable[3].rx = 1;
	node_config.messageTable[3].handler = message_1_callback_local_slave;
	node_config.skip_uart_init = 1;
	node_config.uart_rtos_handle = master_handle->uart_rtos_handle;
	/* Init local Slave Node*/
	local_slave_handle = lin1d3_InitNode(node_config);
#endif

	if((NULL == master_handle)
#if !defined(JUST_MASTER)
		|| (NULL == slave_handle)
		/*|| (NULL == local_slave_handle)*/
#endif
	   ){
		PRINTF(" Init failed!! \r\n");
		error = kStatus_Fail;
	}
	else {
		error = kStatus_Success;
	}

	while (kStatus_Success == error)
    {
    	vTaskDelay(200);
    	lin1d3_masterSendMessage(master_handle, app_message_id_1_d);
    	vTaskDelay(200);
    	lin1d3_masterSendMessage(master_handle, app_message_id_2_d);
    	vTaskDelay(200);
    	lin1d3_masterSendMessage(master_handle, app_message_id_3_d);
    	vTaskDelay(200);
    	lin1d3_masterSendMessage(master_handle, app_message_id_4_d);
    	vTaskDelay(200);
    	lin1d3_masterSendMessage(master_handle, app_message_id_5_d);
    	vTaskDelay(200);
    	lin1d3_masterSendMessage(master_handle, app_message_id_6_d);
    }

    vTaskSuspend(NULL);
}

/*!
 * @brief This task initializes the master
 */
static void Node_A_task(void *pvParameters)
{
	int error;
	lin1d3_nodeConfig_t node_config;
	lin1d3_handle_t* master_handle;
	lin1d3_handle_t* local_slave_handle;
	/* Set Master Config */
	node_config.type = lin1d3_master_nodeType;
	node_config.bitrate = 9600;
	node_config.uartBase = MASTER_UART;
	node_config.srcclk = MASTER_UART_CLK_FREQ;
	node_config.skip_uart_init = 0;
	/* Init Master node */
	master_handle = lin1d3_InitNode(node_config);


	/* Set local Slave Config */
	node_config.type = lin1d3_slave_nodeType;
	node_config.bitrate = 9600;
	node_config.uartBase = LOCAL_SLAVE_UART;
	node_config.srcclk = LOCAL_SLAVE_UART_CLK_FREQ;
	node_config.messageTableSize = 4;
	node_config.messageTable = pvPortMalloc(sizeof(lin1d3_messageConfig_t) * 4);
	memset(node_config.messageTable,0, (sizeof(node_config.messageTable[0])*4));
	node_config.messageTable[0].ID = message_id_1_d;
	node_config.messageTable[0].rx = 0;
	node_config.messageTable[0].handler = Node_A_message_1_callback_slave;
	node_config.messageTable[1].ID = message_id_2_d;
	node_config.messageTable[1].rx = 1;
	node_config.messageTable[1].handler = Node_A_message_2_callback_slave;
	node_config.messageTable[2].ID = message_id_3_d;
	node_config.messageTable[2].rx = 1;
	node_config.messageTable[2].handler = Node_A_message_3_callback_slave;
	node_config.messageTable[3].ID = message_id_4_d;
	node_config.messageTable[3].rx = 1;
	node_config.messageTable[3].handler = Node_A_message_4_callback_slave;
	node_config.skip_uart_init = 1;
	node_config.uart_rtos_handle = master_handle->uart_rtos_handle;
	/* Init local Slave Node*/
	local_slave_handle = lin1d3_InitNode(node_config);


	if((NULL == master_handle) || (NULL == local_slave_handle))
	{
		PRINTF(" Init failed!! \r\n");
		error = kStatus_Fail;
	}
	else {
		error = kStatus_Success;
	}

	volatile uint8_t task_ID_1_counter = 0;
	while (kStatus_Success == error)
	{
		vTaskDelay(25/portTICK_PERIOD_MS);
		// Send messages that are required every 100 ms
		lin1d3_masterSendMessage(master_handle, message_id_2_d);

		vTaskDelay(25/portTICK_PERIOD_MS);
		lin1d3_masterSendMessage(master_handle, message_id_3_d);

		vTaskDelay(25/portTICK_PERIOD_MS);
		lin1d3_masterSendMessage(master_handle, message_id_4_d);

		vTaskDelay(25/portTICK_PERIOD_MS);
		if(task_ID_1_counter >= 10)
		{
			// Send message that is required every second
			lin1d3_masterSendMessage(master_handle, message_id_1_d);
			task_ID_1_counter=0;
		}
		else
		{
			task_ID_1_counter++;
		}

	}

	vTaskSuspend(NULL);
}




/*!
 * @brief This task contains a LIN Slave
 */
static void Node_B_task(void *pvParameters)
{
	lin1d3_nodeConfig_t node_config;
	lin1d3_handle_t* slave_handle;

	/* Set Slave Config */
	node_config.type = lin1d3_slave_nodeType;
	node_config.bitrate = 9600;
	node_config.uartBase = SLAVE_UART;
	node_config.srcclk = SLAVE_UART_CLK_FREQ;
	node_config.skip_uart_init = 0;
	node_config.messageTableSize = 2;
	node_config.messageTable = pvPortMalloc(sizeof(lin1d3_messageConfig_t) * 2);
	memset(node_config.messageTable,0, (sizeof(node_config.messageTable[0])*2));
	node_config.messageTable[0].ID = message_id_1_d;
	node_config.messageTable[0].rx = 1;
	node_config.messageTable[0].handler = Node_BCD_message_1_callback_slave;
	node_config.messageTable[1].ID = message_id_2_d;
	node_config.messageTable[1].rx = 0;
	node_config.messageTable[1].handler = Node_BCD_message_234_callback_slave;
	/* Init Slave Node*/
	slave_handle = lin1d3_InitNode(node_config);

	if(NULL == slave_handle){
		PRINTF(" Init failed!! \r\n");
	}

	vTaskSuspend(NULL);
}


/*!
 * @brief This task contains a LIN Slave
 */
static void Node_C_task(void *pvParameters)
{
	lin1d3_nodeConfig_t node_config;
	lin1d3_handle_t* slave_handle;

	/* Set Slave Config */
	node_config.type = lin1d3_slave_nodeType;
	node_config.bitrate = 9600;
	node_config.uartBase = SLAVE_UART;
	node_config.srcclk = SLAVE_UART_CLK_FREQ;
	node_config.skip_uart_init = 0;
	node_config.messageTableSize = 2;
	node_config.messageTable = pvPortMalloc(sizeof(lin1d3_messageConfig_t) * 2);
	memset(node_config.messageTable,0, (sizeof(node_config.messageTable[0])*2));
	node_config.messageTable[0].ID = message_id_1_d;
	node_config.messageTable[0].rx = 1;
	node_config.messageTable[0].handler = Node_BCD_message_1_callback_slave;
	node_config.messageTable[1].ID = message_id_3_d;
	node_config.messageTable[1].rx = 0;
	node_config.messageTable[1].handler = Node_BCD_message_234_callback_slave;
	/* Init Slave Node*/
	slave_handle = lin1d3_InitNode(node_config);

	if(NULL == slave_handle){
		PRINTF(" Init failed!! \r\n");
	}

	vTaskSuspend(NULL);
}

/*!
 * @brief This task contains a LIN Slave
 */
static void Node_D_task(void *pvParameters)
{
	lin1d3_nodeConfig_t node_config;
	lin1d3_handle_t* slave_handle;

	/* Set Slave Config */
	node_config.type = lin1d3_slave_nodeType;
	node_config.bitrate = 9600;
	node_config.uartBase = SLAVE_UART;
	node_config.srcclk = SLAVE_UART_CLK_FREQ;
	node_config.skip_uart_init = 0;
	node_config.messageTableSize = 2;
	node_config.messageTable = pvPortMalloc(sizeof(lin1d3_messageConfig_t) * 2);
	memset(node_config.messageTable,0, (sizeof(node_config.messageTable[0])*2));
	node_config.messageTable[0].ID = message_id_1_d;
	node_config.messageTable[0].rx = 1;
	node_config.messageTable[0].handler = Node_BCD_message_1_callback_slave;
	node_config.messageTable[1].ID = message_id_4_d;
	node_config.messageTable[1].rx = 0;
	node_config.messageTable[1].handler = Node_BCD_message_234_callback_slave;
	/* Init Slave Node*/
	slave_handle = lin1d3_InitNode(node_config);

	if(NULL == slave_handle){
		PRINTF(" Init failed!! \r\n");
	}

	vTaskSuspend(NULL);
}

/*
 * Node has to change the state value in every call
 */
static void Node_A_message_1_callback_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	static LED_State_t led_state = LED_STATE_OFF;

	PRINTF("Local Slave got message 1 request\r\n");

	// Fill data with current state
	switch(led_state)
	{
	case LED_STATE_RED:
		message_data[0]=1;
		led_state=LED_STATE_GREEN;
		break;
	case LED_STATE_GREEN:
		message_data[0]=2;
		led_state=LED_STATE_BLUE;
		break;
	case LED_STATE_BLUE:
		message_data[0]=3;
		led_state=LED_STATE_OFF;
		break;
	default:
		message_data[0]=0;
		led_state=LED_STATE_RED;
		break;
	}

}

/*
 * Node has to control RED led depending on the response
 */
static void Node_A_message_2_callback_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Local Slave got response to message 2 %d,%d\r\n", message_data[0], message_data[1]);
	if(message_data[0] == 1 || message_data[1] == 1)
	{
		// Turn on RED led if at least one button is pressed
		GPIO_PortClear(BOARD_LED_RED_GPIO, 1u << BOARD_LED_RED_GPIO_PIN);
	}
	else
	{
		// Turn off RED led otherwise
		GPIO_PortSet(BOARD_LED_RED_GPIO, 1u << BOARD_LED_RED_GPIO_PIN);
	}
}

/*
 * Node has to control BLUE led depending on the response
 */
static void Node_A_message_3_callback_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Local Slave got response to message 3 %d,%d\r\n", message_data[0], message_data[1]);
	if(message_data[0] == 1 || message_data[1] == 1)
	{
		// Turn on BLUE led if at least one button is pressed
		GPIO_PortClear(BOARD_LED_BLUE_GPIO, 1u << BOARD_LED_BLUE_GPIO_PIN);
	}
	else
	{
		// Turn off BLUE led otherwise
		GPIO_PortSet(BOARD_LED_BLUE_GPIO, 1u << BOARD_LED_BLUE_GPIO_PIN);
	}
}

/*
 * Node has to control GREEN led depending on the response
 */
static void Node_A_message_4_callback_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Local Slave got response to message 4 %d,%d\r\n", message_data[0], message_data[1]);
	if(message_data[0] == 1 || message_data[1] == 1)
	{
		// Turn on GREEN led if at least one button is pressed
		GPIO_PortClear(BOARD_LED_GREEN_GPIO, 1u << BOARD_LED_GREEN_GPIO_PIN);
	}
	else
	{
		// Turn off GREEN led otherwise
		GPIO_PortSet(BOARD_LED_GREEN_GPIO, 1u << BOARD_LED_GREEN_GPIO_PIN);
	}
}

/*
 * Node has to change LED based on response
 */
static void Node_BCD_message_1_callback_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Slave got response to message 1 %d,%d\r\n", message_data[0], message_data[1]);

	// Turn off all leds
	GPIO_PortSet(BOARD_LED_RED_GPIO, 1u << BOARD_LED_RED_GPIO_PIN);
	GPIO_PortSet(BOARD_LED_BLUE_GPIO, 1u << BOARD_LED_BLUE_GPIO_PIN);
	GPIO_PortSet(BOARD_LED_GREEN_GPIO, 1u << BOARD_LED_GREEN_GPIO_PIN);

	// Turn led based on input code
	switch(message_data[0])
	{
	case LED_STATE_RED:
		GPIO_PortClear(BOARD_LED_RED_GPIO, 1u << BOARD_LED_RED_GPIO_PIN);
		break;
	case LED_STATE_GREEN:
		GPIO_PortClear(BOARD_LED_GREEN_GPIO, 1u << BOARD_LED_GREEN_GPIO_PIN);
		break;
	case LED_STATE_BLUE:
		GPIO_PortClear(BOARD_LED_BLUE_GPIO, 1u << BOARD_LED_BLUE_GPIO_PIN);
		break;
	default: // LED STATE OFF
		break;
	}

}

/*
 * Node has to return SW2.B and SW3.B states
 */
static void Node_BCD_message_234_callback_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Slave got message 2 request\r\n");

	// Fill message with SW2.B and SW3.B states
	message_data[0] = ((GPIO_PinRead(BOARD_SW2_GPIO, BOARD_SW2_GPIO_PIN)==0?1:0) & 0x1);
	message_data[1] = ((GPIO_PinRead(BOARD_SW3_GPIO, BOARD_SW3_GPIO_PIN)==0?1:0) & 0x1);

}




static void	message_4_callback_local_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Local Slave got message 4 request\r\n");
	message_data[0] = 1;
	message_data[1] = 2;
}

static void	message_5_callback_local_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Local Slave got message 5 request\r\n");
	message_data[0] = 1;
	message_data[1] = 2;
	message_data[2] = 3;
	message_data[3] = 4;
}

static void	message_6_callback_local_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Local Slave got message 6 request\r\n");
	message_data[0] = 1;
	message_data[1] = 2;
	message_data[2] = 3;
	message_data[3] = 4;
	message_data[4] = 5;
	message_data[5] = 6;
	message_data[6] = 7;
	message_data[7] = 8;
}

static void	message_1_callback_local_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Local Slave got response to message 1 %d,%d\r\n", message_data[0], message_data[1]);
}

static void	message_1_callback_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Slave got message 1 request\r\n");
	message_data[0] = 79;
	message_data[1] = 80;
}

static void	message_2_callback_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Slave got message 2 request\r\n");
	message_data[0] = 79;
	message_data[1] = 80;
	message_data[2] = 81;
	message_data[3] = 82;
}

static void	message_3_callback_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Slave got message 3 request\r\n");
	message_data[0] = 79;
	message_data[1] = 80;
	message_data[2] = 81;
	message_data[3] = 82;
	message_data[4] = 83;
	message_data[5] = 84;
	message_data[6] = 85;
	message_data[7] = 86;
}

