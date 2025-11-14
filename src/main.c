/*
 * State Machine: IDLE → DETECTING → SYMBOL_READY → SENDING/RECEIVING → DISPLAYING
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include <pico/stdlib.h>

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <semphr.h>

#include "projdefs.h"
#include "tkjhat/sdk.h"

/* ============================================================================
 * CONFIGURATION & CONSTANTS
 * ============================================================================ */

// IMU Detection Parameters
#define IMU_SAMPLE_COUNT 10          // Number of samples to average
#define IMU_SAMPLE_DELAY_MS 50       // Delay between samples (ms)
#define DOT_THRESHOLD 900            // Z-axis threshold for dot (flat position)
#define DASH_THRESHOLD 900           // X-axis threshold for dash (90° tilt)

// Communication Parameters
#define QUEUE_LENGTH 64              // Message queue size
#define QUEUE_ITEM_SIZE sizeof(char) // Size of each queue item

// Task Stack Sizes
#define DEFAULT_STACK_SIZE 2048

/* ============================================================================
 * GLOBAL STATE & SYNCHRONIZATION
 * ============================================================================ */

// State Machine Definition
enum state {
    IDLE,           // Waiting for input
    DETECTING,      // Reading IMU sensors
    SYMBOL_READY,   // Symbol detected, ready to send
    SENDING,        // Transmitting message
    RECEIVING,      // Receiving message
    DISPLAYING      // Showing message on LCD
};

// Global State Variables
uint8_t currentState = IDLE;                    // Current state machine state
char symbolBuffer[64];                          // Buffer for building messages
uint32_t lastReading[3];                        // Last IMU reading (X, Y, Z)

// FreeRTOS Synchronization Objects
QueueHandle_t messageQueue;                     // Queue for inter-task messaging
SemaphoreHandle_t uartMutex;                    // Mutex for UART access

// Legacy global (TODO: refactor)
uint32_t ambientLight;

/* ============================================================================
 * MODULE: BUTTON INPUT HANDLERS
 * ============================================================================ */

/**
 * @brief GPIO interrupt handler for button presses
 * @param gpio GPIO pin that triggered the interrupt
 * @param eventMask Event mask (rising/falling edge)
 *
 * BTN1 (BUTTON1): Insert space character
 * BTN2 (BUTTON2): Send message (newline)
 */
static void btn_fxn(uint gpio, uint32_t eventMask) {
    char symbol;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (gpio == BUTTON1) {
        symbol = ' ';  // Space between symbols
        xQueueSendFromISR(messageQueue, &symbol, &xHigherPriorityTaskWoken);
    } else if (gpio == BUTTON2) {
        symbol = '\n';  // Send message (3 spaces in protocol)
        xQueueSendFromISR(messageQueue, &symbol, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ============================================================================
 * MODULE: SENSOR TASK (IMU Detection & Morse Generation)
 * ============================================================================ */

/**
 * @brief Sensor task 
 *
 * Detection Logic:
 * - Samples IMU 10 times @ 50ms intervals
 * - Averages accelerometer values
 * - Flat position (Z > 900): Dot
 * - Tilted 90° (X > 900): Dash
 *
 * State Flow: IDLE → DETECTING → SYMBOL_READY → (queue symbol) → IDLE
 */
static void sensorTask(void *arg) {
    (void)arg;
    char detected_symbol;

    // Initialize ICM42670 sensor
    printf("[SENSOR] Initializing ICM42670...\n");
    if (init_ICM42670() == 0) {
        printf("[SENSOR] ICM-42670P initialized successfully!\n");
        if (ICM42670_start_with_default_values() != 0) {
            printf("[SENSOR] ERROR: Could not start accelerometer/gyroscope\n");
            vTaskDelete(NULL);
        }
        printf("[SENSOR] IMU ready\n");
    } else {
        printf("[SENSOR] ERROR: Failed to initialize ICM-42670P\n");
        vTaskDelete(NULL);
    }

    // Main sensor loop
    for(;;) {
        // DETECTING State: Sample IMU and detect symbols
        if (currentState == DETECTING) {
            printf("[SENSOR] DETECTING - reading sensors\n");

            long total_x = 0;
            long total_z = 0;
            int successful_reads = 0;

            // Collect IMU_SAMPLE_COUNT samples
            for (int i = 0; i < IMU_SAMPLE_COUNT; i++) {
                float accel_x, accel_y, accel_z;
                float gyro_x, gyro_y, gyro_z;
                float temp;

                vTaskDelay(pdMS_TO_TICKS(10));

                int res = ICM42670_read_sensor_data(&accel_x, &accel_y, &accel_z,
                                                     &gyro_x, &gyro_y, &gyro_z, &temp);

                if (res == 0) {
                    total_x += (long)accel_x;
                    total_z += (long)accel_z;
                    successful_reads++;

                    // Store last reading for moving average filter
                    lastReading[0] = (uint32_t)abs((int)accel_x);
                    lastReading[1] = (uint32_t)abs((int)accel_y);
                    lastReading[2] = (uint32_t)abs((int)accel_z);
                }

                vTaskDelay(pdMS_TO_TICKS(IMU_SAMPLE_DELAY_MS));
            }

            // Process collected data
            if (successful_reads > 0) {
                long avg_x = total_x / successful_reads;
                long avg_z = total_z / successful_reads;

                printf("[SENSOR] Avg: X=%ld, Z=%ld (n=%d)\n", avg_x, avg_z, successful_reads);

                // Threshold-based symbol detection
                if (labs(avg_z) > DOT_THRESHOLD) {
                    detected_symbol = '.';
                    printf("[SENSOR] *** DETECTED: DOT ***\n");
                    currentState = SYMBOL_READY;
                } else if (labs(avg_x) > DASH_THRESHOLD) {
                    detected_symbol = '-';
                    printf("[SENSOR] *** DETECTED: DASH ***\n");
                    currentState = SYMBOL_READY;
                } else {
                    printf("[SENSOR] No clear symbol\n");
                    currentState = IDLE;
                }
            } else {
                printf("[SENSOR] ERROR: No successful reads\n");
                currentState = IDLE;
            }
        }

        // IDLE State: Wait before next detection cycle
        if (currentState == IDLE) {
            vTaskDelay(pdMS_TO_TICKS(400));
            currentState = DETECTING;
        }
        // SYMBOL_READY State: Send detected symbol to queue
        else if (currentState == SYMBOL_READY) {
            printf("[SENSOR] SYMBOL_READY - sending '%c' to queue\n", detected_symbol);

            BaseType_t result = xQueueSend(messageQueue, &detected_symbol, pdMS_TO_TICKS(100));
            if (result == pdPASS) {
                printf("[SENSOR] Symbol queued successfully\n");
            } else {
                printf("[SENSOR] ERROR: Queue full\n");
            }
            currentState = IDLE;
        }
        // Safety: Handle unexpected states
        else if (currentState != DETECTING) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ============================================================================
 * MODULE: UART TASK (Serial Communication TX/RX)
 * ============================================================================ */

/**
 * @brief UART task 
 *
 * Transmit (TX):
 * - Receives symbols from messageQueue
 * - Sends via USB Serial at 115200 baud
 * - Format: ASCII '.', '-', ' ' (space)
 * - Message end: 3 spaces
 *
 * Receive (RX):
 * - TODO: Parse incoming messages
 * - TODO: Trigger displayTask via state machine
 */
static void uartTask(void *arg) {
    (void)arg;
    char received_symbol;

    printf("[UART] Task started\n");

    while (1) {
        // Wait for symbol from queue
        if (xQueueReceive(messageQueue, &received_symbol, portMAX_DELAY) == pdPASS) {
            // Acquire UART mutex for thread-safe transmission
            if (xSemaphoreTake(uartMutex, portMAX_DELAY) == pdTRUE) {
                // Transmit symbol via USB Serial
                printf("%c", received_symbol);
                fflush(stdout);

                // Release mutex
                xSemaphoreGive(uartMutex);

                printf("[UART] Transmitted: '%c'\n", received_symbol);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ============================================================================
 * MODULE: DISPLAY TASK (LCD, LED, Buzzer Feedback)
 * ============================================================================ */

/**
 * @brief Display task 
 *
 * - LCD: Show status messages and received Morse
 * - LED: Visual feedback for transmission/reception
 * - Buzzer: Audio patterns for send/receive success
 *
 * TODO: Implement LCD update logic
 * TODO: Add buzzer melody on message send/receive
 */
static void displayTask(void *arg) {
    (void)arg;

    printf("[DISPLAY] Task started\n");

    // TODO: Initialize LCD display
    // init_display();

    while (1) {
        // TODO: Update LCD with current state
        // TODO: Handle LED visual feedback
        // TODO: Play buzzer melodies on events

        // Placeholder: Blink LED on activity
        if (currentState != IDLE) {
            toggle_led();
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ============================================================================
 * MAIN: INITIALIZATION & TASK CREATION
 * ============================================================================ */

int main() {
    // Initialize USB Serial
    stdio_init_all();

    while (!stdio_usb_connected()) {
        sleep_ms(10);
    }

    printf("\n\n");
    printf("========================================\n");
    printf("  Secret Messages - Morse Communicator\n");
    printf("  Team: Axel, Rasmus, Miika\n");
    printf("========================================\n\n");

    // Initialize HAT SDK (I2C, GPIO, etc.)
    init_hat_sdk();
    sleep_ms(300);
    printf("[INIT] HAT SDK initialized\n");

    // Initialize GPIO: Buttons and LED
    gpio_init(BUTTON1);
    gpio_set_dir(BUTTON1, GPIO_IN);
    gpio_init(BUTTON2);
    gpio_set_dir(BUTTON2, GPIO_IN);
    gpio_init(LED1);
    gpio_set_dir(LED1, GPIO_OUT);

    // Register button interrupt handlers
    gpio_set_irq_enabled_with_callback(BUTTON1, GPIO_IRQ_EDGE_RISE, true, btn_fxn);
    gpio_set_irq_enabled_with_callback(BUTTON2, GPIO_IRQ_EDGE_RISE, true, btn_fxn);
    printf("[INIT] Buttons and LED initialized\n");

    // Create message queue for inter-task communication
    messageQueue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
    if (!messageQueue) {
        printf("[ERROR] Failed to create message queue\n");
        return -1;
    }
    printf("[INIT] Message queue created\n");

    // Create UART mutex for thread-safe serial access
    uartMutex = xSemaphoreCreateMutex();
    if (!uartMutex) {
        printf("[ERROR] Failed to create UART mutex\n");
        return -1;
    }
    printf("[INIT] UART mutex created\n");

    // Create FreeRTOS tasks
    TaskHandle_t hSensorTask, hUartTask, hDisplayTask;

    xTaskCreate(sensorTask, "sensorTask", DEFAULT_STACK_SIZE, NULL, 2, &hSensorTask);
    printf("[INIT] sensorTask created\n");

    xTaskCreate(uartTask, "uartTask", DEFAULT_STACK_SIZE, NULL, 3, &hUartTask);
    printf("[INIT] uartTask created (higher priority)\n");

    xTaskCreate(displayTask, "displayTask", DEFAULT_STACK_SIZE, NULL, 1, &hDisplayTask);
    printf("[INIT] displayTask created\n");

    printf("\n[INIT] Starting FreeRTOS scheduler...\n\n");

    // Start FreeRTOS scheduler (never returns)
    vTaskStartScheduler();

    // Should never reach here
    printf("[ERROR] Scheduler returned unexpectedly\n");
    return 0;
}
