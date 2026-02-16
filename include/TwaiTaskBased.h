#ifndef TWAI_TASK_BASED_WROOM32_HEADER_H
#define TWAI_TASK_BASED_WROOM32_HEADER_H

#pragma once
#include <driver/twai.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <Arduino.h>

/**
 * @class TwaiTaskBased
 * @brief FreeRTOS task-based TWAI (CAN) communication wrapper for ESP32-WROOM-32
 *
 * This library wraps the ESP32 TWAI driver with FreeRTOS tasks to provide
 * asynchronous, non-blocking CAN bus communication. It uses callback functions
 * to notify the application of received messages and transmission results.
 *
 * Usage:
 *   TwaiTaskBased::onReceive(myRxCallback);
 *   TwaiTaskBased::onTransmit(myTxCallback);
 *   TwaiTaskBased::begin(GPIO_NUM_15, GPIO_NUM_13, 500000);  // TX=15, RX=13, 500kbps
 *   // CAN communication now active in background
 *   TwaiTaskBased::send(canMessage);  // Send a message
 */
class TwaiTaskBased {
public:
  // Callback function types
  using RxCallback = void (*)(const twai_message_t &msg);
  using TxCallback = void (*)(bool success);

  /**
   * Initialize the TWAI (CAN) interface
   * @param txPin GPIO pin for CAN TX
   * @param rxPin GPIO pin for CAN RX
   * @param baudrate CAN bus speed (125000, 250000, 500000, 1000000)
   * @param mode TWAI operating mode (TWAI_MODE_NORMAL, TWAI_MODE_NO_ACK, TWAI_MODE_LISTEN_ONLY)
   * @param rxTaskStack RX task stack size in bytes (default 4096)
   * @param rxTaskPrio RX task priority (default 10)
   * @param txTaskPrio TX task priority (default 9)
   * @return true if initialization successful, false otherwise
   */
  static bool begin(gpio_num_t txPin, gpio_num_t rxPin,
                    uint32_t baudrate = 500000,
                    twai_mode_t mode = TWAI_MODE_NORMAL,
                    uint32_t rxTaskStack = 4096,
                    UBaseType_t rxTaskPrio = 10,
                    UBaseType_t txTaskPrio = 9);

  /**
   * Register a callback function for CAN message reception
   * @param cb Callback function to be called when a message is received
   */
  static void onReceive(RxCallback cb);

  /**
   * Register a callback function for CAN message transmission
   * @param cb Callback function to be called when a message transmission completes
   */
  static void onTransmit(TxCallback cb);

  /**
   * Send a CAN message on the bus
   * @param msg The TWAI message to send
   * @param timeout Maximum time to wait for queue space (portMAX_DELAY for infinite)
   * @return true if message was queued successfully, false otherwise
   */
  static bool send(const twai_message_t &msg,
                   TickType_t timeout = portMAX_DELAY);

private:
  // Initialization state flag
  static bool _started;

  // FreeRTOS task handles
  static TaskHandle_t _rxTaskHandle;
  static TaskHandle_t _txTaskHandle;

  // FreeRTOS message queue for TX messages
  static QueueHandle_t _txQueue;

  // Callback function pointers
  static RxCallback _rxCallback;
  static TxCallback _txCallback;

  // Private task functions
  static void rxTask(void *pvParameters);
  static void txTask(void *pvParameters);

  // Helper function to get timing configuration from baudrate
  static twai_timing_config_t timingFromBaudrate(uint32_t baudrate);
};

#endif  // TWAI_TASK_BASED_WROOM32_HEADER_H
