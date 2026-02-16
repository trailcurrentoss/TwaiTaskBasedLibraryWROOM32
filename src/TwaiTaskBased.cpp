#include <TwaiTaskBased.h>

// Static member variable initialization
bool TwaiTaskBased::_started = false;
TaskHandle_t TwaiTaskBased::_rxTaskHandle = nullptr;
TaskHandle_t TwaiTaskBased::_txTaskHandle = nullptr;
QueueHandle_t TwaiTaskBased::_txQueue = nullptr;

TwaiTaskBased::RxCallback TwaiTaskBased::_rxCallback = nullptr;
TwaiTaskBased::TxCallback TwaiTaskBased::_txCallback = nullptr;

bool TwaiTaskBased::begin(gpio_num_t txPin, gpio_num_t rxPin,
                          uint32_t baudrate,
                          twai_mode_t mode,
                          uint32_t rxTaskStack,
                          UBaseType_t rxTaskPrio,
                          UBaseType_t txTaskPrio) {
  // Prevent multiple initializations
  if (_started)
    return false;

  // Configure TWAI general settings
  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(txPin, rxPin, mode);

  // Get timing configuration based on requested baudrate
  twai_timing_config_t t_config = timingFromBaudrate(baudrate);

  // Configure to accept all messages (filtering done in application)
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install the TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("TwaiTaskBased: Failed to install TWAI driver");
    return false;
  }

  // Start the TWAI driver
  if (twai_start() != ESP_OK) {
    Serial.println("TwaiTaskBased: Failed to start TWAI driver");
    twai_driver_uninstall();
    return false;
  }

  // Create the TX message queue (16 messages deep)
  _txQueue = xQueueCreate(16, sizeof(twai_message_t));
  if (!_txQueue) {
    Serial.println("TwaiTaskBased: Failed to create TX queue");
    twai_stop();
    twai_driver_uninstall();
    return false;
  }

  // Create RX task pinned to Core 0
  BaseType_t rxTaskResult = xTaskCreatePinnedToCore(
      rxTask,                        // Task function
      "twai_rx",                     // Task name for debugging
      rxTaskStack,                   // Stack size
      nullptr,                       // Task parameters
      rxTaskPrio,                    // Task priority
      &_rxTaskHandle,                // Task handle output
      0                              // Core 0
  );

  if (rxTaskResult != pdPASS) {
    Serial.println("TwaiTaskBased: Failed to create RX task");
    vQueueDelete(_txQueue);
    twai_stop();
    twai_driver_uninstall();
    _txQueue = nullptr;
    return false;
  }

  // Create TX task pinned to Core 0
  BaseType_t txTaskResult = xTaskCreatePinnedToCore(
      txTask,                        // Task function
      "twai_tx",                     // Task name for debugging
      4096,                          // Stack size for TX task
      nullptr,                       // Task parameters
      txTaskPrio,                    // Task priority
      &_txTaskHandle,                // Task handle output
      0                              // Core 0
  );

  if (txTaskResult != pdPASS) {
    Serial.println("TwaiTaskBased: Failed to create TX task");
    vTaskDelete(_rxTaskHandle);
    vQueueDelete(_txQueue);
    twai_stop();
    twai_driver_uninstall();
    _rxTaskHandle = nullptr;
    _txQueue = nullptr;
    return false;
  }

  _started = true;
  Serial.println("TwaiTaskBased: Initialized successfully");
  return true;
}

void TwaiTaskBased::onReceive(RxCallback cb) {
  _rxCallback = cb;
}

void TwaiTaskBased::onTransmit(TxCallback cb) {
  _txCallback = cb;
}

bool TwaiTaskBased::send(const twai_message_t &msg, TickType_t timeout) {
  if (!_started) {
    Serial.println("TwaiTaskBased: Not initialized");
    return false;
  }

  // Queue the message for transmission
  return xQueueSend(_txQueue, &msg, timeout) == pdTRUE;
}

void TwaiTaskBased::rxTask(void *pvParameters) {
  twai_message_t msg;

  // Continuous receive loop
  while (true) {
    // Wait for a message with infinite timeout
    esp_err_t res = twai_receive(&msg, portMAX_DELAY);

    if (res == ESP_OK && _rxCallback != nullptr) {
      // Call the registered callback with the received message
      _rxCallback(msg);
    }
  }

  // Task cleanup (unreachable in normal operation)
  vTaskDelete(nullptr);
}

void TwaiTaskBased::txTask(void *pvParameters) {
  twai_message_t msg;

  // Continuous transmission loop
  while (true) {
    // Wait for a message from the queue
    if (xQueueReceive(_txQueue, &msg, portMAX_DELAY) == pdTRUE) {
      // Attempt to transmit the message
      esp_err_t res = twai_transmit(&msg, portMAX_DELAY);

      if (_txCallback != nullptr) {
        // Call the registered callback with transmission result
        _txCallback(res == ESP_OK);
      }
    }
  }

  // Task cleanup (unreachable in normal operation)
  vTaskDelete(nullptr);
}

twai_timing_config_t TwaiTaskBased::timingFromBaudrate(uint32_t baudrate) {
  switch (baudrate) {
    case 125000:
      return TWAI_TIMING_CONFIG_125KBITS();
    case 250000:
      return TWAI_TIMING_CONFIG_250KBITS();
    case 500000:
      return TWAI_TIMING_CONFIG_500KBITS();
    case 1000000:
      return TWAI_TIMING_CONFIG_1MBITS();
    default:
      // Default to 500 kbps if invalid baudrate provided
      return TWAI_TIMING_CONFIG_500KBITS();
  }
}
