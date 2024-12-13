#include <Arduino.h>
#include <freertos/FreeRTOS.h>

// Simple arduino code to test NPN-based proximity sensor

// Define the pin for the sensor
#define SENSOR_PIN GPIO_NUM_15

void sensor_proximity_init() {
  // Set the sensor pin as input
  pinMode(SENSOR_PIN, GPIO_MODE_INPUT);
  Serial.println("Sensor initialized");

  // Initialize the onboard LED state
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, digitalRead(SENSOR_PIN));
  Serial.println("LED initialized");
}

void task_sensor_proximity(void *pvParameters) {
  

  Serial.println("Initializing sensor");
  sensor_proximity_init();
  Serial.println("Sensor initialized");

  // Do nothing
  while (1) {
    // Read the sensor value
    int sensorValue = digitalRead(SENSOR_PIN);
    Serial.println(sensorValue);
    // Turn on the LED if the sensor is active
    digitalWrite(LED_BUILTIN, sensorValue);
    // Wait for 100ms
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}



void setup() {
  // Start serial communication
  Serial.begin(115200);
  Serial.println("Proximity sensor test");

  // initialize freeRTOS tasks
  xTaskCreate(task_sensor_proximity, "Proximity Sensor", 4096, NULL, 1, NULL);
  
}

void loop() {
  // Do nothing
  delay(1000);
}
 
