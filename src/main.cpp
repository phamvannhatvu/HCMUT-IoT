#include <Arduino.h>
#include <DHT20.h>
#include <Server_Side_RPC.h>

#include "Thingsboard_Helper.h"

#define LED_PIN 48

constexpr char WIFI_SSID[] = "abcd";
constexpr char WIFI_PASSWORD[] = "123456789";



void initWiFi() {
  Serial.println("Connecting to AP ...");
  
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    // Delay 500ms until a connection has been successfully established
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

const bool reconnect() {
  // Check to ensure we aren't connected yet
  const wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    return true;
  }
  // If we aren't establish a new connection to the given WiFi network
  initWiFi();
  return true;
}



void TaskLEDControl(void *pvParameters) {
  pinMode(GPIO_NUM_48, OUTPUT); // Initialize LED pin
  int ledState = 0;
  while(1) {
    
    if (ledState == 0) {
      digitalWrite(GPIO_NUM_48, HIGH); // Turn ON LED
    } else {
      digitalWrite(GPIO_NUM_48, LOW); // Turn OFF LED
    }
    ledState = 1 - ledState;
    vTaskDelay(2000);
  }
}

void TaskTemperature_Humidity(void *pvParameters){
  DHT20 dht20;
  Wire.begin(GPIO_NUM_11, GPIO_NUM_12);
  dht20.begin();
  while(1){
    dht20.read();
    double temperature = dht20.getTemperature();
    double humidity = dht20.getHumidity();

    Serial.print("Temp: "); Serial.print(temperature); Serial.print(" *C ");
    Serial.print(" Humidity: "); Serial.print(humidity); Serial.print(" %");
    Serial.println();
    if(getThingsboardConnectedStatus() == true){
      Serial.println("Sending temperature data...");
      getThingsboardClient().sendTelemetryData(TEMPERATURE_KEY, temperature);

      Serial.println("Sending humidity data...");
      getThingsboardClient().sendTelemetryData(HUMIDITY_KEY, humidity);

    }

    vTaskDelay(5000);
  }

}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  initWiFi();
  initThingsBoard();
  
  xTaskCreate(TaskLEDControl, "LED Control", 2048, NULL, 2, NULL);
  xTaskCreate(TaskTemperature_Humidity, "Temperature Humidity", 2048, NULL, 2, NULL);
  
}

void loop() {
  
}