#define LED_PIN 48
#define SDA_PIN GPIO_NUM_11
#define SCL_PIN GPIO_NUM_12

#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include "DHT20.h"
#include "Wire.h"
#include <ArduinoOTA.h>

constexpr char WIFI_SSID[] = "nhatvu";
constexpr char WIFI_PASSWORD[] = "25122003";

constexpr char TOKEN[] = "tvj7ij6na9cfs2kki3cy"; // IoT Device 1

constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;

constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

constexpr char BLINKING_INTERVAL_ATTR[] = "blinkingInterval";
constexpr char LED_MODE_ATTR[] = "ledMode";
constexpr char LED_STATE_ATTR[] = "ledState";

volatile bool attributesChanged = false;
volatile int ledMode = 0;
volatile bool ledState = false;
volatile bool wifiConnected = false;
volatile bool tbConnected = false;

constexpr uint16_t BLINKING_INTERVAL_MS_MIN = 10U;
constexpr uint16_t BLINKING_INTERVAL_MS_MAX = 60000U;
constexpr uint16_t WIFI_CONNECT_CHECKING_INTERVAL_MS = 10000U;
constexpr uint16_t TB_CONNECT_CHECKING_INTERVAL_MS = 10000U;
constexpr uint16_t ATTRIBUTE_CHANGED_CHECKING_INTERVAL_MS = 10U;
constexpr uint16_t TB_LOOP_INTERVAL_MS = 10U;
volatile uint16_t blinkingInterval = 1000U;

uint32_t previousStateChange;

constexpr int16_t telemetrySendInterval = 10000U;
constexpr int16_t attributeSendInterval = telemetrySendInterval;
uint32_t previousDataSend;

constexpr std::array<const char *, 2U> SHARED_ATTRIBUTES_LIST = {
  LED_STATE_ATTR,
  BLINKING_INTERVAL_ATTR
};

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

DHT20 dht20;

RPC_Response setLedSwitchState(const RPC_Data &data) {
    Serial.println("Received Switch state");
    bool newState = data;
    Serial.print("Switch state change: ");
    Serial.println(newState);
    digitalWrite(LED_PIN, newState);
    attributesChanged = true;
    return RPC_Response("setLedSwitchValue", newState);
}

const std::array<RPC_Callback, 1U> callbacks = {
  RPC_Callback{ "setLedSwitchValue", setLedSwitchState }
};

void processSharedAttributes(const Shared_Attribute_Data &data) {
  for (auto it = data.begin(); it != data.end(); ++it) {
    if (strcmp(it->key().c_str(), BLINKING_INTERVAL_ATTR) == 0) {
      const uint16_t new_interval = it->value().as<uint16_t>();
      if (new_interval >= BLINKING_INTERVAL_MS_MIN && new_interval <= BLINKING_INTERVAL_MS_MAX) {
        blinkingInterval = new_interval;
        Serial.print("Blinking interval is set to: ");
        Serial.println(new_interval);
      }
    } else if (strcmp(it->key().c_str(), LED_STATE_ATTR) == 0) {
      ledState = it->value().as<bool>();
      digitalWrite(LED_PIN, ledState);
      Serial.print("LED state is set to: ");
      Serial.println(ledState);
    }
  }
  attributesChanged = true;
}

const Shared_Attribute_Callback attributes_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());
const Attribute_Request_Callback attribute_shared_request_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());

void InitWiFi() {
  Serial.println("Connecting to AP ...");
  // Attempting to establish a connection to the given WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    // Delay 500ms until a connection has been successfully established
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

void CheckTBConnection() {
  if (!tb.connected()) {
    Serial.print("Connecting to: ");
    Serial.print(THINGSBOARD_SERVER);
    Serial.print(" with token ");
    Serial.println(TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
      Serial.println("Failed to connect");
      return;
    }

    tb.sendAttributeData("macAddress", WiFi.macAddress().c_str());

    Serial.println("Subscribing for RPC...");
    if (!tb.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) {
      Serial.println("Failed to subscribe for RPC");
      return;
    }

    if (!tb.Shared_Attributes_Subscribe(attributes_callback)) {
      Serial.println("Failed to subscribe for shared attribute updates");
      return;
    }

    Serial.println("Subscribe done");

    if (!tb.Shared_Attributes_Request(attribute_shared_request_callback)) {
      Serial.println("Failed to request for shared attributes");
      return;
    }
  }
}

/* Free RTOS Tasks */
void TaskCheckWiFiConnection(void *pvParameters) {
  while(1) {
    // Check to ensure we aren't connected yet
    const wl_status_t status = WiFi.status();
    if (status != WL_CONNECTED) {
      // If we aren't establish a new connection to the given WiFi network
      InitWiFi();
      // Reconnect to ThingsBoard right after reconnect to WiFi
      CheckTBConnection(); 
    }
    vTaskDelay(pdMS_TO_TICKS(WIFI_CONNECT_CHECKING_INTERVAL_MS));
  }
}

void TaskCheckTBConnection(void *pvParameters) {
  while(1) {
    CheckTBConnection();
    vTaskDelay(pdMS_TO_TICKS(TB_CONNECT_CHECKING_INTERVAL_MS));
  }
}

void TaskCheckAttributesChanged(void *pvParameters) {
  while(1) {
    if (attributesChanged) {
      attributesChanged = false;
      tb.sendAttributeData(LED_STATE_ATTR, digitalRead(LED_PIN));
    }
    vTaskDelay(pdMS_TO_TICKS(ATTRIBUTE_CHANGED_CHECKING_INTERVAL_MS));
  }
}

void TaskReadAndSendTelemetryData(void *pvParameters) {
  while(1) {
    dht20.read();
    
    float temperature = dht20.getTemperature();
    float humidity = dht20.getHumidity();

    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("Failed to read from DHT20 sensor!");
    } else {
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.print(" °C, Humidity: ");
      Serial.print(humidity);
      Serial.println(" %");

      tb.sendTelemetryData("temperature", temperature);
      tb.sendTelemetryData("humidity", humidity);
    }
    vTaskDelay(pdMS_TO_TICKS(telemetrySendInterval));
  }
}

void TaskSendAttributeData(void *pvParameters) {
  while(1) {
    tb.sendAttributeData("rssi", WiFi.RSSI());
    tb.sendAttributeData("channel", WiFi.channel());
    tb.sendAttributeData("bssid", WiFi.BSSIDstr().c_str());
    tb.sendAttributeData("localIp", WiFi.localIP().toString().c_str());
    tb.sendAttributeData("ssid", WiFi.SSID().c_str());
    vTaskDelay(pdMS_TO_TICKS(attributeSendInterval));
  }
}

void TaskTBLoop(void *pvParameters) {
  while (1) {
    tb.loop();
    vTaskDelay(pdMS_TO_TICKS(TB_LOOP_INTERVAL_MS));
  }
}

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  pinMode(LED_PIN, OUTPUT);
  delay(1000);
  InitWiFi();

  Wire.begin(SDA_PIN, SCL_PIN);
  dht20.begin();
  
  xTaskCreate(TaskCheckWiFiConnection, "Check WiFi connection", 2048, NULL, 2, NULL);
  xTaskCreate(TaskCheckTBConnection, "Check Thingsboard connection", 2048, NULL, 2, NULL);
  xTaskCreate(TaskCheckAttributesChanged, "Check attributes changed", 2048, NULL, 2, NULL);
  xTaskCreate(TaskReadAndSendTelemetryData, "Read and send telemetry data", 2048, NULL, 2, NULL);
  xTaskCreate(TaskSendAttributeData, "Send attribute data", 2048, NULL, 2, NULL);
  xTaskCreate(TaskTBLoop, "tb.loop()", 2048, NULL, 2, NULL);
}

void loop() {
  
}
