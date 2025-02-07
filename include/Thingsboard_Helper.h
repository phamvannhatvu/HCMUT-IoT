#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include <WiFi.h>



constexpr char TOKEN[] = "test1";

constexpr char USERNAME[] = "test1";
constexpr char PASSWORD[] = "test1";

constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;
constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;

constexpr char TEMPERATURE_KEY[] = "temperature";
constexpr char HUMIDITY_KEY[] = "humidity";

/*
extern WiFiClient wifiClient;
extern Arduino_MQTT_Client mqttClient;
extern ThingsBoard thingsboardClient;
extern boolean isThingsBoardConnected;
*/

void initThingsBoard();

WiFiClient getWifiClient();
Arduino_MQTT_Client getMqttClient();
ThingsBoard getThingsboardClient();
boolean getThingsboardConnectedStatus();
void setThingsboardConnectedStatus(boolean status);

