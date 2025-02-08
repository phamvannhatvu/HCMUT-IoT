#include "Thingsboard_Helper.h"

boolean isThingsBoardConnected = 0;
WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard thingsboardClient(mqttClient, MAX_MESSAGE_SIZE);
RPC_Response processSetLedStatus(const RPC_Data &data);

WiFiClient getWifiClient(){
    return wifiClient;
}
Arduino_MQTT_Client getMqttClient(){
    return mqttClient;
}
ThingsBoard getThingsboardClient(){
    return thingsboardClient;
}
boolean getThingsboardConnectedStatus(){
    return isThingsBoardConnected;
}
void setThingsboardConnectedStatus(boolean status){
    isThingsBoardConnected = status;
}

void initThingsBoard(){
    
    Serial.print("Connecting to: ");
    Serial.print(THINGSBOARD_SERVER);
    Serial.print(" with token ");
    Serial.println(TOKEN);
    if (!thingsboardClient.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT, USERNAME, PASSWORD)) {
        Serial.println("Failed to connect");
        isThingsBoardConnected = false;
        
        delay(5000);
    }else{
        isThingsBoardConnected = true;
    }
         
}
void reconnectThingsBoard(){
    if (!thingsboardClient.connected()) {
        initThingsBoard();
    }
}