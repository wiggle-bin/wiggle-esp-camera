#include <cstdint>
#ifndef SECRETS_H
#define SECRETS_H

const char* ssid = "";        // Change this
const char* password = ""; // Change this

// ESP-NOW peer MAC address (replace with your receiver's MAC)
#define RECEIVER_MAC {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}

// MQTT broker settings
const char* mqtt_server = "";
const int mqtt_port = 1883;
const char* mqtt_user = "";
const char* mqtt_password = ""; // Change if needed

#endif