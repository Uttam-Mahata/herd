/*
 * HERD Wildlife Detection - Sensor Node Firmware
 * ESP32-CAM + PIR Sensor + ESP-NOW Mesh
 * 
 * Features:
 * - Motion detection via PIR sensor
 * - Image capture with ESP32-CAM
 * - ESP-NOW mesh networking
 * - Deep sleep power management
 * - Multi-gateway failover
 * - Store-and-forward capability
 */

#include "esp_camera.h"
#include "esp_now.h"
#include "WiFi.h"
#include "esp_wifi.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "SD.h"
#include "FS.h"
#include <vector>

// Node Configuration
#define NODE_ID "HERD_NODE_001"  // Change for each node
#define PIR_PIN GPIO_NUM_13
#define FLASH_LED_PIN GPIO_NUM_4
#define BATTERY_PIN A0
#define SD_CS_PIN GPIO_NUM_5

// Camera Configuration (AI-Thinker ESP32-CAM)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Power Management
#define DEEP_SLEEP_DURATION 300000000  // 5 minutes in microseconds
#define MAX_WAKE_TIME 60000            // 1 minute max wake time
#define LOW_BATTERY_THRESHOLD 3.3      // Volts

// Mesh Configuration
#define MAX_RETRIES 3
#define PACKET_DELAY_MS 50
#define ACK_TIMEOUT_MS 5000
#define MAX_PACKET_SIZE 200

// Packet Types
enum PacketType {
    IMAGE_HEADER = 0x01,
    IMAGE_DATA = 0x02,
    IMAGE_ACK = 0x03,
    HEARTBEAT = 0x04,
    NODE_STATUS = 0x05,
    GATEWAY_BEACON = 0x06,
    ROUTE_DISCOVERY = 0x07
};

// Mesh Packet Structure
struct MeshPacket {
    uint8_t type;
    uint8_t nodeId[6];
    uint8_t gatewayId[6];
    uint8_t nextHop[6];
    uint32_t timestamp;
    uint16_t packetIndex;
    uint16_t totalPackets;
    uint16_t dataSize;
    uint32_t imageSize;
    float batteryVoltage;
    int16_t rssi;
    uint8_t hopCount;
    uint8_t data[MAX_PACKET_SIZE];
    uint16_t checksum;
} __attribute__((packed));

// Gateway Information
struct GatewayInfo {
    uint8_t mac[6];
    String zone;
    int16_t rssi;
    unsigned long lastSeen;
    bool isActive;
    uint8_t priority;
    uint8_t hopCount;
};

// Global Variables
std::vector<GatewayInfo> availableGateways;
int currentGatewayIndex = -1;
uint32_t bootCount = 0;
bool motionDetected = false;
unsigned long wakeupTime = 0;
float batteryVoltage = 0.0;
bool cameraInitialized = false;

// RTC Memory for persistent data
RTC_DATA_ATTR uint32_t rtc_bootCount = 0;
RTC_DATA_ATTR uint8_t rtc_primaryGateway[6] = {0};
RTC_DATA_ATTR uint8_t rtc_nextHop[6] = {0};

void setup() {
    Serial.begin(115200);
    wakeupTime = millis();
    bootCount = ++rtc_bootCount;
    
    Serial.printf("\n🌿 HERD Sensor Node %s - Boot #%d\n", NODE_ID, bootCount);
    
    // Initialize pins
    pinMode(PIR_PIN, INPUT);
    pinMode(FLASH_LED_PIN, OUTPUT);
    digitalWrite(FLASH_LED_PIN, LOW);
    
    // Check wakeup reason
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    
    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_EXT0:
            Serial.println("🎯 Wakeup caused by PIR motion detection");
            motionDetected = true;
            break;
        case ESP_SLEEP_WAKEUP_TIMER:
            Serial.println("⏰ Wakeup caused by timer (heartbeat)");
            motionDetected = false;
            break;
        default:
            Serial.println("🔄 First boot or power-on reset");
            motionDetected = false;
            break;
    }
    
    // Read battery voltage
    batteryVoltage = readBatteryVoltage();
    Serial.printf("🔋 Battery voltage: %.2fV\n", batteryVoltage);
    
    // Check for low battery
    if (batteryVoltage < LOW_BATTERY_THRESHOLD) {
        Serial.println("⚠️ Low battery! Entering extended sleep mode");
        enterExtendedSleep();
        return;
    }
    
    // Initialize SD card
    initializeSD();
    
    // Initialize WiFi and ESP-NOW
    setupESPNow();
    
    // Discover gateways
    discoverGateways();
    
    if (motionDetected) {
        // Handle motion detection
        handleMotionDetection();
    } else {
        // Send heartbeat
        sendHeartbeat();
    }
    
    // Check if wake time exceeded
    if (millis() - wakeupTime > MAX_WAKE_TIME) {
        Serial.println("⏰ Max wake time exceeded, entering sleep");
    }
    
    // Enter deep sleep
    enterDeepSleep();
}

void loop() {
    // Should never reach here due to deep sleep
    delay(1000);
}

void setupESPNow() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    
    // Get node MAC address
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    Serial.printf("📡 Node MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("❌ ESP-NOW init failed");
        enterDeepSleep();
        return;
    }
    
    // Register callbacks
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataReceived);
    
    Serial.println("✅ ESP-NOW initialized");
}

void discoverGateways() {
    Serial.println("🔍 Discovering gateways...");
    
    // Clear previous gateway list
    availableGateways.clear();
    
    // Send route discovery broadcast
    broadcastRouteDiscovery();
    
    // Listen for gateway beacons for 5 seconds
    unsigned long startTime = millis();
    while (millis() - startTime < 5000) {
        delay(100);
    }
    
    if (availableGateways.empty()) {
        // Fallback to RTC stored gateway/route
        if (rtc_primaryGateway[0] != 0) {
            Serial.println("📡 Using RTC stored gateway route");
            GatewayInfo gateway;
            memcpy(gateway.mac, rtc_primaryGateway, 6);
            gateway.zone = "Stored";
            gateway.priority = 1;
            gateway.isActive = true;
            gateway.lastSeen = millis();
            gateway.hopCount = 1;
            availableGateways.push_back(gateway);
        } else {
            Serial.println("❌ No gateways found!");
            enterDeepSleep();
            return;
        }
    }
    
    // Sort gateways by priority and hop count
    sortGatewaysByOptimal();
    
    // Add best gateway as ESP-NOW peer
    addGatewayAsPeer(0);
    
    Serial.printf("✅ Found %d gateway(s)\n", availableGateways.size());
}

void broadcastRouteDiscovery() {
    MeshPacket discovery;
    discovery.type = ROUTE_DISCOVERY;
    esp_wifi_get_mac(WIFI_IF_STA, discovery.nodeId);
    memset(discovery.gatewayId, 0xFF, 6); // Broadcast
    discovery.timestamp = millis();
    discovery.batteryVoltage = batteryVoltage;
    discovery.hopCount = 0;
    discovery.dataSize = 0;
    
    uint8_t broadcastMAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    
    // Add broadcast peer temporarily
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, broadcastMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
    
    esp_now_send(broadcastMAC, (uint8_t*)&discovery, sizeof(discovery));
    
    // Remove broadcast peer
    esp_now_del_peer(broadcastMAC);
}

void sortGatewaysByOptimal() {
    // Sort by hop count first, then by priority, then by RSSI
    std::sort(availableGateways.begin(), availableGateways.end(), 
              [](const GatewayInfo& a, const GatewayInfo& b) {
                  if (a.hopCount != b.hopCount) return a.hopCount < b.hopCount;
                  if (a.priority != b.priority) return a.priority < b.priority;
                  return a.rssi > b.rssi; // Higher RSSI is better
              });
}

void addGatewayAsPeer(int index) {
    if (index >= availableGateways.size()) return;
    
    uint8_t* targetMAC = availableGateways[index].mac;
    
    // Use next hop from RTC if available
    if (rtc_nextHop[0] != 0) {
        targetMAC = rtc_nextHop;
    }
    
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, targetMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        currentGatewayIndex = index;
        Serial.printf("✅ Added gateway route to %s zone\n", 
                     availableGateways[index].zone.c_str());
        
        // Store in RTC memory
        memcpy(rtc_primaryGateway, availableGateways[index].mac, 6);
        memcpy(rtc_nextHop, targetMAC, 6);
    }
}

void handleMotionDetection() {
    Serial.println("📸 Motion detected! Capturing image...");
    
    // Initialize camera
    if (!initCamera()) {
        Serial.println("❌ Camera initialization failed");
        return;
    }
    
    // Brief flash to illuminate subject
    digitalWrite(FLASH_LED_PIN, HIGH);
    delay(100);
    
    // Capture image
    camera_fb_t* fb = esp_camera_fb_get();
    
    digitalWrite(FLASH_LED_PIN, LOW);
    
    if (!fb) {
        Serial.println("❌ Camera capture failed");
        return;
    }
    
    Serial.printf("📷 Image captured: %d bytes\n", fb->len);
    
    // Try to send image via ESP-NOW mesh
    bool success = sendImageViaMesh(fb->buf, fb->len);
    
    if (!success) {
        // Store image to SD card for later transmission
        String filename = storeImageToSD(fb->buf, fb->len);
        if (filename != "") {
            Serial.printf("💾 Image stored to SD: %s\n", filename.c_str());
        }
    } else {
        Serial.println("✅ Image transmitted successfully");
    }
    
    // Release frame buffer
    esp_camera_fb_return(fb);
    
    // Deinitialize camera to save power
    esp_camera_deinit();
    cameraInitialized = false;
}

bool initCamera() {
    if (cameraInitialized) return true;
    
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    
    // Frame size and quality settings for power efficiency
    config.frame_size = FRAMESIZE_VGA;  // 640x480
    config.jpeg_quality = 15;           // Balance quality vs size
    config.fb_count = 1;
    
    // Initialize camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("❌ Camera init failed with error 0x%x\n", err);
        return false;
    }
    
    cameraInitialized = true;
    return true;
}

bool sendImageViaMesh(uint8_t* imageData, size_t imageSize) {
    if (currentGatewayIndex < 0) {
        Serial.println("❌ No gateway route available");
        return false;
    }
    
    uint32_t timestamp = millis();
    size_t packetSize = MAX_PACKET_SIZE;
    uint16_t totalPackets = (imageSize + packetSize - 1) / packetSize;
    
    Serial.printf("📤 Sending image: %d bytes in %d packets\n", imageSize, totalPackets);
    
    // Send image header first
    if (!sendImageHeader(imageSize, totalPackets, timestamp)) {
        Serial.println("❌ Failed to send image header");
        return false;
    }
    
    // Send image data packets
    for (uint16_t i = 0; i < totalPackets; i++) {
        size_t offset = i * packetSize;
        size_t currentPacketSize = min(packetSize, imageSize - offset);
        
        if (!sendImagePacket(imageData + offset, currentPacketSize, i, totalPackets, timestamp)) {
            Serial.printf("❌ Failed to send packet %d\n", i);
            
            // Try failover to next gateway
            if (tryNextGateway()) {
                i--; // Retry this packet
                continue;
            } else {
                return false;
            }
        }
        
        // Progress indicator
        if (i % 5 == 0) {
            Serial.printf("📤 Progress: %d/%d packets\n", i + 1, totalPackets);
        }
        
        delay(PACKET_DELAY_MS);
        
        // Check for timeout
        if (millis() - wakeupTime > MAX_WAKE_TIME) {
            Serial.println("⏰ Transmission timeout");
            return false;
        }
    }
    
    Serial.println("✅ All image packets sent");
    return true;
}

bool sendImageHeader(uint32_t imageSize, uint16_t totalPackets, uint32_t timestamp) {
    MeshPacket header;
    header.type = IMAGE_HEADER;
    esp_wifi_get_mac(WIFI_IF_STA, header.nodeId);
    memcpy(header.gatewayId, availableGateways[currentGatewayIndex].mac, 6);
    memcpy(header.nextHop, rtc_nextHop, 6);
    header.timestamp = timestamp;
    header.totalPackets = totalPackets;
    header.imageSize = imageSize;
    header.batteryVoltage = batteryVoltage;
    header.rssi = WiFi.RSSI();
    header.hopCount = availableGateways[currentGatewayIndex].hopCount;
    header.dataSize = 0;
    header.checksum = calculateChecksum((uint8_t*)&header, sizeof(header) - 2);
    
    return sendPacketWithRetry(&header);
}

bool sendImagePacket(uint8_t* data, size_t dataSize, uint16_t packetIndex, 
                    uint16_t totalPackets, uint32_t timestamp) {
    MeshPacket packet;
    packet.type = IMAGE_DATA;
    esp_wifi_get_mac(WIFI_IF_STA, packet.nodeId);
    memcpy(packet.gatewayId, availableGateways[currentGatewayIndex].mac, 6);
    memcpy(packet.nextHop, rtc_nextHop, 6);
    packet.timestamp = timestamp;
    packet.packetIndex = packetIndex;
    packet.totalPackets = totalPackets;
    packet.dataSize = dataSize;
    packet.batteryVoltage = batteryVoltage;
    packet.rssi = WiFi.RSSI();
    packet.hopCount = availableGateways[currentGatewayIndex].hopCount;
    
    memcpy(packet.data, data, dataSize);
    packet.checksum = calculateChecksum((uint8_t*)&packet, sizeof(packet) - 2);
    
    return sendPacketWithRetry(&packet);
}

bool sendPacketWithRetry(MeshPacket* packet) {
    uint8_t* targetMAC = rtc_nextHop[0] != 0 ? rtc_nextHop : availableGateways[currentGatewayIndex].mac;
    
    for (int retry = 0; retry < MAX_RETRIES; retry++) {
        esp_err_t result = esp_now_send(targetMAC, (uint8_t*)packet, sizeof(MeshPacket));
        
        if (result == ESP_OK) {
            return true;
        }
        
        Serial.printf("⚠️ Send attempt %d failed: 0x%x\n", retry + 1, result);
        delay(100 * (retry + 1)); // Increasing delay
    }
    
    return false;
}

bool tryNextGateway() {
    if (currentGatewayIndex + 1 < availableGateways.size()) {
        // Remove current peer
        uint8_t* currentTarget = rtc_nextHop[0] != 0 ? rtc_nextHop : availableGateways[currentGatewayIndex].mac;
        esp_now_del_peer(currentTarget);
        
        // Try next gateway
        addGatewayAsPeer(currentGatewayIndex + 1);
        
        Serial.printf("🔄 Switched to gateway %s\n", 
                     availableGateways[currentGatewayIndex].zone.c_str());
        return true;
    }
    
    return false;
}

void sendHeartbeat() {
    if (currentGatewayIndex < 0) return;
    
    MeshPacket heartbeat;
    heartbeat.type = HEARTBEAT;
    esp_wifi_get_mac(WIFI_IF_STA, heartbeat.nodeId);
    memcpy(heartbeat.gatewayId, availableGateways[currentGatewayIndex].mac, 6);
    memcpy(heartbeat.nextHop, rtc_nextHop, 6);
    heartbeat.timestamp = millis();
    heartbeat.batteryVoltage = batteryVoltage;
    heartbeat.rssi = WiFi.RSSI();
    heartbeat.hopCount = availableGateways[currentGatewayIndex].hopCount;
    heartbeat.dataSize = 0;
    heartbeat.checksum = calculateChecksum((uint8_t*)&heartbeat, sizeof(heartbeat) - 2);
    
    if (sendPacketWithRetry(&heartbeat)) {
        Serial.println("💓 Heartbeat sent");
    }
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // Optional: Handle send status for debugging
    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.printf("⚠️ Send failed to %02X:%02X:%02X:%02X:%02X:%02X\n",
                      mac_addr[0], mac_addr[1], mac_addr[2], 
                      mac_addr[3], mac_addr[4], mac_addr[5]);
    }
}

void onDataReceived(const uint8_t* mac, const uint8_t* data, int len) {
    if (len != sizeof(MeshPacket)) return;
    
    MeshPacket* packet = (MeshPacket*)data;
    
    // Verify checksum
    uint16_t receivedChecksum = packet->checksum;
    packet->checksum = 0;
    uint16_t calculatedChecksum = calculateChecksum((uint8_t*)packet, sizeof(MeshPacket) - 2);
    packet->checksum = receivedChecksum;
    
    if (receivedChecksum != calculatedChecksum) {
        Serial.println("❌ Packet checksum mismatch");
        return;
    }
    
    if (packet->type == GATEWAY_BEACON) {
        handleGatewayBeacon(packet, mac);
    } else if (packet->type == IMAGE_ACK) {
        handleImageAck(packet);
    }
}

void handleGatewayBeacon(MeshPacket* packet, const uint8_t* mac) {
    // Check if this gateway is already known
    bool found = false;
    for (auto& gateway : availableGateways) {
        if (memcmp(gateway.mac, packet->gatewayId, 6) == 0) {
            gateway.lastSeen = millis();
            gateway.rssi = packet->rssi;
            gateway.hopCount = packet->hopCount + 1; // Add one hop
            found = true;
            break;
        }
    }
    
    if (!found && availableGateways.size() < 5) { // Limit gateway list
        // Add new gateway
        GatewayInfo newGateway;
        memcpy(newGateway.mac, packet->gatewayId, 6);
        newGateway.zone = String((char*)packet->data);
        newGateway.rssi = packet->rssi;
        newGateway.lastSeen = millis();
        newGateway.isActive = true;
        newGateway.priority = packet->hopCount + 1;
        newGateway.hopCount = packet->hopCount + 1;
        
        availableGateways.push_back(newGateway);
        
        Serial.printf("📡 Discovered gateway: %s zone (%d hops)\n", 
                     newGateway.zone.c_str(), newGateway.hopCount);
    }
}

void handleImageAck(MeshPacket* packet) {
    if (packet->dataSize > 0) {
        Serial.println("✅ Image ACK received");
    } else {
        Serial.println("❌ Image NACK received");
    }
}

void initializeSD() {
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("⚠️ SD Card initialization failed");
        return;
    }
    
    Serial.println("✅ SD Card initialized");
}

String storeImageToSD(uint8_t* imageData, size_t imageSize) {
    if (!SD.begin(SD_CS_PIN)) {
        return "";
    }
    
    // Create filename with timestamp
    String filename = "/img_" + String(millis()) + ".jpg";
    
    File file = SD.open(filename, FILE_WRITE);
    if (!file) {
        Serial.println("❌ Failed to create SD file");
        return "";
    }
    
    size_t bytesWritten = file.write(imageData, imageSize);
    file.close();
    
    if (bytesWritten == imageSize) {
        return filename;
    }
    
    return "";
}

uint16_t calculateChecksum(uint8_t* data, size_t length) {
    uint16_t checksum = 0;
    for (size_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;
}

float readBatteryVoltage() {
    // Read battery voltage through voltage divider
    int rawReading = analogRead(BATTERY_PIN);
    float voltage = (rawReading / 4095.0) * 3.3 * 2.0; // Assuming 2:1 voltage divider
    return voltage;
}

void enterDeepSleep() {
    unsigned long activeTime = millis() - wakeupTime;
    Serial.printf("💤 Active time: %lu ms\n", activeTime);
    
    // Configure PIR pin as wakeup source
    esp_sleep_enable_ext0_wakeup(PIR_PIN, 1); // Wake on HIGH
    
    // Configure timer wakeup for heartbeat
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_DURATION);
    
    Serial.printf("💤 Entering deep sleep for %d seconds...\n", DEEP_SLEEP_DURATION / 1000000);
    delay(100);
    
    esp_deep_sleep_start();
}

void enterExtendedSleep() {
    // Extended sleep for low battery - 20 minutes
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_DURATION * 4);
    Serial.println("💤 Entering extended sleep (low battery)...");
    delay(100);
    esp_deep_sleep_start();
}
