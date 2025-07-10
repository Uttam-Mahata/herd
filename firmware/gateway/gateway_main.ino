/*
 * HERD Wildlife Detection - Gateway Node Firmware
 * ESP32 with ESP-NOW Mesh Receiver + Android Communication
 * 
 * Features:
 * - ESP-NOW mesh receiver for sensor nodes
 * - Image reassembly from multiple packets
 * - WiFi hotspot for Android communication
 * - HTTP server for Android app interface
 * - Multi-gateway coordination
 * - Store-and-forward routing
 */

#include "esp_now.h"
#include "WiFi.h"
#include "esp_wifi.h"
#include "WebServer.h"
#include "ArduinoJson.h"
#include "SPIFFS.h"
#include "HTTPClient.h"
#include <map>
#include <vector>

// Gateway Configuration
#define GATEWAY_ID "HERD_GW_001"  // Change for each gateway
#define GATEWAY_ZONE "North"      // Zone identifier
#define GATEWAY_PRIORITY 1        // 1=primary, 2=secondary, etc.

// Network Configuration
#define HOTSPOT_SSID "HERD_Gateway_001"
#define HOTSPOT_PASSWORD "HerdGateway2024"
#define HTTP_SERVER_PORT 80
#define ANDROID_API_PORT 8080

// Mesh Configuration
#define MAX_PENDING_IMAGES 10
#define BEACON_INTERVAL 30000     // 30 seconds
#define CLEANUP_INTERVAL 300000   // 5 minutes
#define MAX_PACKET_SIZE 200

// Backend Configuration
#define BACKEND_URL "http://your-server.com/api"
#define MAX_RETRY_ATTEMPTS 3

// Packet Types (same as node)
enum PacketType {
    IMAGE_HEADER = 0x01,
    IMAGE_DATA = 0x02,
    IMAGE_ACK = 0x03,
    HEARTBEAT = 0x04,
    NODE_STATUS = 0x05,
    GATEWAY_BEACON = 0x06,
    ROUTE_DISCOVERY = 0x07
};

// Mesh Packet Structure (same as node)
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

// Image Assembly Structure
struct ImageAssembly {
    String nodeId;
    uint32_t timestamp;
    uint32_t totalSize;
    uint16_t totalPackets;
    uint16_t receivedPackets;
    uint8_t* imageData;
    bool* packetReceived;
    unsigned long lastPacketTime;
    float batteryVoltage;
    int16_t rssi;
    uint8_t hopCount;
};

// Node Information
struct NodeInfo {
    String nodeId;
    unsigned long lastSeen;
    float batteryVoltage;
    int16_t rssi;
    uint32_t imagesSent;
    bool isActive;
};

// Global Variables
WebServer server(HTTP_SERVER_PORT);
std::map<String, ImageAssembly> pendingImages;
std::map<String, NodeInfo> knownNodes;
unsigned long lastBeaconTime = 0;
unsigned long lastCleanupTime = 0;
uint32_t processedImages = 0;
uint32_t receivedHeartbeats = 0;

void setup() {
    Serial.begin(115200);
    Serial.printf("\n🏰 HERD Gateway %s - Zone %s\n", GATEWAY_ID, GATEWAY_ZONE);
    
    // Initialize SPIFFS for image storage
    if (!SPIFFS.begin(true)) {
        Serial.println("❌ SPIFFS initialization failed");
        return;
    }
    
    // Setup WiFi hotspot
    setupWiFiHotspot();
    
    // Initialize ESP-NOW
    setupESPNow();
    
    // Setup HTTP server for Android communication
    setupHTTPServer();
    
    // Print gateway information
    printGatewayInfo();
    
    Serial.println("✅ Gateway initialization complete");
}

void loop() {
    // Handle HTTP requests
    server.handleClient();
    
    // Send periodic beacon
    if (millis() - lastBeaconTime > BEACON_INTERVAL) {
        sendGatewayBeacon();
        lastBeaconTime = millis();
    }
    
    // Cleanup old assemblies
    if (millis() - lastCleanupTime > CLEANUP_INTERVAL) {
        cleanupOldAssemblies();
        lastCleanupTime = millis();
    }
    
    // Check for completed images
    checkCompletedImages();
    
    delay(100);
}

void setupWiFiHotspot() {
    WiFi.mode(WIFI_AP_STA);
    
    // Configure Access Point
    WiFi.softAP(HOTSPOT_SSID, HOTSPOT_PASSWORD);
    
    IPAddress IP = WiFi.softAPIP();
    Serial.printf("📶 WiFi Hotspot created: %s\n", HOTSPOT_SSID);
    Serial.printf("🌐 Gateway IP: %s\n", IP.toString().c_str());
    
    // Try to connect to internet (optional for backend communication)
    // This would connect to a 4G hotspot or existing WiFi network
    // WiFi.begin("InternetNetwork", "password");
}

void setupESPNow() {
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("❌ ESP-NOW init failed");
        return;
    }
    
    // Register callbacks
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataReceived);
    
    // Get gateway MAC address
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    Serial.printf("📡 Gateway MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
    Serial.println("✅ ESP-NOW initialized");
}

void setupHTTPServer() {
    // API endpoint for Android app to get pending images
    server.on("/api/images/pending", HTTP_GET, handleGetPendingImages);
    
    // API endpoint for Android app to report detection results
    server.on("/api/detection/result", HTTP_POST, handleDetectionResult);
    
    // API endpoint for node status
    server.on("/api/nodes/status", HTTP_GET, handleGetNodeStatus);
    
    // API endpoint for gateway status
    server.on("/api/gateway/status", HTTP_GET, handleGetGatewayStatus);
    
    // API endpoint for Android app to get specific image
    server.on("/api/image", HTTP_GET, handleGetImage);
    
    // CORS headers for all requests
    server.enableCORS(true);
    
    server.begin();
    Serial.printf("🌐 HTTP server started on port %d\n", HTTP_SERVER_PORT);
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // Optional: Log send status
}

void onDataReceived(const uint8_t* mac, const uint8_t* data, int len) {
    if (len != sizeof(MeshPacket)) {
        Serial.printf("⚠️ Invalid packet size: %d\n", len);
        return;
    }
    
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
    
    // Update node information
    updateNodeInfo(packet);
    
    switch (packet->type) {
        case IMAGE_HEADER:
            handleImageHeader(packet);
            break;
        case IMAGE_DATA:
            handleImageData(packet);
            break;
        case HEARTBEAT:
            handleHeartbeat(packet);
            break;
        case ROUTE_DISCOVERY:
            handleRouteDiscovery(packet, mac);
            break;
        default:
            Serial.printf("⚠️ Unknown packet type: 0x%02X\n", packet->type);
            break;
    }
}

void handleImageHeader(MeshPacket* packet) {
    String nodeId = macToString(packet->nodeId);
    String key = nodeId + "_" + String(packet->timestamp);
    
    Serial.printf("📋 Image header from %s: %d bytes, %d packets\n", 
                  nodeId.c_str(), packet->imageSize, packet->totalPackets);
    
    // Clean up any existing assembly for this node
    cleanupImageAssembly(key);
    
    // Create new image assembly
    ImageAssembly assembly;
    assembly.nodeId = nodeId;
    assembly.timestamp = packet->timestamp;
    assembly.totalSize = packet->imageSize;
    assembly.totalPackets = packet->totalPackets;
    assembly.receivedPackets = 0;
    assembly.imageData = (uint8_t*)malloc(packet->imageSize);
    assembly.packetReceived = (bool*)calloc(packet->totalPackets, sizeof(bool));
    assembly.lastPacketTime = millis();
    assembly.batteryVoltage = packet->batteryVoltage;
    assembly.rssi = packet->rssi;
    assembly.hopCount = packet->hopCount;
    
    if (assembly.imageData == nullptr || assembly.packetReceived == nullptr) {
        Serial.println("❌ Memory allocation failed");
        cleanupImageAssembly(key);
        return;
    }
    
    pendingImages[key] = assembly;
    
    // Send ACK
    sendImageAck(packet->nodeId, packet->timestamp, true);
}

void handleImageData(MeshPacket* packet) {
    String nodeId = macToString(packet->nodeId);
    String key = nodeId + "_" + String(packet->timestamp);
    
    auto it = pendingImages.find(key);
    if (it == pendingImages.end()) {
        Serial.printf("⚠️ No header found for image data from %s\n", nodeId.c_str());
        return;
    }
    
    ImageAssembly& assembly = it->second;
    
    // Check packet index validity
    if (packet->packetIndex >= assembly.totalPackets) {
        Serial.printf("⚠️ Invalid packet index: %d/%d\n", packet->packetIndex, assembly.totalPackets);
        return;
    }
    
    // Check if packet already received
    if (assembly.packetReceived[packet->packetIndex]) {
        Serial.printf("⚠️ Duplicate packet %d from %s\n", packet->packetIndex, nodeId.c_str());
        return;
    }
    
    // Copy packet data
    size_t offset = packet->packetIndex * MAX_PACKET_SIZE;
    size_t copySize = min((size_t)packet->dataSize, assembly.totalSize - offset);
    
    memcpy(assembly.imageData + offset, packet->data, copySize);
    assembly.packetReceived[packet->packetIndex] = true;
    assembly.receivedPackets++;
    assembly.lastPacketTime = millis();
    
    // Progress indicator
    if (packet->packetIndex % 10 == 0) {
        Serial.printf("📥 Progress %s: %d/%d packets\n", 
                     nodeId.c_str(), assembly.receivedPackets, assembly.totalPackets);
    }
}

void handleHeartbeat(MeshPacket* packet) {
    String nodeId = macToString(packet->nodeId);
    receivedHeartbeats++;
    
    Serial.printf("💓 Heartbeat from %s (Battery: %.2fV, RSSI: %ddBm)\n", 
                  nodeId.c_str(), packet->batteryVoltage, packet->rssi);
}

void handleRouteDiscovery(MeshPacket* packet, const uint8_t* mac) {
    // Respond with gateway beacon
    MeshPacket beacon;
    beacon.type = GATEWAY_BEACON;
    esp_wifi_get_mac(WIFI_IF_STA, beacon.gatewayId);
    memcpy(beacon.nodeId, packet->nodeId, 6);
    beacon.timestamp = millis();
    beacon.hopCount = 0; // This is the gateway
    beacon.dataSize = strlen(GATEWAY_ZONE);
    memcpy(beacon.data, GATEWAY_ZONE, beacon.dataSize);
    beacon.checksum = calculateChecksum((uint8_t*)&beacon, sizeof(beacon) - 2);
    
    // Add the requesting node as a peer temporarily
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
    
    esp_now_send(mac, (uint8_t*)&beacon, sizeof(beacon));
    
    // Remove peer after sending
    esp_now_del_peer(mac);
    
    Serial.printf("📡 Sent beacon to %s\n", macToString((uint8_t*)mac).c_str());
}

void sendGatewayBeacon() {
    MeshPacket beacon;
    beacon.type = GATEWAY_BEACON;
    esp_wifi_get_mac(WIFI_IF_STA, beacon.gatewayId);
    beacon.timestamp = millis();
    beacon.hopCount = 0;
    beacon.dataSize = strlen(GATEWAY_ZONE);
    memcpy(beacon.data, GATEWAY_ZONE, beacon.dataSize);
    beacon.checksum = calculateChecksum((uint8_t*)&beacon, sizeof(beacon) - 2);
    
    // Broadcast beacon
    uint8_t broadcastMAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, broadcastMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
    
    esp_now_send(broadcastMAC, (uint8_t*)&beacon, sizeof(beacon));
    
    esp_now_del_peer(broadcastMAC);
    
    Serial.printf("📡 Beacon broadcast (Zone: %s)\n", GATEWAY_ZONE);
}

void sendImageAck(uint8_t* nodeMAC, uint32_t timestamp, bool success) {
    MeshPacket ack;
    ack.type = IMAGE_ACK;
    esp_wifi_get_mac(WIFI_IF_STA, ack.gatewayId);
    memcpy(ack.nodeId, nodeMAC, 6);
    ack.timestamp = timestamp;
    ack.dataSize = success ? 1 : 0;
    ack.checksum = calculateChecksum((uint8_t*)&ack, sizeof(ack) - 2);
    
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, nodeMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
    
    esp_now_send(nodeMAC, (uint8_t*)&ack, sizeof(ack));
    
    esp_now_del_peer(nodeMAC);
}

void checkCompletedImages() {
    for (auto it = pendingImages.begin(); it != pendingImages.end();) {
        ImageAssembly& assembly = it->second;
        
        if (assembly.receivedPackets == assembly.totalPackets) {
            // Image complete!
            Serial.printf("✅ Image complete from %s: %d bytes\n", 
                         assembly.nodeId.c_str(), assembly.totalSize);
            
            // Store image to SPIFFS
            String filename = storeImageToSPIFFS(assembly);
            
            if (filename != "") {
                // Notify Android app
                notifyAndroidApp(assembly, filename);
                processedImages++;
            }
            
            // Cleanup
            cleanupImageAssembly(it->first);
            it = pendingImages.erase(it);
        } else {
            // Check for timeout (5 minutes)
            if (millis() - assembly.lastPacketTime > 300000) {
                Serial.printf("⏰ Image timeout from %s (%d/%d packets)\n", 
                             assembly.nodeId.c_str(), assembly.receivedPackets, assembly.totalPackets);
                cleanupImageAssembly(it->first);
                it = pendingImages.erase(it);
            } else {
                ++it;
            }
        }
    }
}

String storeImageToSPIFFS(const ImageAssembly& assembly) {
    String filename = "/img_" + assembly.nodeId + "_" + String(assembly.timestamp) + ".jpg";
    
    File file = SPIFFS.open(filename, FILE_WRITE);
    if (!file) {
        Serial.println("❌ Failed to create SPIFFS file");
        return "";
    }
    
    size_t bytesWritten = file.write(assembly.imageData, assembly.totalSize);
    file.close();
    
    if (bytesWritten == assembly.totalSize) {
        Serial.printf("💾 Image stored: %s (%d bytes)\n", filename.c_str(), assembly.totalSize);
        return filename;
    }
    
    return "";
}

void notifyAndroidApp(const ImageAssembly& assembly, const String& filename) {
    // Send HTTP POST to Android app running on same device
    HTTPClient http;
    
    // Android app listens on localhost:8080
    http.begin("http://192.168.4.2:8080/api/new-image");
    http.addHeader("Content-Type", "application/json");
    
    // Create JSON payload
    DynamicJsonDocument doc(1024);
    doc["node_id"] = assembly.nodeId;
    doc["timestamp"] = assembly.timestamp;
    doc["filename"] = filename;
    doc["file_size"] = assembly.totalSize;
    doc["battery_voltage"] = assembly.batteryVoltage;
    doc["rssi"] = assembly.rssi;
    doc["hop_count"] = assembly.hopCount;
    doc["gateway_id"] = GATEWAY_ID;
    doc["gateway_zone"] = GATEWAY_ZONE;
    
    String jsonString;
    serializeJson(doc, jsonString);
    
    int httpResponseCode = http.POST(jsonString);
    
    if (httpResponseCode > 0) {
        Serial.printf("📱 Android notified: %d\n", httpResponseCode);
    } else {
        Serial.printf("❌ Android notification failed: %d\n", httpResponseCode);
    }
    
    http.end();
}

// HTTP API Handlers
void handleGetPendingImages() {
    DynamicJsonDocument doc(2048);
    JsonArray images = doc.createNestedArray("pending_images");
    
    for (const auto& pair : pendingImages) {
        JsonObject img = images.createNestedObject();
        const ImageAssembly& assembly = pair.second;
        
        img["node_id"] = assembly.nodeId;
        img["timestamp"] = assembly.timestamp;
        img["progress"] = (float)assembly.receivedPackets / assembly.totalPackets;
        img["total_size"] = assembly.totalSize;
        img["battery_voltage"] = assembly.batteryVoltage;
        img["rssi"] = assembly.rssi;
        img["hop_count"] = assembly.hopCount;
    }
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
}

void handleDetectionResult() {
    if (!server.hasArg("plain")) {
        server.send(400, "text/plain", "Missing JSON body");
        return;
    }
    
    String body = server.arg("plain");
    DynamicJsonDocument doc(1024);
    
    if (deserializeJson(doc, body) != DeserializationError::Ok) {
        server.send(400, "text/plain", "Invalid JSON");
        return;
    }
    
    // Extract detection result
    String nodeId = doc["node_id"];
    uint32_t timestamp = doc["timestamp"];
    bool isWildlife = doc["is_wildlife"];
    float confidence = doc["confidence"];
    String species = doc["species"];
    
    Serial.printf("🤖 Detection result: %s - %s (%.2f confidence)\n", 
                  nodeId.c_str(), species.c_str(), confidence);
    
    if (isWildlife && confidence > 0.7) {
        // Forward to backend server
        forwardToBackend(doc);
    }
    
    server.send(200, "application/json", "{\"status\":\"received\"}");
}

void handleGetNodeStatus() {
    DynamicJsonDocument doc(2048);
    JsonArray nodes = doc.createNestedArray("nodes");
    
    for (const auto& pair : knownNodes) {
        JsonObject node = nodes.createNestedObject();
        const NodeInfo& info = pair.second;
        
        node["node_id"] = info.nodeId;
        node["last_seen"] = info.lastSeen;
        node["battery_voltage"] = info.batteryVoltage;
        node["rssi"] = info.rssi;
        node["images_sent"] = info.imagesSent;
        node["is_active"] = info.isActive;
        node["minutes_since_last_seen"] = (millis() - info.lastSeen) / 60000;
    }
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
}

void handleGetGatewayStatus() {
    DynamicJsonDocument doc(512);
    
    doc["gateway_id"] = GATEWAY_ID;
    doc["zone"] = GATEWAY_ZONE;
    doc["priority"] = GATEWAY_PRIORITY;
    doc["uptime_ms"] = millis();
    doc["processed_images"] = processedImages;
    doc["received_heartbeats"] = receivedHeartbeats;
    doc["pending_images"] = pendingImages.size();
    doc["known_nodes"] = knownNodes.size();
    doc["free_heap"] = ESP.getFreeHeap();
    doc["spiffs_used"] = SPIFFS.usedBytes();
    doc["spiffs_total"] = SPIFFS.totalBytes();
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
}

void handleGetImage() {
    if (!server.hasArg("filename")) {
        server.send(400, "text/plain", "Missing filename parameter");
        return;
    }
    
    String filename = server.arg("filename");
    
    if (!SPIFFS.exists(filename)) {
        server.send(404, "text/plain", "Image not found");
        return;
    }
    
    File file = SPIFFS.open(filename, FILE_READ);
    if (!file) {
        server.send(500, "text/plain", "Failed to open image");
        return;
    }
    
    server.streamFile(file, "image/jpeg");
    file.close();
}

void forwardToBackend(const DynamicJsonDocument& detection) {
    HTTPClient http;
    
    // Configure backend URL
    http.begin(BACKEND_URL "/wildlife-detection");
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", "Bearer your-api-key"); // Add if needed
    
    String jsonString;
    serializeJson(detection, jsonString);
    
    int httpResponseCode = http.POST(jsonString);
    
    if (httpResponseCode > 0) {
        Serial.printf("🌐 Backend notified: %d\n", httpResponseCode);
    } else {
        Serial.printf("❌ Backend notification failed: %d\n", httpResponseCode);
    }
    
    http.end();
}

// Helper Functions
void updateNodeInfo(const MeshPacket* packet) {
    String nodeId = macToString(packet->nodeId);
    
    NodeInfo& info = knownNodes[nodeId];
    info.nodeId = nodeId;
    info.lastSeen = millis();
    info.batteryVoltage = packet->batteryVoltage;
    info.rssi = packet->rssi;
    info.isActive = true;
    
    if (packet->type == IMAGE_HEADER) {
        info.imagesSent++;
    }
}

void cleanupImageAssembly(const String& key) {
    auto it = pendingImages.find(key);
    if (it != pendingImages.end()) {
        if (it->second.imageData != nullptr) {
            free(it->second.imageData);
        }
        if (it->second.packetReceived != nullptr) {
            free(it->second.packetReceived);
        }
    }
}

void cleanupOldAssemblies() {
    unsigned long currentTime = millis();
    
    for (auto it = pendingImages.begin(); it != pendingImages.end();) {
        if (currentTime - it->second.lastPacketTime > 600000) { // 10 minutes
            Serial.printf("🧹 Cleaning up old assembly: %s\n", it->first.c_str());
            cleanupImageAssembly(it->first);
            it = pendingImages.erase(it);
        } else {
            ++it;
        }
    }
    
    // Update node active status
    for (auto& pair : knownNodes) {
        if (currentTime - pair.second.lastSeen > 900000) { // 15 minutes
            pair.second.isActive = false;
        }
    }
}

String macToString(const uint8_t* mac) {
    char macStr[18];
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", 
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(macStr);
}

uint16_t calculateChecksum(uint8_t* data, size_t length) {
    uint16_t checksum = 0;
    for (size_t i = 0; i < length; i++) {
        checksum += data[i];
    }
    return checksum;
}

void printGatewayInfo() {
    Serial.println("\n📋 Gateway Information:");
    Serial.printf("   ID: %s\n", GATEWAY_ID);
    Serial.printf("   Zone: %s\n", GATEWAY_ZONE);
    Serial.printf("   Priority: %d\n", GATEWAY_PRIORITY);
    Serial.printf("   WiFi SSID: %s\n", HOTSPOT_SSID);
    Serial.printf("   HTTP Port: %d\n", HTTP_SERVER_PORT);
    Serial.printf("   Free Heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("   SPIFFS: %d/%d bytes\n", SPIFFS.usedBytes(), SPIFFS.totalBytes());
    Serial.println();
}
