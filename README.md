

---

#  Project HERD - Jungle Wildlife Intrusion Detection System using ESP32-CAM, LoRa, and Mobile-Connected Gateway

## 📌 Project Overview

This project aims to build a **long-range, low-power, camera-based wildlife monitoring system** using **ESP32-CAM + LoRa** sensor nodes and a **Gateway Node** connected to a mobile network (via smartphone hotspot or LTE module). The system detects motion (e.g., elephant intrusions), captures an image, and relays alerts through a chain of communication to a central **py web server** with **AI-powered image analysis (TensorFlow Lite)**.

---

## 🌐 High-Level Architecture

```mermaid
graph TD
    subgraph Jungle
        Node1[Sensor Node #1<br>ESP32-CAM + PIR + LoRa + SD]
        Node2[Sensor Node #2]
        Node3[Sensor Node #3]
        Gateway[Gateway Node<br>ESP32 + LoRa + Wi-Fi]
    end
    subgraph Internet
        Phone[Smartphone Hotspot]
        Server[py Web Server<br>+ MySQL + YOLO/TFLite AI]
        App[Mobile App<br>Push Notification]
    end

    Node1 --433MHz LoRa--> Gateway
    Node2 --433MHz LoRa--> Gateway
    Node3 --433MHz LoRa--> Gateway
    Gateway -->|Wi-Fi| Phone
    Phone -->|Mobile Network| Server
    Server --> App
```

---

## 📦 Hardware Components

### 1. Sensor Node (ESP32-CAM + SX1278 + PIR)

* **ESP32-CAM Module**
* **SX1278/SX1276 LoRa module @433MHz**
* **PIR Sensor (e.g., HC-SR501)**
* **MicroSD card for image storage**
* **Solar panel + TP4056 charger + LiPo battery**

### 2. Gateway Node

* **ESP32 (or ESP32-CAM reused)**
* **LoRa Module**
* **Wi-Fi (via smartphone hotspot or LTE module)**
* **Large solar panel + battery (always-on)**

---

## ⚙️ Software Stack

### 📍 Sensor Node Firmware

* ESP32 wakes on motion (via PIR)
* Captures and stores image on SD card
* Sends metadata (Node ID, filename, battery level) via LoRa

### 📍 Gateway Node Firmware

* Continuously listens for LoRa packets
* On receive, makes HTTP POST request to `alert.py` on server using smartphone Wi-Fi

### 📍 py Web Server

* `alert.py`: Stores metadata, triggers push notifications
* `get_image.py`: Retrieves image via on-demand LoRa command relay
* TensorFlow Lite model runs AI detection (e.g., Elephant detection)
* Sends image + AI result to mobile app

---

## 🔁 Workflow Diagram (Mermaid)

### 🌿 Detection Workflow

```mermaid
sequenceDiagram
    participant PIR as PIR Sensor
    participant ESP as ESP32-CAM
    participant LoRa as LoRa Radio
    participant GW as Gateway Node
    participant NET as Internet
    participant SRV as py Server
    participant APP as Mobile App

    PIR->>ESP: Motion Detected (Interrupt)
    ESP->>ESP: Wake Up from Deep Sleep
    ESP->>ESP: Capture Image & Save to SD
    ESP->>LoRa: Send Metadata Packet
    LoRa->>GW: LoRa Packet Received
    GW->>NET: HTTP POST /alert.py
    NET->>SRV: Process Alert
    SRV->>APP: Push Notification: "Motion Detected"
```

### 📸 On-Demand Image Retrieval

```mermaid
sequenceDiagram
    participant APP as Mobile App
    participant SRV as py Server
    participant GW as Gateway Node
    participant NODE as Sensor Node
    participant NET as Internet

    APP->>SRV: Request Image IMG_0451.JPG
    SRV->>GW: Send Command via API
    GW->>NODE: LoRa Command: Upload IMG_0451.JPG
    NODE->>GW: Send Image Chunks via LoRa
    GW->>SRV: Upload Full Image via Wi-Fi
    SRV->>SRV: Run AI (TFLite)
    SRV->>APP: Final Notification + Image + AI Result
```

---

## 🔌 Can It Connect from Deep Jungle?

Yes, **LoRa @433MHz** allows **line-of-sight communication up to 10+ km**, especially in rural or jungle terrains with low interference. However:

* **Nodes far from the Gateway** should **relay** their message via intermediate nodes (multi-hop if needed).
* You can implement **LoRa Mesh-like** behavior (basic repeaters) using Time Division or Address-based forwarding.
* The **Gateway** is the only component requiring **internet access**, typically placed at the edge (ranger station, tree top, or watchtower) using:

  * Smartphone hotspot (Wi-Fi)
  * SIM7600 4G module

---

## 🧠 Local AI on Gateway (TFLite on Android)

> Yes, the **smartphone providing Wi-Fi hotspot** can **also run TensorFlow Lite model** to classify elephant images **before uploading to the server**.

This has advantages:

* Reduce bandwidth (upload only confirmed images)
* Lower server load
* Quicker confirmation

You can:

* Build a background Android service (e.g., Kotlin app with Firebase + TFLite)
* Use an auto-upload folder (where the ESP uploads images to the phone via FTP or shared folder)
* Run `tflite.Interpreter` in Android app for inference

---

## 🔧 Example Payload

### LoRa Alert Packet (from Sensor Node):

```json
{
  "node_id": "Node_12",
  "event": "Motion",
  "file": "IMG_0451.JPG",
  "battery": 3.85
}
```

### HTTP POST to Server:

```http
POST /api/alert.py
Content-Type: application/json

{
  "node_id": "Node_12",
  "file": "IMG_0451.JPG",
  "timestamp": "2025-07-09T20:00:00",
  "battery": 3.85
}
```

---

## 🔐 Security Recommendations

* Use HTTPS on your server
* Include `auth_token` in API requests from Gateway
* Monitor for spoofed LoRa packets (add signature or CRC)

---

## 🧪 Testing Recommendations

1. Simulate PIR event manually
2. Check LoRa transmission from Node to Gateway
3. Validate gateway to server HTTP request
4. Push test payloads to Firebase
5. Test TFLite inference on Android

---

## 📁 Directory Structure

```
/firmware
  └── sensor_node.ino
  └── gateway_node.ino
/server

/app
