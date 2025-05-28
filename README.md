# Smart Home Control with YoloUNO (ESP32-S3)

This project utilizes PlatformIO to develop firmware for the YoloUNO board, based on the ESP32-S3 chip, for smart home control.  It connects to WiFi and communicates with ThingsBoard for IoT functionality.

## Features

* **WiFi Connectivity:** Connects to a specified WiFi network.  Includes retry logic for robust connection establishment.
* **ThingsBoard Integration:** Communicates with a ThingsBoard instance for data exchange and remote control.  Supports OTA updates.
* **Light Control:** Controls a light connected to the YoloUNO board.  Allows for local and remote control via ThingsBoard.
* **Over-the-Air (OTA) Updates:** Enables firmware updates over WiFi for easy maintenance and feature upgrades.


## Tasks Performed by YoloUNO

* **Connects to WiFi:**  Establishes a connection to the configured WiFi network.  Displays connection status and IP address via serial monitor.
* **Communicates with ThingsBoard:** Sends and receives data from ThingsBoard.  Handles MQTT communication and data serialization using ArduinoJson.
* **Controls Light:**  Turns a connected light on or off based on commands received from ThingsBoard or local control.  Maintains the light state even before a connection to ThingsBoard is established.
* **Receives OTA Updates:** Downloads and installs firmware updates from ThingsBoard when available.


## Usage

1. Install PlatformIO.
2. Clone this repository.
3. Configure the `platformio.ini` file with your WiFi credentials and ThingsBoard server details.
4. Build and upload the firmware to your YoloUNO board.


## Configuration

See `platformio.ini` for configuration options, including WiFi credentials and ThingsBoard connection details.  Refer to the [PlatformIO documentation](https://docs.platformio.org/page/platforms/espressif32.html) for more information.

## ThingsBoard Rulechain Configuration

Dự án này sử dụng hai file cấu hình rulechain cho ThingsBoard Cloud Server:

- **root_rule_chain.json**: File cấu hình cho Rule Chain gốc (Root Rule Chain), chịu trách nhiệm xử lý luồng dữ liệu chính từ thiết bị, chuyển tiếp dữ liệu và các sự kiện đến các rulechain khác hoặc xử lý logic tổng thể.
- **environmental_alert.json**: File cấu hình cho Rule Chain thông báo môi trường, dùng để phát hiện các điều kiện môi trường bất thường (ví dụ: nhiệt độ, độ ẩm vượt ngưỡng) và gửi thông báo đến người dùng.

Bạn có thể import các file này vào ThingsBoard để thiết lập tự động xử lý và cảnh báo cho hệ thống Smart Home.

## ThingsBoard Dashboard

Bạn có thể giám sát và điều khiển thiết bị thông qua ThingsBoard Dashboard tại đây:  
[Smart Home Dashboard](https://app.coreiot.io/dashboard/447e5710-e699-11ef-87b5-21bccf7d29d5?publicId=2f886da0-e3c4-11ef-ad09-515f790ed9df)