#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <Arduino_MQTT_Client.h>
#include <Server_Side_RPC.h>
#include <Attribute_Request.h>
#include <Shared_Attribute_Update.h>
#include <ThingsBoard.h>
#include <SimpleDHT.h>
#include <PZEM004Tv30.h>
#include <HardwareSerial.h>
#include <OTA_Firmware_Update.h>
#include <Espressif_Updater.h>

// Cấu hình kết nối
#define ENCRYPTED false
constexpr char CURRENT_FIRMWARE_TITLE[] = "DA_TKLL";
constexpr char CURRENT_FIRMWARE_VERSION[] = "2.0.0";

// Cấu hình OTA
constexpr uint8_t FIRMWARE_FAILURE_RETRIES = 5U; // Giảm từ 12 xuống 5 để tránh lãng phí bộ nhớ
constexpr uint16_t FIRMWARE_PACKET_SIZE = 4096U; // Giảm từ 4096 xuống 2048 để giảm áp lực stack

// GPIO pin definitions
#define DHT_PIN GPIO_NUM_6
#define LIGHT_PIN 18
#define FAN_PIN GPIO_NUM_47
#define LIGHT_SENSOR_PIN GPIO_NUM_4
#define RXD2 38
#define TXD2 21
#define PZEM_SERIAL Serial2

// WiFi và ThingsBoard credentials
const char *ssid = "Bonjour";
const char *password = "hellosine";
const char *TOKEN = "22o1nfbfv24b9x1oq4ie";
const char *THINGSBOARD_SERVER = "app.coreiot.io";
// const char *THINGSBOARD_SERVER = "thingsboard.cloud";
// const char *Client_ID = "DA_TKLL";
// const char *User_Name = "DA_TKLL";
// const char *Password = "123456789";
// const char *Device_id = "ff1ebc00-e693-11ef-87b5-21bccf7d29d5";

#if ENCRYPTED
constexpr uint16_t THINGSBOARD_PORT = 8883U;
#else
constexpr uint16_t THINGSBOARD_PORT = 1883U;
#endif

// Các thông số cấu hình
constexpr uint16_t MAX_MESSAGE_SIZE = 256U; // Giảm từ 512 xuống 256
constexpr size_t MAX_ATTRIBUTES = 7U;
constexpr size_t MAX_ATTRIBUTES_REQUEST = 5U;                    // Giảm từ 3 xuống 2
constexpr uint64_t REQUEST_TIMEOUT_MICROSECONDS = 3000U * 1000U; // Giảm từ 5000 xuống 3000
constexpr int16_t TELEMETRY_SEND_INTERVAL = 10000U;              // Tăng từ 5000 lên 10000 để giảm tần suất gửi dữ liệu
constexpr uint32_t OTA_CHECK_INTERVAL = 60000U;                  // Tăng từ 30000 lên 60000
constexpr uint32_t AUTOLIGHT_CHECK_INTERVAL = 15000U;            // Tăng từ 5000 lên 10000 để giảm tần suất gửi dữ liệu
constexpr uint32_t SCHEDULE_CHECK_INTERVAL = 15000U;                // Tăng từ 5000 lên 10000 để giảm tần suất gửi dữ liệu
constexpr uint32_t SENSOR_READ_INTERVAL = 5000U;                 // Xác định rõ khoảng thời gian đọc cảm biến
constexpr uint32_t LIGHT_AUTO_INTERVAL = 2000U;                  // Tăng từ 10000 lên 30000 để giảm tần suất gửi dữ liệu
constexpr uint32_t WIFI_RECONNECT_DELAY = 5000U;                 // Thời gian chờ kết nối lại WiFi
constexpr uint32_t TB_RECONNECT_DELAY = 5000U;                   // Thời gian chờ kết nối lại ThingsBoard
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

#if ENCRYPTED
constexpr char ROOT_CERT[] = R"(-----BEGIN CERTIFICATE-----
MIIFIzCCBAugAwIBAgISBUc/e8RVqNedBh84txQ1YAkeMA0GCSqGSIb3DQEBCwUA
MDMxCzAJBgNVBAYTAlVTMRYwFAYDVQQKEw1MZXQncyBFbmNyeXB0MQwwCgYDVQQD
EwNSMTEwHhcNMjUwNDA0MDYwMTAxWhcNMjUwNzAzMDYwMTAwWjAcMRowGAYDVQQD
ExF0aGluZ3Nib2FyZC5jbG91ZDCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoC
ggEBAJoxHcveQBtaeS+KbgNjl2+TzxRp/3rYmRYWJ82MlcdXY42+MzB0wF+GJLR3
fMI1h6tzMDB/Y2UcVL4glCLaXAmDhzi7y/eJe8XS6Iozhz9Ww96+SjHxE5vrrmQk
V3zG9pp60ah7G+CvhAR0o4rvg5UKKP7lFNJU44fhaB4m/FfXEnQ/hCbrDWf6qrwc
rZK1P0MW4z3hhW+ZSmtxVAbdWSmqHQN4EkaWGZEd3ZU9xUswd96daicf1BpRYel4
aTpcfhbAjTWQvf+LonbbUhmd2QhwQ38xaWPtUtcNPXMVHSNJ6BQp/n6VipNkrmU7
2WrALRFUb5jYxmNmvT9LnnfNzcUCAwEAAaOCAkYwggJCMA4GA1UdDwEB/wQEAwIF
oDAdBgNVHSUEFjAUBggrBgEFBQcDAQYIKwYBBQUHAwIwDAYDVR0TAQH/BAIwADAd
BgNVHQ4EFgQUbFMcbSwAuwIexuiogNEguS305FkwHwYDVR0jBBgwFoAUxc9GpOr0
w8B6bJXELbBeki8m47kwVwYIKwYBBQUHAQEESzBJMCIGCCsGAQUFBzABhhZodHRw
Oi8vcjExLm8ubGVuY3Iub3JnMCMGCCsGAQUFBzAChhdodHRwOi8vcjExLmkubGVu
Y3Iub3JnLzAcBgNVHREEFTATghF0aGluZ3Nib2FyZC5jbG91ZDATBgNVHSAEDDAK
MAgGBmeBDAECATAuBgNVHR8EJzAlMCOgIaAfhh1odHRwOi8vcjExLmMubGVuY3Iu
b3JnLzMxLmNybDCCAQUGCisGAQQB1nkCBAIEgfYEgfMA8QB2AMz7D2qFcQll/pWb
U87psnwi6YVcDZeNtql+VMD+TA2wAAABlf+Y/qIAAAQDAEcwRQIgQgJDd+WdGFy+
qIJ1LE4sFB98ncU9FNiS0o5MEOdWatMCIQCQZKkhmLJqgJLOpLyRqIrx7q1vV038
NSA6lQDFRjQrrgB3AN3cyjSV1+EWBeeVMvrHn/g9HFDf2wA6FBJ2Ciysu8gqAAAB
lf+Y/osAAAQDAEgwRgIhAMZ57uTJQqx5i/P1cxz6VX39ygT0CV/B4QFQPUAT7oVf
AiEAiOYLQaY+Cg+fnCBRtFx+UH4q2TKe3p6YRI5poNBu8mkwDQYJKoZIhvcNAQEL
BQADggEBALEWATKmmnDzEfMs8vjrGuSIkzV48ESv56qfCs76C+tH70HVn14MkeAU
lm6kyPZVYAiXK0g1OxjpxZAiLcdBE/2tVl8HNWUCcHyz2vHa9dwVgXx5qUE7j9HH
VpuQzfK/T7preDgx241ts46H/NlpI4ne0YNyQfajq3oQETVElH3b52Ccp0QuK937
OP2ERR7/FQC8fqfxpuVpio9c+8iCzUOUyKfCGhtOBClAC35CI1f+zv1ECmfTja9K
GdXYRZF0lUVUws03dE530AOt9Z951SvXACwbPsqo5XZwANkSrOs0lKuFGMrHVGpc
/1Rp/v2NOIaXdRFre4wg9RXjltl/8TY=
-----END CERTIFICATE-----
)";
#endif

// Khởi tạo clients
#if ENCRYPTED
WiFiClientSecure espClient;
#else
WiFiClient espClient;
#endif
Arduino_MQTT_Client mqttClient(espClient);

// Khởi tạo các API
Server_Side_RPC<2U, 3U> rpc; // Giảm từ 3U, 5U xuống 2U, 3U để tiết kiệm bộ nhớ
Attribute_Request<2U, MAX_ATTRIBUTES_REQUEST> attr_request;
Shared_Attribute_Update<2U, MAX_ATTRIBUTES> shared_update; // Giảm từ 3U xuống 2U
OTA_Firmware_Update<> ota;

const std::array<IAPI_Implementation *, 4U> apis = {
    &rpc,
    &attr_request,
    &shared_update,
    &ota};

ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE, MAX_MESSAGE_SIZE, Default_Max_Stack_Size, apis);
Espressif_Updater<> updater;

// Khởi tạo cảm biến
SimpleDHT11 dht(DHT_PIN);
PZEM004Tv30 pzem(PZEM_SERIAL, RXD2, TXD2);

// RTOS resources
QueueHandle_t sensorQueue = NULL;
SemaphoreHandle_t serialMutex = NULL;
SemaphoreHandle_t tbMutex = NULL; // Thêm mutex cho ThingsBoard
TaskHandle_t ThingsBoard_Task_Handle = NULL;
TaskHandle_t Sensor_Task_Handle = NULL;
TaskHandle_t Auto_Light_Handle = NULL;
// Status flags
bool currentFWSent = false;
bool updateRequestSent = false;
bool isUpdatingOTA = false;
bool isConnectingTB = false;
bool wifiConnected = false;
bool tbConnected = false;

// Device state
volatile bool fanState = false;
volatile bool lightState = false;
volatile bool lightAuto = false;
volatile float LIGHT_AUTO_THRESHOLD = 30.0f; // Giảm từ 50.0f xuống 30.0f
volatile uint64_t startTime = 0.0f;
volatile uint64_t endTime = 0.0f;
volatile bool daily = 0.0f;
// Attribute names
constexpr const char Fan_STATE_ATTR[] = "sharedvalueFan";
constexpr const char Light_STATE_ATTR[] = "sharedvalueLight";
constexpr const char Light_AUTO[] = "Light_auto";
constexpr const char Brightness_THRESHOLD[] = "Threshold_brightness";
constexpr const char Start_TIME[] = "Start";
constexpr const char End_TIME[] = "End";
constexpr const char Daily[] = "Daily";
// Danh sách shared attributes để subscribe và request
constexpr std::array<const char *, MAX_ATTRIBUTES> SHARED_ATTRIBUTES_LIST = {
    Fan_STATE_ATTR,
    Light_STATE_ATTR};
constexpr std::array<const char *, MAX_ATTRIBUTES_REQUEST> REQUEST_ATTRIBUTES_LIST = {
    Light_AUTO,
    Brightness_THRESHOLD,
    Start_TIME,
    End_TIME,
    Daily};

// Cấu trúc dữ liệu cảm biến
struct SensorData
{
  float temperature;
  float humidity;
  float brightness;
  float voltage;
  float current;
  float power;
  float energy;
  bool valid; // Thêm trường để kiểm tra dữ liệu hợp lệ
};

// Biến toàn cục để lưu dữ liệu cảm biến gần nhất
SensorData lastSensorData = {0, 0, 0, 0, 0, 0, 0, false};

// Safe serial print function
void safeSerialPrintln(const char *message)
{
  if (serialMutex != NULL && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(500)) == pdTRUE)
  {
    Serial.println(message);
    xSemaphoreGive(serialMutex);
  }
}

void safeSerialPrint(const char *message)
{
  if (serialMutex != NULL && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(500)) == pdTRUE)
  {
    Serial.print(message);
    xSemaphoreGive(serialMutex);
  }
}

void safeSerialPrintf(const char *format, ...)
{
  if (serialMutex != NULL && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(500)) == pdTRUE)
  {
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    // Serial.println();
    xSemaphoreGive(serialMutex);
  }
}

// RPC callback cho setValueFan
void processSetValueFan(const JsonVariantConst &data, JsonDocument &response)
{
  fanState = data.as<bool>();
  digitalWrite(FAN_PIN, fanState ? HIGH : LOW);

  safeSerialPrintf("Received RPC setValueFan: %d\n", fanState ? 1 : 0);

  tb.sendAttributeData("sharedvalueFan", fanState);

  StaticJsonDocument<32> response_doc;
  response_doc["newFanState"] = fanState;
  response.set(response_doc);
}

// RPC callback cho setValueLight
void processSetValueLight(const JsonVariantConst &data, JsonDocument &response)
{
  lightState = data.as<bool>();
  digitalWrite(LIGHT_PIN, lightState ? LOW : HIGH);

  safeSerialPrintf("Received RPC setValueLight: %d\n", lightState ? 1 : 0);

  tb.sendAttributeData("sharedvalueLight", lightState);

  StaticJsonDocument<32> response_doc;
  response_doc["newLightState"] = lightState;
  response.set(response_doc);
}

// Đăng ký RPC callbacks
const std::array<RPC_Callback, 2U> rpcCallbacks = {
    RPC_Callback{"setValueFan", processSetValueFan},
    RPC_Callback{"setValueLight", processSetValueLight}};

// Callback xử lý shared attributes
void processSharedAttributes(const JsonObjectConst &data)
{
  if (data.isNull())
  {
    safeSerialPrintln("Received null shared attributes data");
    return;
  }

  for (auto it = data.begin(); it != data.end(); ++it)
  {
    const char *key = it->key().c_str();

    safeSerialPrintf("Received attribute: %s\n", key);

    if (strcmp(key, "sharedvalueFan") == 0 && it->value().is<bool>())
    {
      fanState = it->value().as<bool>();
      digitalWrite(FAN_PIN, fanState ? HIGH : LOW);
      safeSerialPrintf("Updated fan state: %d\n", fanState ? 1 : 0);
    }
    else if (strcmp(key, "sharedvalueLight") == 0 && it->value().is<bool>())
    {
      lightState = it->value().as<bool>();
      digitalWrite(LIGHT_PIN, lightState ? LOW : HIGH);
      safeSerialPrintf("Updated light state: %d\n", lightState ? 1 : 0);
    }
    else if (strcmp(key, "Light_auto") == 0 && it->value().is<bool>())
    {
      lightAuto = it->value().as<bool>();
      safeSerialPrintf("Received Light_auto: %d\n", lightAuto ? 1 : 0);
    }
    else if (strcmp(key, "Threshold_brightness") == 0 && it->value().is<float>())
    {
      LIGHT_AUTO_THRESHOLD = it->value().as<float>();
      safeSerialPrintf("Received Threshold_brightness: %f\n", LIGHT_AUTO_THRESHOLD);
    }
    else if  (strcmp(key, "Start") == 0 && it ->value().is<uint64_t>())
    {
      startTime = it->value().as<uint64_t>();
      safeSerialPrintf("Received Start: %f\n", startTime);
    }
    else if (strcmp(key, "End") == 0 && it->value().is<uint64_t>())
    {
      endTime = it->value().as<uint64_t>();
      safeSerialPrintf("Received End: %f\n", endTime);
    }
    else if (strcmp(key, "Daily") == 0 && it->value().is<bool>())
    {
      daily = it->value().as<bool>();
      safeSerialPrintf("Received Daily: %d\n", daily ? 1 : 0);
    }
    else
    {
      safeSerialPrintf("Unknown attribute: %s\n", key);
    }
  }
  isConnectingTB = true;
}

void requestTimedOut()
{
  safeSerialPrintf("Attribute request timed out after %llu ms\n", REQUEST_TIMEOUT_MICROSECONDS / 1000);
}

const Shared_Attribute_Callback<MAX_ATTRIBUTES> attributes_callback(
    &processSharedAttributes,
    SHARED_ATTRIBUTES_LIST.cbegin(),
    SHARED_ATTRIBUTES_LIST.cend());

const Attribute_Request_Callback<MAX_ATTRIBUTES_REQUEST> attribute_shared_request_callback(
    &processSharedAttributes,
    REQUEST_TIMEOUT_MICROSECONDS,
    &requestTimedOut,
    REQUEST_ATTRIBUTES_LIST);

// WiFi connection management
bool connectToWiFi()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    if (!wifiConnected)
    {
      safeSerialPrintln("WiFi already connected");
      wifiConnected = true;
    }
    return true;
  }

  // Tạo kết nối mới
  safeSerialPrintln("Connecting to WiFi...");

  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Đợi kết nối với timeout
  uint8_t retryCount = 0;
  const uint8_t maxRetries = 10;

  while (WiFi.status() != WL_CONNECTED && retryCount < maxRetries)
  {
    if (retryCount % 3 == 0)
    {
      safeSerialPrint(".");
    }
    vTaskDelay(pdMS_TO_TICKS(500));
    retryCount++;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    wifiConnected = true;
    safeSerialPrintf("WiFi connected - IP: %s\n", WiFi.localIP().toString().c_str());
    return true;
  }
  else
  {
    wifiConnected = false;
    safeSerialPrintln("WiFi connection failed");
    return false;
  }
}

bool connectToThingsBoard()
{
  if (tb.connected())
  {
    if (!tbConnected)
    {
      safeSerialPrintln("ThingsBoard already connected");
      tbConnected = true;
    }
    return true;
  }

  if (!wifiConnected)
  {
    safeSerialPrintln("WiFi not connected, can't connect to ThingsBoard");
    return false;
  }

  safeSerialPrintln("Connecting to ThingsBoard...");

  // Acquire the ThingsBoard mutex before connecting
  if (tbMutex != NULL && xSemaphoreTake(tbMutex, pdMS_TO_TICKS(1000)) == pdTRUE)
  {
    bool connected = tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT);
    xSemaphoreGive(tbMutex);

    if (!connected)
    {
      safeSerialPrintln("ThingsBoard connect failed");
      tbConnected = false;
      return false;
    }
  }
  else
  {
    safeSerialPrintln("Failed to acquire ThingsBoard mutex");
    return false;
  }

  // Subscribe to RPC and shared attributes
  if (!isConnectingTB)
  {
    safeSerialPrintln("Subscribing to RPC and shared attributes...");

    if (tbMutex != NULL && xSemaphoreTake(tbMutex, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
      bool rpcSuccess = rpc.RPC_Subscribe(rpcCallbacks.cbegin(), rpcCallbacks.cend());
      bool sharedSuccess = shared_update.Shared_Attributes_Subscribe(attributes_callback);
      // bool requestSuccess = attr_request.Shared_Attributes_Request(attribute_shared_request_callback);
      xSemaphoreGive(tbMutex);

      if (!rpcSuccess)
      {
        safeSerialPrintln("Failed to subscribe for RPC");
        return false;
      }
      if (!sharedSuccess)
      {
        safeSerialPrintln("Failed to subscribe for shared attributes");
        return false;
      }
      // if (!requestSuccess) {
      //   safeSerialPrintln("Failed to request shared attributes");
      //   return false;
      // }

      safeSerialPrintln("Successfully subscribed to ThingsBoard services");
    }
    else
    {
      safeSerialPrintln("Failed to acquire ThingsBoard mutex for subscriptions");
      return false;
    }
  }

  tbConnected = true;
  return true;
}

// OTA update callbacks
void update_starting_callback()
{
  safeSerialPrintln("Update starting, stopping tasks...");

  // Dừng task đọc cảm biến
  if (Sensor_Task_Handle != NULL)
  {
    vTaskSuspend(Sensor_Task_Handle);
    vTaskSuspend(Auto_Light_Handle);
  }

  // Hủy đăng ký RPC để tránh nhận RPC call trong quá trình OTA
  if (tbMutex != NULL && xSemaphoreTake(tbMutex, pdMS_TO_TICKS(1000)) == pdTRUE)
  {
    rpc.RPC_Unsubscribe();
    safeSerialPrintln("RPC unsubscribed for OTA update");
    xSemaphoreGive(tbMutex);
  }
  else
  {
    safeSerialPrintln("Failed to acquire ThingsBoard mutex for RPC unsubscribe");
  }

  isUpdatingOTA = true;
  safeSerialPrintln("Tasks suspended for OTA update");
}

void finished_callback(const bool &success)
{
  if (success)
  {
    safeSerialPrintln("OTA update successful, rebooting in 3 seconds...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    esp_restart();
  }
  else
  {
    safeSerialPrintln("OTA update failed, resuming tasks...");

    // Khôi phục task đọc cảm biến
    if (Sensor_Task_Handle != NULL)
    {
      vTaskResume(Sensor_Task_Handle);
    }

    // Khôi phục tự động bật/tắt đèn
    if (Auto_Light_Handle != NULL)
    {
      vTaskResume(Auto_Light_Handle);
    }

    // Đăng ký lại RPC nếu cập nhật OTA thất bại
    if (tbMutex != NULL && xSemaphoreTake(tbMutex, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
      bool rpcSuccess = rpc.RPC_Subscribe(rpcCallbacks.cbegin(), rpcCallbacks.cend());
      if (rpcSuccess)
      {
        safeSerialPrintln("RPC resubscribed after failed OTA update");
      }
      else
      {
        safeSerialPrintln("Failed to resubscribe RPC after failed OTA update");
      }
      xSemaphoreGive(tbMutex);
    }
    else
    {
      safeSerialPrintln("Failed to acquire ThingsBoard mutex for RPC resubscribe");
    }

    isUpdatingOTA = false;
  }
}

void progress_callback(const size_t &current, const size_t &total)
{
  static uint8_t lastPercentage = 0;
  uint8_t percentage = static_cast<uint8_t>(current * 100U / total);

  // Chỉ in ra khi phần trăm thay đổi đáng kể để giảm log
  if (percentage - lastPercentage >= 5)
  {
    safeSerialPrintf("OTA Progress: %u%%\n", percentage);
    lastPercentage = percentage;
  }
}

// Kiểm tra dữ liệu cảm biến hợp lệ
bool isValidSensorData(const SensorData &data)
{
  return !isnan(data.temperature) &&
         !isnan(data.humidity) &&
         !isnan(data.brightness) &&
         !isnan(data.voltage) &&
         !isnan(data.current) &&
         !isnan(data.power) &&
         !isnan(data.energy) &&
         data.temperature >= 0 && data.temperature <= 50 &&
         data.humidity >= 0 && data.humidity <= 100 &&
         data.brightness >= 0 && data.brightness <= 100 &&
         data.voltage >= 0 && data.voltage <= 250 &&
         data.current >= 0 && data.current <= 100 &&
         data.power >= 0 && data.power <= 10000 &&
         data.energy >= 0;
}

// Kiểm tra và gửi dữ liệu thất bại
void handleFailedSensorRead(int dht_err, float voltage, float current, float power, float energy)
{
  if (dht_err != SimpleDHTErrSuccess)
  {
    safeSerialPrintf("DHT11 read failed, error code: %d\n", dht_err);
  }
  if (voltage < 0)
  {
    safeSerialPrintln("Voltage read failed");
  }
  if (current < 0)
  {
    safeSerialPrintln("Current read failed");
  }
  if (power < 0)
  {
    safeSerialPrintln("Power read failed");
  }
  if (energy < 0)
  {
    safeSerialPrintln("Energy read failed");
  }
}

// Task đọc dữ liệu cảm biến
void Sensor_Task(void *pvParameters)
{
  // Đặt độ phân giải ADC cao hơn cho độ chính xác
  analogReadResolution(12);

  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (true)
  {
    float temperature = 0;
    float humidity = 0;

    // Đọc dữ liệu từ cảm biến DHT11
    int dht_err = dht.read2(&temperature, &humidity, NULL);

    // Đọc dữ liệu từ cảm biến quang trở (ADC);
    int raw_value = analogRead(LIGHT_SENSOR_PIN);
    float brightness = (float)raw_value / 4095.0 * 100; // Chuyển đổi giá trị ADC sang % (0-3.3V)

    // Đọc dữ liệu từ PZEM004T
    float voltage = pzem.voltage();
    float current = pzem.current();
    float power = pzem.power();
    float energy = pzem.energy();
    //(dht_err == SimpleDHTErrSuccess && !isnan(temperature) && !isnan(humidity) && raw_value >= 0 && voltage >= 0 && current >= 0 && power >= 0 && energy >= 0)
    if (dht_err == SimpleDHTErrSuccess && !isnan(temperature) && !isnan(humidity))
    {
      safeSerialPrintf("Read Sensors -> Temp: %.2f°C, Humi: %.2f%%, Light: %.2f%%, Voltage: %.2fV, Current: %.2fA, Power: %.2fW, Energy: %.2fWh\n",
                       temperature, humidity, brightness, voltage, current, power, energy);
      SensorData data = {temperature, humidity, brightness, voltage, current, power, energy};

      // Gửi dữ liệu vào hàng đợi
      if (sensorQueue && xQueueSend(sensorQueue, &data, pdMS_TO_TICKS(1000)) != pdTRUE)
      {
        safeSerialPrintln("Queue full! Overwriting oldest data...");
        SensorData dummy;
        xQueueReceive(sensorQueue, &dummy, 0);
        xQueueSend(sensorQueue, &data, pdMS_TO_TICKS(500));
      }
    }
    else
    {
      if (serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(1000)) == pdTRUE)
      {
        safeSerialPrintln("Failed to read sensor data");
        handleFailedSensorRead(dht_err, voltage, current, power, energy);
        vTaskDelay(pdMS_TO_TICKS(100));
        xSemaphoreGive(serialMutex);
      }
    }
    // Sử dụng vTaskDelayUntil để đảm bảo thời gian chính xác giữa các lần đọc
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SENSOR_READ_INTERVAL));
  }
}

// Task giao tiếp với ThingsBoard
void ThingsBoard_Task(void *pvParameters)
{
  uint32_t previousTelemetrySend = 0;
  uint32_t previousOTAcheck = 0;
  uint32_t previousSchedulecheck = 0;
  while (true)
  {
    // Kiểm tra và kết nối WiFi
    if (!connectToWiFi())
    {
      vTaskDelay(pdMS_TO_TICKS(WIFI_RECONNECT_DELAY));
      continue;
    }

    // Kết nối đến ThingsBoard nếu chưa kết nối
    if (!tb.connected())
    {
      if (!connectToThingsBoard())
      {
        vTaskDelay(pdMS_TO_TICKS(TB_RECONNECT_DELAY));
        continue;
      }
    }

    // Xử lý dữ liệu từ queue
    SensorData data;
    bool newDataReceived = false;

    if (sensorQueue != NULL && xQueueReceive(sensorQueue, &data, 0) == pdTRUE)
    {
      newDataReceived = true;
    }
    else if (lastSensorData.valid)
    {
      // Nếu không có dữ liệu mới, sử dụng dữ liệu cuối cùng
      data = lastSensorData;
      newDataReceived = true;
    }

    // Gửi telemetry nếu có dữ liệu mới và đủ thời gian
    if (newDataReceived && !isUpdatingOTA && tb.connected() &&
        (millis() - previousTelemetrySend > TELEMETRY_SEND_INTERVAL))
    {

      safeSerialPrintln("Sending telemetry data to ThingsBoard...");

      // Gửi dữ liệu với mutex bảo vệ
      if (tbMutex != NULL && xSemaphoreTake(tbMutex, pdMS_TO_TICKS(1000)) == pdTRUE)
      {
        tb.sendTelemetryData("temperature", data.temperature);
        tb.sendTelemetryData("humidity", data.humidity);
        tb.sendTelemetryData("brightness", data.brightness);
        tb.sendTelemetryData("voltage", data.voltage);
        tb.sendTelemetryData("current", data.current);
        tb.sendTelemetryData("power", data.power);
        tb.sendTelemetryData("energy", data.energy);
        xSemaphoreGive(tbMutex);

        safeSerialPrintln("Telemetry data sent successfully");
        previousTelemetrySend = millis();
      }
      else
      {
        safeSerialPrintln("Failed to acquire ThingsBoard mutex for telemetry");
      }
    }

    // Kiểm tra OTA updates định kỳ
    if (millis() - previousOTAcheck > OTA_CHECK_INTERVAL && !isUpdatingOTA && tb.connected())
    {
      safeSerialPrintln("Checking for OTA updates...");

      // Reset các flags
      currentFWSent = false;
      updateRequestSent = false;

      // Gửi thông tin firmware hiện tại với mutex bảo vệ
      if (tbMutex != NULL && xSemaphoreTake(tbMutex, pdMS_TO_TICKS(1000)) == pdTRUE)
      {
        if (!currentFWSent)
        {
          currentFWSent = ota.Firmware_Send_Info(CURRENT_FIRMWARE_TITLE, CURRENT_FIRMWARE_VERSION);
          safeSerialPrintln("Firmware info sent to ThingsBoard");
        }

        if (!updateRequestSent)
        {
          safeSerialPrintln("Checking for firmware update...");

          const OTA_Update_Callback callback(
              CURRENT_FIRMWARE_TITLE,
              CURRENT_FIRMWARE_VERSION,
              &updater,
              &finished_callback,
              &progress_callback,
              &update_starting_callback,
              FIRMWARE_FAILURE_RETRIES,
              FIRMWARE_PACKET_SIZE);

          updateRequestSent = ota.Start_Firmware_Update(callback);

          if (updateRequestSent)
          {
            safeSerialPrintln("Firmware update request sent");
          }
          else
          {
            safeSerialPrintln("Failed to send firmware update request");
          }
        }

        xSemaphoreGive(tbMutex);
      }
      else
      {
        safeSerialPrintln("Failed to acquire ThingsBoard mutex for OTA check");
      }

      previousOTAcheck = millis();
    }

    // Xử lý các sự kiện ThingsBoard
    if (tb.connected())
    {
      if (tbMutex != NULL && xSemaphoreTake(tbMutex, pdMS_TO_TICKS(1000)) == pdTRUE)
      {
        tb.loop();
        xSemaphoreGive(tbMutex);
      }
    }
    else
    {
      // Nếu kết nối đã mất, cập nhật trạng thái và thử lại
      if (tbConnected)
      {
        safeSerialPrintln("ThingsBoard connection lost, will reconnect...");
        tbConnected = false;
      }

      // Ngắt kết nối hiện tại để chuẩn bị kết nối lại
      if (tbMutex != NULL && xSemaphoreTake(tbMutex, pdMS_TO_TICKS(1000)) == pdTRUE)
      {
        tb.disconnect();
        xSemaphoreGive(tbMutex);
      }
    }
    if (millis() - previousSchedulecheck > AUTOLIGHT_CHECK_INTERVAL)
    {
      previousSchedulecheck = millis();
      bool requestSuccess = attr_request.Shared_Attributes_Request(attribute_shared_request_callback);
    }
    // Ngủ ngắn để nhả CPU cho các tác vụ khác
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void Auto_Light(void *pvParameters)
{
  // Đặt độ phân giải ADC cao hơn cho độ chính xác
  analogReadResolution(12);
  uint32_t lastLightCheck = 0;
  bool below_threshold = false;
  float light_brightness = 0.0f;
  // Đợi một chút trước khi bắt đầu
  vTaskDelay(pdMS_TO_TICKS(2000));
  while (true)
  {
    // Kiểm tra trạng thái tự động bật/tắt đèn
    if (lightAuto && millis() - lastLightCheck >= LIGHT_AUTO_INTERVAL)
    {
      // Nếu tự động, kiểm tra độ sáng và điều chỉnh đèn
      int lightLevel = analogRead(LIGHT_SENSOR_PIN);
      float brightness = (float)lightLevel / 4095.0 * 100 - light_brightness; // Chuyển đổi giá trị ADC sang % (0-3.3V)
      safeSerialPrintf("Light level: %.2f%%\n", brightness);
      if (brightness < LIGHT_AUTO_THRESHOLD)
      {
        // Nếu độ sáng thấp hơn ngưỡng, bật đèn
        if (!lightState)
        {
          lightState = true;
          digitalWrite(LIGHT_PIN, LOW);
          safeSerialPrintln("Light turned ON");
          if (tbMutex != NULL && xSemaphoreTake(tbMutex, pdMS_TO_TICKS(1000)) == pdTRUE)
          {
            tb.sendAttributeData("sharedvalueLight", lightState);
            safeSerialPrintln("Light state sent to ThingsBoard");
            xSemaphoreGive(tbMutex);
          }
          else
          {
            safeSerialPrintln("Failed to acquire ThingsBoard mutex for light state");
          }
        }
        int lightLevel = analogRead(LIGHT_SENSOR_PIN);
        light_brightness = (float)lightLevel / 4095.0 * 100 - brightness; // Chuyển đổi giá trị ADC sang % (0-3.3V)
        vTaskDelay(pdMS_TO_TICKS(5000));                                  // Đợi 5 giây để tránh nhấp nháy đèn
      }
      else
      {
        // Nếu độ sáng cao hơn ngưỡng, tắt đèn
        if (lightState)
        {
          lightState = false;
          light_brightness = 0.0f;
          digitalWrite(LIGHT_PIN, HIGH);
          safeSerialPrintln("Light turned OFF");
          if (tbMutex != NULL && xSemaphoreTake(tbMutex, pdMS_TO_TICKS(1000)) == pdTRUE)
          {
            tb.sendAttributeData("sharedvalueLight", lightState);
            safeSerialPrintln("Light state sent to ThingsBoard");
            xSemaphoreGive(tbMutex);
          }
          else
          {
            safeSerialPrintln("Failed to acquire ThingsBoard mutex for light state");
          }
        }
        vTaskDelay(pdMS_TO_TICKS(5000)); // Đợi 5 giây để tránh nhấp nháy đèn
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); // Thời gian chờ giữa các lần kiểm tra
  }
}

void setup()
{
  // Khởi tạo Serial cho debug
  Serial.begin(SERIAL_DEBUG_BAUD);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  // Đợi Serial ổn định
  delay(1000);
  Serial.println("\n--- ESP32-S3 IoT System Starting ---");

  // Cấu hình các pins
  pinMode(FAN_PIN, OUTPUT);
  pinMode(LIGHT_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW);
  digitalWrite(LIGHT_PIN, HIGH);

  // Khởi tạo RTOS resources
  sensorQueue = xQueueCreate(5, sizeof(SensorData)); // Giảm kích thước queue từ 10 xuống 5
  serialMutex = xSemaphoreCreateMutex();
  tbMutex = xSemaphoreCreateMutex(); // Mutex mới cho ThingsBoard

  if (!sensorQueue || !serialMutex || !tbMutex)
  {
    Serial.println("Failed to create required RTOS resources!");
    while (true)
    {
      delay(1000);
    }
  }

  // Kết nối WiFi ban đầu
  connectToWiFi();

  // Tạo task cho ThingsBoard
  xTaskCreate(Sensor_Task, "SensorTask", 4096, NULL, 1, &Sensor_Task_Handle);
  xTaskCreate(Auto_Light, "AutomationTask", 4096, NULL, 1, &Auto_Light_Handle);
  xTaskCreate(ThingsBoard_Task, "TBTask", 8192, NULL, 1, &ThingsBoard_Task_Handle);
  // Đảm bảo task được tạo thành công
  if (ThingsBoard_Task_Handle == NULL || Sensor_Task_Handle == NULL)
  {
    Serial.println("Failed to create tasks!");
    while (true)
    {
      delay(1000);
    }
  }
}
void loop()
{
  // Không cần làm gì trong loop() vì tất cả các hoạt động đã được xử lý trong các task
  delay(1000);
}