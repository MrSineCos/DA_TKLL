#include <WiFi.h>
#include "time.h"
#include "esp_sntp.h"

const char *ssid = "Bonjour";
const char *password = "hellosine";

const char *ntpServer1 = "pool.ntp.org";
const char *ntpServer2 = "time.nist.gov";
const long gmtOffset_sec = 25200;
const int daylightOffset_sec = 0;

const char *time_zone = "CET-1CEST,M3.5.0,M10.5.0/3"; // TimeZone rule for Europe/Rome including daylight adjustment rules (optional)

void printLocalTime(struct timeval *t)
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

uint64_t getTimestampMillis()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("No time available (yet) for timestamp");
    return 0;
  }

  // Chuyển đổi struct tm thành time_t (seconds từ epoch)
  time_t epochTime = mktime(&timeinfo);

  // Chuyển đổi thành milliseconds và in ra
  uint64_t epochMillis = (uint64_t)epochTime * 1000ULL;
  return epochMillis;
}

void Schedule_Task(void *pvParameters)
{
  Serial.println("Schedule Task started");
  esp_sntp_servermode_dhcp(1); // (optional)
  sntp_set_time_sync_notification_cb(printLocalTime);
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);
  Serial.println("NTP server set");
  while (true)
  {
    uint64_t currentTime = getTimestampMillis();
    Serial.printf("Current time: %llu\n", currentTime);
    vTaskDelay(pdMS_TO_TICKS(10000)); // Thời gian chờ giữa các lần kiểm tra
  }
}


void setup()
{
  Serial.begin(115200);

  // First step is to configure WiFi STA and connect in order to get the current time and date.
  Serial.printf("Connecting to %s ", ssid);
  WiFi.begin(ssid, password);

  /**
   * NTP server address could be acquired via DHCP,
   *
   * NOTE: This call should be made BEFORE esp32 acquires IP address via DHCP,
   * otherwise SNTP option 42 would be rejected by default.
   * NOTE: configTime() function call if made AFTER DHCP-client run
   * will OVERRIDE acquired NTP server address
   */

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" CONNECTED");
  xTaskCreate(Schedule_Task, "Schedule_Task", 4096, NULL, 1, NULL);
}

void loop()
{
  delay(5000);
}
