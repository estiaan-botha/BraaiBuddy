#include "LocalOTA.h"
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <SPIFFS.h>
#include <Update.h>
#include <esp_task_wdt.h>
#define U_PART U_SPIFFS

size_t content_len;
AsyncWebServer OTAserver(100);

void handleRoot(AsyncWebServerRequest *request)
{
    request->redirect("/update");
}

void handleUpdate(AsyncWebServerRequest *request)
{
    request->send(SPIFFS, "/OTA.html", "text/html");
}

void handleDoUpdate(AsyncWebServerRequest *request, const String &filename, size_t index, uint8_t *data, size_t len, bool final)
{
    if (!index)
    {
        Serial.println("Update");
        content_len = request->contentLength();
        // if filename includes spiffs, update the spiffs partition
        int cmd = (filename.indexOf("SPIFFS") > -1) ? U_PART : U_FLASH;
        int FWpos = filename.indexOf("FW");
        Serial.print("CMD value: ");
        Serial.println(cmd);
        Serial.print("FWPos value: ");
        Serial.println(FWpos);
        if ((cmd != 100) & (FWpos != 10))
        {
            Serial.println("Incorrect file for update, restarting...");
            request->send(200, "text/plain", "Incorrect file for upload, restarting ...");
            delay(3000);
            ESP.restart();
        }
#ifdef ESP8266
        Update.runAsync(true);
        if (!Update.begin(content_len, cmd))
        {
#else
        if (!Update.begin(UPDATE_SIZE_UNKNOWN, cmd))
        {
#endif
            Update.printError(Serial);
        }
    }

    if (Update.write(data, len) != len)
    {
        Update.printError(Serial);
#ifdef ESP8266
    }
    else
    {
        Serial.printf("Progress: %d%%\n", (Update.progress() * 100) / Update.size());
#endif
    }

    if (final)
    {
        if (!Update.end(true))
        {
            Update.printError(Serial);
        }
        else
        {
            Serial.println("Update complete");
            request->send(200, "text/plain", "Update completed sucessfully, restarting now...");
            delay(5000);
            Serial.flush();
            ESP.restart();
        }
    }
}

void printProgress(size_t prg, size_t sz)
{
    Serial.printf("Progress: %d%%\n", (prg * 100) / content_len);
}

void init_local_ota()
{
    esp_task_wdt_deinit();
    OTAserver.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
                 { request->redirect("/update"); });
    OTAserver.on("/update", HTTP_GET, [](AsyncWebServerRequest *request)
                 { handleUpdate(request); });
    OTAserver.on("/doUpdate", HTTP_POST, [](AsyncWebServerRequest *request) {}, [](AsyncWebServerRequest *request, const String &filename, size_t index, uint8_t *data, size_t len, bool final)
                 { handleDoUpdate(request, filename, index, data, len, final); });
    OTAserver.onNotFound([](AsyncWebServerRequest *request)
                         { request->send(404); });
    OTAserver.begin();
#ifdef ESP32
    Update.onProgress(printProgress);
#endif
    MDNS.addService("http", "tcp", 100);
}

void local_ota_loop()
{
    while (1)
    {
        delay(5);
        esp_task_wdt_reset();
    }
}
