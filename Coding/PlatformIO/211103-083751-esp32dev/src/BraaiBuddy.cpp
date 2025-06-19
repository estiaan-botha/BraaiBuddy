//  includes
#include <Arduino.h>
#include <EEPROM.h>
#include <ESPAsyncWebServer.h>
#include <FS.h>
#include <HTTPClient.h>
#include <OneWire.h>
#include <SPIFFS.h>

#include <SPI.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <esp_task_wdt.h>

#include "Adafruit_MCP9600.h"
#include "LIS3DHTR.h"

// header files
#include "Globals.h"
#include "LocalOTA.h"
#include "time.h"

// SD card includes
#include <SD.h>
SPIClass SD_SPI(FSPI);
#define SD_CLK 19              // SD card clock pin
#define SD_MISO 20             // SD card MISO pin
#define SD_MOSI 18             // SD card MOSI pin
#define SD_CS 23               // SD card CS pin
#define SD_DETECT_PIN 5        // GPIO5 for SD card detect
bool last_sd_inserted = false; // Track previous state

LIS3DHTR<TwoWire> LIS; // IIC

// Define I2C pins
#define WIRE Wire
#define IIC_SDA 6 // SDA on GPIO6
#define IIC_SCL 7 // SCL on GPIO7

// I2C Addresses (MCP9600's):
#define I2C_ADDRESS_1 (0x60) // CONN1
#define I2C_ADDRESS_2 (0x62) // CONN2
#define I2C_ADDRESS_3 (0x64) // CONN3
#define I2C_ADDRESS_4 (0x66) // CONN4
#define I2C_ADDRESS_5 (0x67) // CONN5

// I2C Address (LIS3DH):
#define ACC_I2C_ADR 0x18

Adafruit_MCP9600 mcp1;
Adafruit_MCP9600 mcp2;
Adafruit_MCP9600 mcp3;
Adafruit_MCP9600 mcp4;
Adafruit_MCP9600 mcp5;

// RGB pin setup
#define LED_RED_PIN 3
#define LED_GREEN_PIN 2
#define LED_BLUE_PIN 4

// Buzzer pin Setup
#define BUZZER_PIN 14

// Battery voltage divider pin
#define BATTERY_ADC_PIN 1 // ADC1 on GPIO1

// Buzzer notes
#define NOTE_C4 262
#define NOTE_CS4 277
#define NOTE_D4 294
#define NOTE_DS4 311
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_FS4 370
#define NOTE_G4 392
#define NOTE_GS4 415
#define NOTE_A4 440
#define NOTE_AS4 466
#define NOTE_B4 494

// IP Address RGB LED handling
bool IP_address_sequence_done = false;
enum IP_address_sequence_state
{
    IP_IDLE,
    IP_NEXT_CHAR,
    IP_SET_COLOR,
    IP_BEEP_ON,
    IP_BEEP_OFF,
    IP_DOT_PAUSE,
    IP_CHAR_PAUSE,
    IP_DONE
};
static IP_address_sequence_state ip_comm_state = IP_IDLE;
static unsigned long ip_comm_timer = 0;
static int ip_comm_index = 0;
static int ip_comm_beep_count = 0;
static int ip_comm_beep_target = 0;
static int ip_comm_color_phase = 0; // 0=red, 1=green, 2=blue

// Grid Turn Time variables
int grid_turn_time = 180; // in seconds

// SD card variables
// bool csv_labels_written = false;

// Stuct setup
struct SensorData sensor_data;
struct SystemStatus system_status;
struct WebSystemState web_system_state;

/* Set and print ambient resolution */
Ambient_Resolution ambientRes = RES_ZERO_POINT_0625;

//  Wifi Constants
const char *password = "braaibuddy";
const int dns_port = 53;
const int http_port = 80;
const int ws_port = 1337;

uint8_t start_ap = 1; // 1 for True then start wifi AP
char ssid_buffer[19];
String ip_address;
char network_password[64] = ""; // Max password length is 64 characters
// char network_password[64] = "salamander";
uint8_t raw_data[512];

// Webserver and sockets
AsyncWebServer server(80);
WebSocketsServer web_socket = WebSocketsServer(1337);
char msg_buf[255];
// char network_ssid[32] = "Estiaan PC";
char network_ssid[32] = ""; // Max SSID length is 32 characters

// defines
#define TRUE 1
#define FALSE 0
#define WDT_TIMEOUT 10 // seconds
#define EEPROM_SIZE 512

// Timer flags
hw_timer_t *task_timer = NULL;
uint32_t timer_1_ms = 0;
uint8_t do_1_hz_task = 0;
uint8_t do_100hz_task = 0;
uint8_t do_2s_task = 0;
uint8_t do_20hz_task = 0;
uint8_t do_initialise_web_service = 0;
uint8_t do_ota_update = 0;

// Sequence selection flags
char CH1_seq[20];
char CH2_seq[20];
char CH3_seq[20];
char CH4_seq[20];
char CH5_seq[20];
bool CH1_seq_active = false;
bool CH2_seq_active = false;
bool CH3_seq_active = false;
bool CH4_seq_active = false;
bool CH5_seq_active = false;

// Array
volatile float temp_readings[18000] = {0}; // 18000 = 5 hours of data at 1Hz

// Function declarations
void wifi_event(WiFiEvent_t event);
void initialise_web_services(void);
void on_web_socket_event(uint8_t client_num, WStype_t type, uint8_t *payload, size_t length);
void on_index_request(AsyncWebServerRequest *request);
void on_network_connect_request(AsyncWebServerRequest *request);
void on_css_request(AsyncWebServerRequest *request);
void on_page_not_found(AsyncWebServerRequest *request);
void led_rgb_setup(void);
void led_rgb_handling(void);
void accelerometer_setup(void);
void accelerometer_handling(void);
void temp_sensors_setup(void);
void temp_sensor_handling(void);
void buzzer_handling(void);
void sequence_handling(void);
void sequence_handling_data_capture(void);
void refresh_handling(void);
void ip_address_start_sequence(void); // Start IP address RGB+Buzzer sequence
void ip_address_handling(void);
void set_rgb_color(uint8_t r, uint8_t g, uint8_t b);
void clear_temperature_log_file(void);
void log_temperature_to_sd_card(float temp1, float temp2, float temp3, float temp4, float temp5);
void read_battery_voltage(void);
void log_sequence_completion_to_sd(int probe_num, const char *test_type, float temp);
void data_retrieval_test_handling(void);

// Timers acting as task schedulers
void IRAM_ATTR onTimer()
{
    timer_1_ms++;
    // Serial.println("Timer 1ms ticked");
    if ((timer_1_ms % 1000) == 0)
    {
        do_1_hz_task = TRUE;
    }
    if ((timer_1_ms % 100) == 0)
    {
        do_100hz_task = TRUE; // 100Hz task flag
    }
    if ((timer_1_ms % 50) == 0)
    {
        do_20hz_task = TRUE;
    }
    if ((timer_1_ms % 2000) == 0)
    {
        do_2s_task = TRUE;
    }
    if (timer_1_ms > 0xFFFFFFFE)
    { // reset timer_1_ms to prevent overflow
        timer_1_ms = 0;
    }
}

void setup()
{
    // Start Serial port for debugging
    Serial.begin(115200);

    // Sequence handling flag
    web_system_state.sequence_active = false;

    // RGB LED pin setup
    led_rgb_setup();

    // Set I2C pins
    Wire.begin(IIC_SDA, IIC_SCL); // SDA, SCL

    // Temperature sensor setup
    temp_sensors_setup();

    // Accelerometer setup
    accelerometer_setup();

    // Buzzer pin setup
    pinMode(BUZZER_PIN, OUTPUT);

    // SD card detect pin setup
    pinMode(SD_DETECT_PIN, INPUT_PULLUP); // SD detect pin, HIGH = no card, LOW = card inserted
    last_sd_inserted = (digitalRead(SD_DETECT_PIN) == LOW);

    // Initialize SD card in SPI mode (4-bit mode not supported with ESP32-C6)
    // Custom SPI pins for SD card
    SD_SPI.begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS, SD_SPI))
    {
        Serial.println("SD Card Mount Failed!");
    }
    else
    {
        Serial.println("SD Card initialized in SPI mode.");
        clear_temperature_log_file(); // Clear the temperature log file on startup
    }

    // create unique wifi ssid from ESP32 chip ID
    uint64_t chip_id;
    char chip_id_string_array[30];
    chip_id = ESP.getEfuseMac();
    char id_lower_4_bytes[9];
    char id_higher_2_bytes[5];
    id_lower_4_bytes[8] = '\0';
    id_higher_2_bytes[4] = '\0';
    sprintf(id_lower_4_bytes, "%08X", (uint32_t)chip_id);
    sprintf(id_higher_2_bytes, "%04X", (uint16_t)(chip_id >> 32));
    sprintf(chip_id_string_array, "%s%s", id_higher_2_bytes, id_lower_4_bytes);
    chip_id_string_array[12] = '\0';
    sprintf(ssid_buffer, "BB1 %s", chip_id_string_array);
    Serial.println(ssid_buffer);

    // Make sure we can read the file system, if not, let watchdog reset the ESP32
    if (!SPIFFS.begin())
    {
        Serial.println("Error mounting SPIFFS");
        while (1)
            ;
    }

    EEPROM.begin(EEPROM_SIZE); // EEPROM has a maximum of 512 bytes in flash
                               /*EEPROM memory map
                                *Byte 0-30 SSID
                                *Byte 31-60 Password
                                */

    if (network_ssid[0] == 0)
    {
        start_ap = TRUE;
    }
    else if (network_ssid[0] == 255)
    {
        Serial.println("Firmware was updated via USB, clearing network credentials ");
        start_ap = TRUE;
        for (uint8_t network_clear_counter = 1; network_clear_counter < 61; network_clear_counter++)
        {
            EEPROM.write(network_clear_counter, 0); // Clear saved network credentials
            EEPROM.commit();
        }
        EEPROM.write(61, 0); // Clear saved network credentials
        EEPROM.commit();
    }
    else
    {
        start_ap = FALSE;
    }

    //  Start listening for Wifi events
    WiFi.onEvent(wifi_event);

    // // Timers
    task_timer = timerBegin(1000000);
    timerAttachInterrupt(task_timer, &onTimer);
    timerWrite(task_timer, 1000);                           // Set to 1000000 for a sec and decrease for faster
    timerAlarm(task_timer, 1000, true, (sizeof(uint64_t))); // Set alarm to 1 second (1000000 microseconds) and auto-reload
    timerStart(task_timer);                                 // Start the timer

    // Starting WiFi AP and connecting to local network
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(ssid_buffer, password);

    // If no network credentials are found
    if (start_ap == TRUE)
    {
        Serial.println("AP running");
        Serial.print("AP IP address: ");
        Serial.println(WiFi.softAPIP()); // Print our IP address
        ip_address = WiFi.softAPIP().toString();
    }
    else
    {
        WiFi.begin(network_ssid, network_password);
        Serial.println("Connecting to local network : ");
        Serial.println(network_ssid);
        Serial.println(network_password);
    }

    // initialize watchdog timers at end of setup()
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WDT_TIMEOUT * 1000, // Convert seconds to milliseconds
        .trigger_panic = true             // Enable panic so ESP32 restarts
    };
    esp_task_wdt_init(&wdt_config); // Initialize watchdog with the new configuration
    esp_task_wdt_add(NULL);         // add main thread to WDT watch list
}
/////////////////////////////////////////////////////////////////////////////////////////MAIN////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{                         // main loop
    esp_task_wdt_reset(); // kick watchdog

    if (do_initialise_web_service == TRUE)
    {
        do_initialise_web_service = FALSE;
        initialise_web_services(); // start websockets etc after establishing wifi connection
        Serial.println("Web services initialised");
    }

    if (!do_initialise_web_service)
    {
        web_socket.loop(); // Look for and handle WebSocket data asynchronously
    }

    // if (do_ota_update) // if OTA update is requested, disable timer and start OTA process
    // {
    //     do_ota_update = 0;
    //     // timerAlarmDisable(task_timer);
    //     timerDetachInterrupt(task_timer);
    //     init_local_ota();
    //     local_ota_loop();
    // }

    if (do_20hz_task) // 20Hz task
    {
        do_20hz_task = FALSE;
        // Serial.println("[20Hz] 20Hz task triggered");

        // 20Hz tasks go here
        accelerometer_handling();

        // if (web_system_state.webpage_load_graph_info == true)
        // {
        //     refresh_handling();
        // }
    }

    if (do_1_hz_task) // 1Hz task
    {
        do_1_hz_task = FALSE;
        // Serial.println("[1Hz] 1Hz task triggered");

        // 1Hz tasks go here
        temp_sensor_handling();
        // led_rgb_handling();

        // Sequence handling
        if (web_system_state.sequence_active == true)
        {
            sequence_handling();
            sequence_handling_data_capture(); // Capture data for the sequence
        }

        // Data retrieval test
        // data_retrieval_test_handling(); // Handle data retrieval test

        // SD card detect handling
        bool sd_inserted = (digitalRead(SD_DETECT_PIN) == LOW);
        if (sd_inserted != last_sd_inserted)
        {
            if (sd_inserted)
            {
                Serial.println("SD card inserted.");
            }
            else
            {
                Serial.println("SD card removed.");
            }
            last_sd_inserted = sd_inserted;
        }
        // Always print current state at 1Hz
        Serial.print("SD card state: ");
        Serial.println(sd_inserted ? "Inserted" : "Not inserted");
    }

    if (do_100hz_task) // 100Hz task
    {
        do_100hz_task = FALSE;
        // Serial.println("[100Hz] 100Hz task triggered");

        ip_address_handling(); // IP address sequence handling
    }

    if (do_2s_task) // 2s task
    {
        do_2s_task = FALSE;
        // Serial.println("[0.5Hz] 0.5Hz task triggered");

        // 2s tasks go here
        read_battery_voltage();
    }
}

int8_t wifi_network_scan() // returns number of networks found
{
    Serial.println("Wi-Fi network scan start");
    int8_t n = WiFi.scanNetworks(); // WiFi.scanNetworks will return the number of networks found
    Serial.println("scan done");
    if (n <= 0)
    {
        Serial.println("no networks found");
    }
    else
    {
        Serial.print(n);
        Serial.println(" networks found");
        for (uint8_t i = 0; i < n; ++i) // Print SSID and RSSI for each network found
        {
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(WiFi.SSID(i));
        }
    }
    Serial.println("");
    return (n);
}

void recover_wifi_connection() // attempt to reconnect to stored network
{
    if ((WiFi.status() != WL_CONNECTED) && (start_ap == FALSE))
    {
        uint8_t MatchingNetworkFound = FALSE;
        Serial.println("Wifi disconnected, attempting to reconnect now");
        WiFi.disconnect(true);
        int8_t n = wifi_network_scan();
        if (n < 0)
        {
            n = 0;
        }
        for (int8_t i = 0; i < n; ++i)
        {
            char temp[4];
            WiFi.SSID(i).toCharArray(temp, sizeof(temp));
            if (temp[0] == network_ssid[0])
                if (temp[1] == network_ssid[1])
                    if (temp[2] == network_ssid[2])
                    {
                        i = n;
                        Serial.println(
                            "Network found that matched stored credentials, "
                            "attempting connection now");
                        WiFi.disconnect();
                        WiFi.begin(network_ssid, network_password);
                        Serial.println("Connecting to local network : ");
                        Serial.println(network_ssid);
                        Serial.println(network_password);
                        MatchingNetworkFound = TRUE;
                    }
        }
        if (MatchingNetworkFound == FALSE)
        {
            Serial.println("No matching networks found, starting AP");
            WiFi.disconnect();
            Serial.println("AP running");
            Serial.print("My IP address: "); // Print our IP address
            Serial.println(WiFi.softAPIP());
            ip_address = WiFi.softAPIP().toString();
        }
    }
}

void wifi_event(WiFiEvent_t event)
{
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch (event)
    {
    case ARDUINO_EVENT_WIFI_READY:
        Serial.println("WiFi interface ready");
        break;
    case ARDUINO_EVENT_WIFI_SCAN_DONE:
        Serial.println("Completed scan for access points");
        break;
    case ARDUINO_EVENT_WIFI_STA_START:
        Serial.println("WiFi client started");
        break;
    case ARDUINO_EVENT_WIFI_STA_STOP:
        Serial.println("WiFi clients stopped");
        break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
        Serial.println("Connected to access point");
        break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        Serial.println("Disconnected from WiFi access point or connection failed");
        if (start_ap == FALSE)
        {
            Serial.println("Lost connection to AP, reconnecting...");
        }
        break;
    case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE:
        Serial.println("Authentication mode of access point has changed");
        break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        Serial.print("Obtained IP address: ");
        Serial.println(WiFi.localIP());
        Serial.println("Connected to local network:");
        Serial.println(network_ssid);
        Serial.print("My IP address: ");
        Serial.println(WiFi.localIP());
        ip_address = WiFi.localIP().toString();
        do_initialise_web_service = TRUE;
        Serial.println("Initialise web services flag set to TRUE");
        ip_address_start_sequence(); // Start IP address RGB+Buzzer sequence
        break;
    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
        Serial.println("Lost IP address and IP address is reset to 0");
        break;
    case ARDUINO_EVENT_WIFI_AP_START:
        Serial.println("WiFi access point started");
        Serial.println("AP running");
        Serial.print("My IP address: ");
        Serial.println(WiFi.softAPIP());
        ip_address = WiFi.softAPIP().toString();
        do_initialise_web_service = TRUE;
        Serial.println("Initialise web services flag set to TRUE (1)");
        ip_address_start_sequence(); // Start IP address RGB+Buzzer sequence
        break;
    case ARDUINO_EVENT_WIFI_AP_STOP:
        Serial.println("WiFi access point  stopped");
        break;
    case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
        Serial.println("Client connected");
        break;
    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
        Serial.println("Client disconnected");
        break;
    case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED:
        Serial.println("Assigned IP address to client");
        Serial.println(WiFi.softAPIP());
        break;
    case ARDUINO_EVENT_WIFI_AP_PROBEREQRECVED:
        Serial.println("Received probe request");
        break;
    default:
        break;
    }
}

void initialise_web_services()
{
    web_socket.begin(); // Start WebSocket server and assign callback when
                        // connected to WiFi network
    web_socket.onEvent(on_web_socket_event);
    server.begin(); // Start web server
    server.on("/", HTTP_GET,
              on_index_request); // On HTTP request for root, provide index.html file
    server.on("/NetworkConnection", HTTP_GET,
              on_network_connect_request); // On HTTP request for connect to local network
    server.on("/style.css", HTTP_GET,
              on_css_request); // On HTTP request for style sheet, provide style.css

    server.on("/chart.js", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/chart.js", "application/javascript"); }); // Serve chart.js file
    server.on("/chartjs-adapter-date-fns.js", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/chartjs-adapter-date-fns.js", "application/javascript"); }); // Serve chartjs-adapter-date-fns.js file

    server.onNotFound(on_page_not_found); // Handle requests for pages that do not exist
}

// Callback: receiving any WebSocket message
void on_web_socket_event(uint8_t client_num, WStype_t type, uint8_t *payload, size_t length)
{
    uint8_t string_counter = 0;
    char wifi_rx_buffer_array[30];
    // Figure out the type of WebSocket event
    switch (type)
    {
    // Client has disconnected
    case WStype_DISCONNECTED:
        Serial.printf("[%u] Disconnected!\n", client_num);
        break;
    // New client has connected send new parameter
    case WStype_CONNECTED:
    {
        sprintf(msg_buf, "CSV%d", 15);
        web_socket.sendTXT(client_num, msg_buf);
    }
    break;
    // Handle text messages from websocket client
    case WStype_TEXT:
        // Print out received message
        // Serial.printf("[%u] Received text: %s\n", client_num, payload);

        sprintf(wifi_rx_buffer_array, "%s", payload);

        if (strcmp((char *)payload, "SendMeParameters") == 0)
        {
            // send polled data
            if (WiFi.status() == WL_CONNECTED)
            { // Send WiFi connection status
                sprintf(msg_buf, "WS1:%s", network_ssid);
                web_socket.sendTXT(client_num, msg_buf);
            }
            else if (WiFi.status() == WL_CONNECT_FAILED)
            {
                sprintf(msg_buf, "WS0:Incorrect Password!");
                web_socket.sendTXT(client_num, msg_buf);
            }
            else
            {
                sprintf(msg_buf, "WS0:Disconnected!");
                web_socket.sendTXT(client_num, msg_buf);
            }
            // send all other parameters here
            // Temperature Sensor data
            sprintf(msg_buf, "MC1%f", sensor_data.temp_sense_ch1);
            web_socket.sendTXT(client_num, msg_buf);
            // Serial.print("[WebSocket] Sent: ");
            // Serial.println(msg_buf);

            sprintf(msg_buf, "MC2%f", sensor_data.temp_sense_ch2);
            web_socket.sendTXT(client_num, msg_buf);
            // Serial.print("[WebSocket] Sent: ");
            // Serial.println(msg_buf);

            sprintf(msg_buf, "MC3%f", sensor_data.temp_sense_ch3);
            web_socket.sendTXT(client_num, msg_buf);
            // Serial.print("[WebSocket] Sent: ");
            // Serial.println(msg_buf);

            sprintf(msg_buf, "MC4%f", sensor_data.temp_sense_ch4);
            web_socket.sendTXT(client_num, msg_buf);
            // Serial.print("[WebSocket] Sent: ");
            // Serial.println(msg_buf);

            sprintf(msg_buf, "MC5%f", sensor_data.temp_sense_ch5);
            web_socket.sendTXT(client_num, msg_buf);
            // Serial.print("[WebSocket] Sent: ");
            // Serial.println(msg_buf);

            // Accelerometer data
            sprintf(msg_buf, "ACX%f", sensor_data.accelerometer_x);
            web_socket.sendTXT(client_num, msg_buf);
            // Serial.print("[WebSocket] Sent: ");
            // Serial.println(msg_buf);

            sprintf(msg_buf, "ACY%f", sensor_data.accelerometer_y);
            web_socket.sendTXT(client_num, msg_buf);
            // Serial.print("[WebSocket] Sent: ");
            // Serial.println(msg_buf);

            sprintf(msg_buf, "ACZ%f", sensor_data.accelerometer_z);
            web_socket.sendTXT(client_num, msg_buf);
            // Serial.print("[WebSocket] Sent: ");
            // Serial.println(msg_buf);

            // Battery voltage
            sprintf(msg_buf, "BAT%f", sensor_data.battery_voltage);
            web_socket.sendTXT(client_num, msg_buf);
            // Serial.print("[WebSocket] Sent: ");
            // Serial.println(msg_buf);

            // Sequence data
            if (web_system_state.sequence_active == true)
            {
                sprintf(msg_buf, "SQ1%s", web_system_state.probe1_seq);
                web_socket.sendTXT(client_num, msg_buf);
                sprintf(msg_buf, "SQ2%s", web_system_state.probe2_seq);
                web_socket.sendTXT(client_num, msg_buf);
                sprintf(msg_buf, "SQ3%s", web_system_state.probe3_seq);
                web_socket.sendTXT(client_num, msg_buf);
                sprintf(msg_buf, "SQ4%s", web_system_state.probe4_seq);
                web_socket.sendTXT(client_num, msg_buf);
                sprintf(msg_buf, "SQ5%s", web_system_state.probe5_seq);
                web_socket.sendTXT(client_num, msg_buf);

                sprintf(msg_buf, "SEQ%s", web_system_state.sequence_status);
                web_socket.sendTXT(client_num, msg_buf);
                // Serial.print("[WebSocket] Sent: ");
                // Serial.println(msg_buf);
            }
            // else
            // {
            //     sprintf(msg_buf, "SEQ%s", "Sequence Idle");
            //     web_socket.sendTXT(client_num, msg_buf);
            //     // Serial.print("[WebSocket] Sent: ");
            //     // Serial.println(msg_buf);
            // }

            // char ws_log_buf[512];
            // snprintf(ws_log_buf, sizeof(ws_log_buf),
            //          "[WebSocket] Sent: MC1%.2f, MC2%.2f, MC3%.2f, MC4%.2f, MC5%.2f | "
            //          "ACX%.2f, ACY%.2f, ACZ%.2f | BAT%.2f | SEQ:%s",
            //          sensor_data.temp_sense_ch1,
            //          sensor_data.temp_sense_ch2,
            //          sensor_data.temp_sense_ch3,
            //          sensor_data.temp_sense_ch4,
            //          sensor_data.temp_sense_ch5,
            //          sensor_data.accelerometer_x,
            //          sensor_data.accelerometer_y,
            //          sensor_data.accelerometer_z,
            //          sensor_data.battery_voltage,
            //          web_system_state.sequence_active ? web_system_state.sequence_status : "Sequence Idle");
            // Serial.println(ws_log_buf);
        }

        if ((wifi_rx_buffer_array[0] == 'S') & (wifi_rx_buffer_array[1] == 'S'))
        { // receive SSID
            start_ap = FALSE;
            for (string_counter = 0; string_counter < 25; string_counter++)
            {
                network_ssid[string_counter] = wifi_rx_buffer_array[string_counter + 5];
                EEPROM.write(string_counter + 1, network_ssid[string_counter]);
                EEPROM.commit();
            }
            Serial.println("SSID received and handled");
        }
        else if ((wifi_rx_buffer_array[0] == 'P') & (wifi_rx_buffer_array[1] == 'a'))
        { // receive password
            for (string_counter = 0; string_counter < 25; string_counter++)
            {
                network_password[string_counter] = wifi_rx_buffer_array[string_counter + 5];
                EEPROM.write(string_counter + 31, network_password[string_counter]);
                EEPROM.commit();
            }
            Serial.println("Connecting to Network: ");
            Serial.println(network_ssid);
            Serial.println(network_password);
            start_ap = FALSE;
            delay(200);
            WiFi.begin(network_ssid, network_password);
        }
        else if (strcmp((char *)payload, "RefreshWifiNetworks") == 0)
        { // send scanned networks
            uint8_t number_of_found_networks = wifi_network_scan();
            if (number_of_found_networks == 0)
            {
                sprintf(msg_buf, "Net0");
                web_socket.sendTXT(client_num, msg_buf);
            }

            for (uint8_t i = 0; i < number_of_found_networks; ++i)
            {
                char tempSSID[64];
                WiFi.SSID(i).toCharArray(tempSSID, sizeof(tempSSID));
                sprintf(msg_buf, "Net%s", tempSSID);
                web_socket.sendTXT(client_num, msg_buf);
                delay(10);
            }
        }

        // else if (strcmp((char *)payload, "FirmwareOTA") == 0)
        // { // Firmware OTA mode
        //     do_ota_update = TRUE;
        // }
        else if (strcmp((char *)payload, "GotoAPMode") == 0)
        { // Go to Wifi AP-only mode
            for (string_counter = 1; string_counter < 61; string_counter++)
            { // Clear saved network credentials
                EEPROM.write(string_counter, 0);
                EEPROM.commit();
            }
            WiFi.disconnect();
            start_ap = TRUE;
            ip_address = WiFi.softAPIP().toString();
        }

        // Handle Buzzer Test button push
        else if (strcmp((char *)payload, "toggleR1") == 0)
        { // Play test buzzer tone
            buzzer_handling();
        }
        // Handle the sequence message
        else if ((wifi_rx_buffer_array[0] == 's') & (wifi_rx_buffer_array[5] == '_'))
        {
            // Set sequence handling flag so function is called every xxx interval
            web_system_state.sequence_active = true;
            sprintf(web_system_state.sequence_status, "Sequence Busy");

            // Read each of the probe selections and set flags before calling the sequence handling function
            // Read probe 1 selection
            if ((wifi_rx_buffer_array[9] == 'B') & (wifi_rx_buffer_array[10] == 'T'))
            {
                // Boiling Test selected for Probe 1
                sprintf(web_system_state.probe1_seq, "Boil Test");
                CH1_seq_active = true;
                web_system_state.probe_states[0] = true;
            }
            else if ((wifi_rx_buffer_array[9] == 'C') & (wifi_rx_buffer_array[10] == 'T'))
            {
                // Cold Test selected for Probe 1
                sprintf(web_system_state.probe1_seq, "Cold Test");
                CH1_seq_active = true;
                web_system_state.probe_states[0] = true;
            }
            else if ((wifi_rx_buffer_array[9] == 'N') & (wifi_rx_buffer_array[10] == 'A'))
            {
                // None selected for Probe 1
                sprintf(web_system_state.probe1_seq, "N.A.");
                CH1_seq_active = false;
                web_system_state.probe_states[0] = false;
            }

            // Read probe 2 selection
            if ((wifi_rx_buffer_array[12] == 'B') & (wifi_rx_buffer_array[13] == 'T'))
            {
                // Boiling Test selected for Probe 2
                sprintf(web_system_state.probe2_seq, "Boil Test");
                CH2_seq_active = true;
                web_system_state.probe_states[1] = true;
            }
            else if ((wifi_rx_buffer_array[12] == 'C') & (wifi_rx_buffer_array[13] == 'T'))
            {
                // Cold Test selected for Probe 2
                sprintf(web_system_state.probe2_seq, "Cold Test");
                CH2_seq_active = true;
                web_system_state.probe_states[1] = true;
            }
            else if ((wifi_rx_buffer_array[12] == 'N') & (wifi_rx_buffer_array[13] == 'A'))
            {
                // None selected for Probe 2
                sprintf(web_system_state.probe2_seq, "N.A.");
                CH2_seq_active = false;
                web_system_state.probe_states[1] = false;
            }

            // Read probe 3 selection
            if ((wifi_rx_buffer_array[15] == 'B') & (wifi_rx_buffer_array[16] == 'T'))
            {
                // Boiling Test selected for Probe 3
                sprintf(web_system_state.probe3_seq, "Boil Test");
                CH3_seq_active = true;
                web_system_state.probe_states[2] = true;
            }
            else if ((wifi_rx_buffer_array[15] == 'C') & (wifi_rx_buffer_array[16] == 'T'))
            {
                // Cold Test selected for Probe 3
                sprintf(web_system_state.probe3_seq, "Cold Test");
                CH3_seq_active = true;
                web_system_state.probe_states[2] = true;
            }
            else if ((wifi_rx_buffer_array[15] == 'N') & (wifi_rx_buffer_array[16] == 'A'))
            {
                // None selected for Probe 3
                sprintf(web_system_state.probe3_seq, "N.A.");
                CH3_seq_active = false;
                web_system_state.probe_states[2] = false;
            }

            // Read probe 4 selection
            if ((wifi_rx_buffer_array[18] == 'B') & (wifi_rx_buffer_array[19] == 'T'))
            {
                // Boiling Test selected for Probe 4
                sprintf(web_system_state.probe4_seq, "Boil Test");
                CH4_seq_active = true;
                web_system_state.probe_states[3] = true;
            }
            else if ((wifi_rx_buffer_array[18] == 'C') & (wifi_rx_buffer_array[19] == 'T'))
            {
                // Cold Test selected for Probe 4
                sprintf(web_system_state.probe4_seq, "Cold Test");
                CH4_seq_active = true;
                web_system_state.probe_states[3] = true;
            }
            else if ((wifi_rx_buffer_array[18] == 'N') & (wifi_rx_buffer_array[19] == 'A'))
            {
                // None selected for Probe 4
                sprintf(web_system_state.probe4_seq, "N.A.");
                CH4_seq_active = false;
                web_system_state.probe_states[3] = false;
            }

            // Read probe 5 selection
            if ((wifi_rx_buffer_array[21] == 'B') & (wifi_rx_buffer_array[22] == 'T'))
            {
                // Boiling Test selected for Probe 5
                sprintf(web_system_state.probe5_seq, "Boil Test");
                CH5_seq_active = true;
                web_system_state.probe_states[4] = true;
            }
            else if ((wifi_rx_buffer_array[21] == 'C') & (wifi_rx_buffer_array[22] == 'T'))
            {
                // Cold Test selected for Probe 5
                sprintf(web_system_state.probe5_seq, "Cold Test");
                CH5_seq_active = true;
                web_system_state.probe_states[4] = true;
            }
            else if ((wifi_rx_buffer_array[21] == 'N') & (wifi_rx_buffer_array[22] == 'A'))
            {
                // None selected for Probe 5
                sprintf(web_system_state.probe5_seq, "N.A.");
                CH5_seq_active = false;
                web_system_state.probe_states[4] = false;
            }

            // Read Grid Turn Time value
            if ((wifi_rx_buffer_array[24] == 'G') & (wifi_rx_buffer_array[25] == 'T'))
            {
                char *gtt_ptr = strstr(wifi_rx_buffer_array, "GTT");
                if (gtt_ptr != NULL)
                {
                    int value = atoi(gtt_ptr + 3); // skip "GTT" and convert the rest to int
                    if (value >= 60 && value <= 300)
                    {
                        grid_turn_time = value;
                        // Serial.println(grid_turn_time);
                    }
                    else
                    {
                        grid_turn_time = 180; // default value
                    }
                }
            }
        }
        else if (strcmp((char *)payload, "SendMeRefresh") == 0)
        { // Page refreshed, send system status and parameters if needed
            // Set page refresh flag to true
            // web_system_state.webpage_load_graph_info = true;
            buzzer_handling();
        }
        break;
    // For everything else: do nothing
    case WStype_BIN:
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
    default:
        break;
    }
}

void on_index_request(AsyncWebServerRequest *request) // Callback: send homepage
{
    IPAddress remote_ip = request->client()->remoteIP();
    Serial.println("[" + remote_ip.toString() + "] HTTP GET request of " + request->url());
    request->send(SPIFFS, "/index.html", "text/html");
}
void on_network_connect_request(AsyncWebServerRequest *request) // Callback: send network connection page
{
    IPAddress remote_ip = request->client()->remoteIP();
    Serial.println("[" + remote_ip.toString() + "] HTTP GET request of " + request->url());
    request->send(SPIFFS, "/NetworkConnection.html", "text/html");
}

void on_css_request(AsyncWebServerRequest *request) // Callback: send style sheet
{
    IPAddress remote_ip = request->client()->remoteIP();
    Serial.println("[" + remote_ip.toString() + "] HTTP GET request of " + request->url());
    request->send(SPIFFS, "/style.css", "text/css");
}

void on_page_not_found(AsyncWebServerRequest *request) // Callback: send 404 if requested file does not exist
{
    IPAddress remote_ip = request->client()->remoteIP();
    Serial.println("[" + remote_ip.toString() + "] HTTP GET request of " + request->url());
    request->send(404, "text/plain", "Not found");
}

void led_rgb_setup(void)
{
    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    pinMode(LED_BLUE_PIN, OUTPUT);
    digitalWrite(LED_RED_PIN, HIGH);
    digitalWrite(LED_GREEN_PIN, LOW);
    digitalWrite(LED_BLUE_PIN, LOW);
}

void led_rgb_handling(void) // Used to test every possible RGB LED color combination
{
    if (digitalRead(LED_RED_PIN) == HIGH && digitalRead(LED_GREEN_PIN) == LOW && digitalRead(LED_BLUE_PIN) == LOW)
    {
        digitalWrite(LED_GREEN_PIN, HIGH); // RED + GREEN (YELLOW)
    }
    else if (digitalRead(LED_RED_PIN) == HIGH && digitalRead(LED_GREEN_PIN) == HIGH && digitalRead(LED_BLUE_PIN) == LOW)
    {
        digitalWrite(LED_RED_PIN, LOW); // GREEN
    }
    else if (digitalRead(LED_RED_PIN) == LOW && digitalRead(LED_GREEN_PIN) == HIGH && digitalRead(LED_BLUE_PIN) == LOW)
    {
        digitalWrite(LED_BLUE_PIN, HIGH); // GREEN + BLUE (CYAN)
    }
    else if (digitalRead(LED_RED_PIN) == LOW && digitalRead(LED_GREEN_PIN) == HIGH && digitalRead(LED_BLUE_PIN) == HIGH)
    {
        digitalWrite(LED_GREEN_PIN, LOW); // BLUE
    }
    else if (digitalRead(LED_RED_PIN) == LOW && digitalRead(LED_GREEN_PIN) == LOW && digitalRead(LED_BLUE_PIN) == HIGH)
    {
        digitalWrite(LED_RED_PIN, HIGH); // RED + BLUE (MAGENTA)
    }
    else if (digitalRead(LED_RED_PIN) == HIGH && digitalRead(LED_GREEN_PIN) == LOW && digitalRead(LED_BLUE_PIN) == HIGH)
    {
        digitalWrite(LED_GREEN_PIN, HIGH); // RED + GREEN + BLUE (WHITE)
    }
    else if (digitalRead(LED_RED_PIN) == HIGH && digitalRead(LED_GREEN_PIN) == HIGH && digitalRead(LED_BLUE_PIN) == HIGH)
    {
        digitalWrite(LED_GREEN_PIN, LOW); // RED
        digitalWrite(LED_BLUE_PIN, LOW);
    }
}

void accelerometer_setup(void)
{
    // Begin LIS3DH communication
    LIS.begin(WIRE, ACC_I2C_ADR); // IIC init dafault :0x18
    delay(100);
    LIS.setOutputDataRate(LIS3DHTR_DATARATE_50HZ);
    LIS.setHighSolution(true); // High solution enable
}

void accelerometer_handling(void)
{
    sensor_data.accelerometer_x = LIS.getAccelerationX();
    sensor_data.accelerometer_y = LIS.getAccelerationY();
    sensor_data.accelerometer_z = LIS.getAccelerationZ();

    // Serial.println("----------Accelerometer----------");
    // Serial.print("x:");
    // Serial.print(sensor_data.accelerometer_x);
    // Serial.print("  ");
    // Serial.print("y:");
    // Serial.print(sensor_data.accelerometer_y);
    // Serial.print("  ");
    // Serial.print("z:");
    // Serial.println(sensor_data.accelerometer_z);
    // Serial.println(F("------------------------------------------------------------"));
}

void temp_sensors_setup(void)
{

    Serial.println("MCP9600 HW test");

    /* Initialise the driver with I2C_ADDRESS and the default I2C bus. */
    if (!mcp1.begin(I2C_ADDRESS_1, &Wire))
    {
        Serial.println("Sensor 1 not found. Check wiring!");
        while (1)
            ;
    }
    if (!mcp2.begin(I2C_ADDRESS_2, &Wire))
    {
        Serial.println("Sensor 2 not found. Check wiring!");
        while (1)
            ;
    }
    if (!mcp3.begin(I2C_ADDRESS_3, &Wire))
    {
        Serial.println("Sensor 3 not found. Check wiring!");
        while (1)
            ;
    }
    if (!mcp4.begin(I2C_ADDRESS_4, &Wire))
    {
        Serial.println("Sensor 4 not found. Check wiring!");
        while (1)
            ;
    }
    if (!mcp5.begin(I2C_ADDRESS_5, &Wire))
    {
        Serial.println("Sensor 5 not found. Check wiring!");
        while (1)
            ;
    }

    Serial.println("Found MCP9600!");

    /* Set and print ambient resolution */
    mcp1.setAmbientResolution(ambientRes);
    mcp2.setAmbientResolution(ambientRes);
    mcp3.setAmbientResolution(ambientRes);
    mcp4.setAmbientResolution(ambientRes);
    mcp5.setAmbientResolution(ambientRes);
    Serial.print("Ambient Resolution set to: ");
    switch (ambientRes)
    {
    case RES_ZERO_POINT_25:
        Serial.println("0.25°C");
        break;
    case RES_ZERO_POINT_125:
        Serial.println("0.125°C");
        break;
    case RES_ZERO_POINT_0625:
        Serial.println("0.0625°C");
        break;
    case RES_ZERO_POINT_03125:
        Serial.println("0.03125°C");
        break;
    }

    mcp1.setADCresolution(MCP9600_ADCRESOLUTION_18);
    mcp2.setADCresolution(MCP9600_ADCRESOLUTION_18);
    mcp3.setADCresolution(MCP9600_ADCRESOLUTION_18);
    mcp4.setADCresolution(MCP9600_ADCRESOLUTION_18);
    mcp5.setADCresolution(MCP9600_ADCRESOLUTION_18);
    Serial.print("ADC resolution set to ");
    switch (mcp1.getADCresolution())
    {
    case MCP9600_ADCRESOLUTION_18:
        Serial.print("18");
        break;
    case MCP9600_ADCRESOLUTION_16:
        Serial.print("16");
        break;
    case MCP9600_ADCRESOLUTION_14:
        Serial.print("14");
        break;
    case MCP9600_ADCRESOLUTION_12:
        Serial.print("12");
        break;
    }
    Serial.println(" bits");

    mcp1.setThermocoupleType(MCP9600_TYPE_K);
    mcp2.setThermocoupleType(MCP9600_TYPE_K);
    mcp3.setThermocoupleType(MCP9600_TYPE_K);
    mcp4.setThermocoupleType(MCP9600_TYPE_K);
    mcp5.setThermocoupleType(MCP9600_TYPE_K);
    Serial.print("Thermocouple type set to ");
    switch (mcp1.getThermocoupleType())
    {
    case MCP9600_TYPE_K:
        Serial.print("K");
        break;
    case MCP9600_TYPE_J:
        Serial.print("J");
        break;
    case MCP9600_TYPE_T:
        Serial.print("T");
        break;
    case MCP9600_TYPE_N:
        Serial.print("N");
        break;
    case MCP9600_TYPE_S:
        Serial.print("S");
        break;
    case MCP9600_TYPE_E:
        Serial.print("E");
        break;
    case MCP9600_TYPE_B:
        Serial.print("B");
        break;
    case MCP9600_TYPE_R:
        Serial.print("R");
        break;
    }
    Serial.println(" type");

    mcp1.setFilterCoefficient(2); // Design choice mention
    mcp2.setFilterCoefficient(2);
    mcp3.setFilterCoefficient(2);
    mcp4.setFilterCoefficient(2);
    mcp5.setFilterCoefficient(2);
    Serial.print("Filter coefficient value set to: ");
    Serial.println(mcp1.getFilterCoefficient());

    // mcp.setAlertTemperature(1, 30);
    // Serial.print("Alert #1 temperature set to ");
    // Serial.println(mcp.getAlertTemperature(1));
    // mcp.configureAlert(1, true, true);  // alert 1 enabled, rising temp

    mcp1.enable(true);
    mcp2.enable(true);
    mcp3.enable(true);
    mcp4.enable(true);
    mcp5.enable(true);
}

void temp_sensor_handling(void)
{
    sensor_data.temp_sense_ch1 = mcp1.readThermocouple();
    sensor_data.temp_sense_ch2 = mcp2.readThermocouple();
    sensor_data.temp_sense_ch3 = mcp3.readThermocouple();
    sensor_data.temp_sense_ch4 = mcp4.readThermocouple();
    sensor_data.temp_sense_ch5 = mcp5.readThermocouple();

    // Serial.println("----------MCP1 Readings----------");
    // Serial.print("Hot Junction: ");
    // Serial.println(sensor_data.temp_sense_ch1);
    // Serial.print("Cold Junction: ");
    // Serial.println(mcp1.readAmbient());
    // Serial.print("ADC: ");
    // Serial.print(mcp1.readADC() * 2);
    // Serial.println(" uV");
    // Serial.println(F("------------------------------------------------------------"));

    // Serial.println("----------MCP2 Readings----------");
    // Serial.print("Hot Junction: ");
    // Serial.println(sensor_data.temp_sense_ch2);
    // Serial.print("Cold Junction: ");
    // Serial.println(mcp2.readAmbient());
    // Serial.print("ADC: ");
    // Serial.print(mcp2.readADC() * 2);
    // Serial.println(" uV");
    // Serial.println(F("------------------------------------------------------------"));

    // Serial.println("----------MCP3 Readings----------");
    // Serial.print("Hot Junction: ");
    // Serial.println(sensor_data.temp_sense_ch3);
    // Serial.print("Cold Junction: ");
    // Serial.println(mcp3.readAmbient());
    // Serial.print("ADC: ");
    // Serial.print(mcp3.readADC() * 2);
    // Serial.println(" uV");
    // Serial.println(F("------------------------------------------------------------"));

    // Serial.println("----------MCP4 Readings----------");
    // Serial.print("Hot Junction: ");
    // Serial.println(sensor_data.temp_sense_ch4);
    // Serial.print("Cold Junction: ");
    // Serial.println(mcp4.readAmbient());
    // Serial.print("ADC: ");
    // Serial.print(mcp4.readADC() * 2);
    // Serial.println(" uV");
    // Serial.println(F("------------------------------------------------------------"));
}

void buzzer_handling(void)
{
    // Test play tone for one second
    tone(BUZZER_PIN, 2000, 1000); // Play a note for 1 second
}

void log_sequence_completion_to_sd(int probe_num, const char *test_type, float temp)
{
    File file = SD.open("/sequence_log.csv", FILE_APPEND);
    if (file)
    {
        file.printf("Probe %d,%s Complete,Final Temp: %.2f\n", probe_num, test_type, temp);
        file.close();
    }
}

void sequence_handling_data_capture(void)
{
    log_temperature_to_sd_card(
        sensor_data.temp_sense_ch1,
        sensor_data.temp_sense_ch2,
        sensor_data.temp_sense_ch3,
        sensor_data.temp_sense_ch4,
        sensor_data.temp_sense_ch5);
}

void sequence_handling(void)
{
    static bool waiting_for_flip[5] = {false, false, false, false, false};
    static float initial_z[5] = {0, 0, 0, 0, 0};
    float z = sensor_data.accelerometer_z;

    struct
    {
        float temp;
        bool *seq_active;
        char *seq_name;
        const char *label;
    } probes[5] = {
        {sensor_data.temp_sense_ch1, &CH1_seq_active, web_system_state.probe1_seq, "Probe 1"},
        {sensor_data.temp_sense_ch2, &CH2_seq_active, web_system_state.probe2_seq, "Probe 2"},
        {sensor_data.temp_sense_ch3, &CH3_seq_active, web_system_state.probe3_seq, "Probe 3"},
        {sensor_data.temp_sense_ch4, &CH4_seq_active, web_system_state.probe4_seq, "Probe 4"},
        {sensor_data.temp_sense_ch5, &CH5_seq_active, web_system_state.probe5_seq, "Probe 5"}};

    for (int i = 0; i < 5; i++)
    {
        if (*(probes[i].seq_active) && strcmp(probes[i].seq_name, "Complete") != 0)
        {
            // Boil Test
            if (strcmp(probes[i].seq_name, "Boil Test") == 0)
            {
                if (fabs(probes[i].temp - 30.0) <= 1.5)
                {
                    // Is within range for boiling test, trigger buzzer
                    if (!waiting_for_flip[i])
                    {
                        // Start buzzer and record initial z
                        tone(BUZZER_PIN, 3000, 500); // Short beep, will be retriggered
                        waiting_for_flip[i] = true;
                        initial_z[i] = z;

                        // RGB LED
                        set_rgb_color(1, 0, 0);
                    }
                    else
                    {
                        // Buzz everytime until flip is detected
                        tone(BUZZER_PIN, 3000, 500);

                        // RGB LED
                        if (digitalRead(LED_RED_PIN) == HIGH && digitalRead(LED_GREEN_PIN) == LOW && digitalRead(LED_BLUE_PIN) == LOW)
                        {
                            set_rgb_color(0, 1, 0); // GREEN
                        }
                        else if (digitalRead(LED_RED_PIN) == LOW && digitalRead(LED_GREEN_PIN) == HIGH && digitalRead(LED_BLUE_PIN) == LOW)
                        {
                            set_rgb_color(0, 0, 1); // BLUE
                        }
                        else if (digitalRead(LED_RED_PIN) == LOW && digitalRead(LED_GREEN_PIN) == LOW && digitalRead(LED_BLUE_PIN) == HIGH)
                        {
                            set_rgb_color(1, 0, 0); // RED
                        }

                        // Detect a z-axis change of more than 1.0
                        if ((initial_z[i] > 0 && z < 0) || (initial_z[i] < 0 && z > 0) || fabs(z - initial_z[i]) > 1.0)
                        {
                            // Flip detected, stop buzzer, complete sequence
                            noTone(BUZZER_PIN);
                            strcpy(probes[i].seq_name, "Complete");
                            *(probes[i].seq_active) = false;
                            waiting_for_flip[i] = false;
                            log_sequence_completion_to_sd(i + 1, "Boil Test", probes[i].temp);

                            // RGB LED
                            set_rgb_color(0, 0, 0); // All off
                        }
                    }

                    // strcpy(probes[i].seq_name, "Complete");
                    // *(probes[i].seq_active) = false;
                    // tone(BUZZER_PIN, NOTE_C4, 200); // Beep
                    // log_sequence_completion_to_sd(i + 1, "Boil Test", probes[i].temp);
                }
                else
                {
                    // Not within boiling range, reset waiting state
                    waiting_for_flip[i] = false;
                    noTone(BUZZER_PIN);     // Stop buzzer if not boiling
                    set_rgb_color(0, 0, 0); // All off
                }
            }

            // Ice Test
            else if (strcmp(probes[i].seq_name, "Ice Test") == 0)
            {
                if (fabs(probes[i].temp - 0.0) <= 1.5)
                {
                    // Is within range for ice test, trigger buzzer
                    if (!waiting_for_flip[i])
                    {
                        // Start buzzer and record initial z
                        tone(BUZZER_PIN, NOTE_C4, 100); // Short beep, will be retriggered
                        waiting_for_flip[i] = true;
                        initial_z[i] = z;
                    }
                    else
                    {
                        // Buzz everytime until flip is detected
                        tone(BUZZER_PIN, NOTE_C4, 100);

                        // Detect a z-axis change of more than 1.0
                        if ((initial_z[i] > 0 && z < 0) || (initial_z[i] < 0 && z > 0) || fabs(z - initial_z[i]) > 1.0)
                        {
                            // Flip detected, stop buzzer, complete sequence
                            noTone(BUZZER_PIN);
                            strcpy(probes[i].seq_name, "Complete");
                            *(probes[i].seq_active) = false;
                            waiting_for_flip[i] = false;
                            log_sequence_completion_to_sd(i + 1, "Ice Test", probes[i].temp);
                        }
                    }

                    // strcpy(probes[i].seq_name, "Complete");
                    // *(probes[i].seq_active) = false;
                    // tone(BUZZER_PIN, NOTE_C4, 200); // Beep
                    // log_sequence_completion_to_sd(i + 1, "Ice Test", probes[i].temp);
                }
                else
                {
                    // Not within ice range, reset waiting state
                    waiting_for_flip[i] = false;
                    noTone(BUZZER_PIN); // Stop buzzer if not ice
                }
            }
            else
            {
                waiting_for_flip[i] = false;
            }
        }
    }
}

void refresh_handling(void)
{
    // When the WebSocket page refreshes, send the relevant information like the logged temperatures of the probes and
    // which probes are still active
    web_system_state.webpage_load_graph_info = false;
}

void ip_address_start_sequence(void)
{
    ip_comm_state = IP_NEXT_CHAR;
    ip_comm_index = 0;
    ip_comm_color_phase = 0;
    ip_comm_timer = millis();
}

void ip_address_handling(void)
{
    static const unsigned long beep_duration = 140; // ms buzzer ON
    static const unsigned long beep_pause = 200;    // ms buzzer OFF between beeps
    static const unsigned long dot_duration = 600;  // ms for dot (white)
    static const unsigned long char_pause = 200;    // ms pause between chars

    if (ip_comm_state == IP_IDLE || IP_address_sequence_done)
        return;

    unsigned long now = millis();

    switch (ip_comm_state)
    {
    case IP_NEXT_CHAR:
    {
        if (ip_comm_index >= ip_address.length())
        {
            set_rgb_color(0, 0, 0); // All off
            ip_comm_state = IP_DONE;
            IP_address_sequence_done = true;
            break;
        }
        char c = ip_address[ip_comm_index];
        if (c == '.')
        {
            set_rgb_color(1, 1, 1); // White
            // noTone(BUZZER_PIN);
            ip_comm_timer = now;
            ip_comm_state = IP_DOT_PAUSE;
        }
        else if (c >= '0' && c <= '9')
        {
            // Set color: cycle red, green, blue
            if (ip_comm_color_phase == 0)
                set_rgb_color(1, 0, 0); // Red
            else if (ip_comm_color_phase == 1)
                set_rgb_color(0, 1, 0); // Green
            else
                set_rgb_color(0, 0, 1); // Blue

            ip_comm_beep_target = c - '0';
            ip_comm_beep_count = 0;
            ip_comm_timer = now;
            if (ip_comm_beep_target == 0)
            {
                // For '0', just pause with LED on, no beep
                ip_comm_state = IP_CHAR_PAUSE;
            }
            else
            {
                ip_comm_state = IP_BEEP_ON;
                tone(BUZZER_PIN, 2000, beep_duration);
            }
        }
        else
        {
            // Ignore unexpected chars
            ip_comm_index++;
        }
        break;
    }
    case IP_BEEP_ON:
        if (now - ip_comm_timer >= beep_duration)
        {
            // noTone(BUZZER_PIN);
            ip_comm_timer = now;
            ip_comm_state = IP_BEEP_OFF;
        }
        break;
    case IP_BEEP_OFF:
        if (now - ip_comm_timer >= beep_pause)
        {
            ip_comm_beep_count++;
            if (ip_comm_beep_count < ip_comm_beep_target)
            {
                tone(BUZZER_PIN, 2000, beep_duration);
                ip_comm_timer = now;
                ip_comm_state = IP_BEEP_ON;
            }
            else
            {
                ip_comm_timer = now;
                ip_comm_state = IP_CHAR_PAUSE;
            }
        }
        break;
    case IP_CHAR_PAUSE:
        if (now - ip_comm_timer >= char_pause)
        {
            ip_comm_index++;
            // Only increment color phase if previous char was a digit
            char prev = ip_address[ip_comm_index - 1];
            if (prev >= '0' && prev <= '9')
            {
                ip_comm_color_phase = (ip_comm_color_phase + 1) % 3;
            }
            ip_comm_state = IP_NEXT_CHAR;
        }
        break;
    case IP_DOT_PAUSE:
        if (now - ip_comm_timer >= dot_duration)
        {
            set_rgb_color(0, 0, 0); // All off
            // ip_comm_index++;
            ip_comm_state = IP_CHAR_PAUSE;
            ip_comm_timer = now;
        }
        break;
    case IP_DONE:
        set_rgb_color(0, 0, 0);
        // noTone(BUZZER_PIN);
        ip_comm_state = IP_IDLE;
        break;
    default:
        ip_comm_state = IP_IDLE;
        break;
    }
}

// --- Helper to set RGB color ---
void set_rgb_color(uint8_t r, uint8_t g, uint8_t b)
{
    digitalWrite(LED_RED_PIN, r ? HIGH : LOW);
    digitalWrite(LED_GREEN_PIN, g ? HIGH : LOW);
    digitalWrite(LED_BLUE_PIN, b ? HIGH : LOW);
}

void clear_temperature_log_file()
{
    File file = SD.open("/temp_log.csv", FILE_WRITE); // FILE_WRITE truncates the file
    if (file)
    {
        file.close();
        Serial.println("temp_log.csv cleared.");
    }
    else
    {
        Serial.println("Failed to clear temp_log.csv");
    }
}

void log_temperature_to_sd_card(float temp1, float temp2, float temp3, float temp4, float temp5)
{
    static unsigned long first_log_time = 0;
    if (first_log_time == 0)
    {
        first_log_time = millis();
    }
    unsigned long elapsed_seconds = (millis() - first_log_time) / 1000;

    File file = SD.open("/temp_log.csv", FILE_APPEND);

    if (!file)
    {
        Serial.println("Failed to open file for appending");
        return;
    }

    // Input header in csv file
    if (file.size() == 0)
    {
        file.println("timestamp,temp1,temp2,temp3,temp4,temp5");
        Serial.println("Header written to temp_log.csv");
    }

    // Format: timestamp, temp1, temp2, temp3, temp4, temp5
    file.printf("%lu,%.2f,%.2f,%.2f,%.2f,%.2f\n", elapsed_seconds, temp1, temp2, temp3, temp4, temp5);
    file.close();
}

void read_battery_voltage()
{
    // Configure ADC1_CH1 if not already done
    analogReadResolution(12);       // ESP32 default is 12 bits (0-4095)
    analogSetAttenuation(ADC_11db); // Allows input up to ~3.6V

    int raw = analogRead(BATTERY_ADC_PIN);
    float v_adc = (raw / 4095.0f) * 3.3f; // Convert ADC reading to voltage (V)
    float v_batt = v_adc * 2.0f;          // Compensate for voltage divider

    sensor_data.battery_voltage = v_batt;
}

void data_retrieval_test_handling()
{
    // // 1 Hz serial test output
    // Serial.print("[1Hz] Temps: ");
    // Serial.print("CH1 ");
    // Serial.print(sensor_data.temp_sense_ch1, 2);
    // Serial.print(" | CH2 ");
    // Serial.print(sensor_data.temp_sense_ch2, 2);
    // Serial.print(" | CH3 ");
    // Serial.print(sensor_data.temp_sense_ch3, 2);
    // Serial.print(" | CH4 ");
    // Serial.print(sensor_data.temp_sense_ch4, 2);
    // Serial.print(" | CH5 ");
    // Serial.println(sensor_data.temp_sense_ch5, 2);

    // Serial.print("Accel: X ");
    // Serial.print(sensor_data.accelerometer_x, 2);
    // Serial.print(" | Y ");
    // Serial.print(sensor_data.accelerometer_y, 2);
    // Serial.print(" | Z ");
    // Serial.println(sensor_data.accelerometer_z, 2);

    // Serial.print("Battery: ");
    // Serial.println(sensor_data.battery_voltage, 2);

    // Serial.print("System Sequence Status: ");
    // Serial.print("CH1 ");
    // Serial.print(web_system_state.probe1_seq);
    // Serial.print(" | CH2 ");
    // Serial.print(web_system_state.probe2_seq);
    // Serial.print(" | CH3 ");
    // Serial.print(web_system_state.probe3_seq);
    // Serial.print(" | CH4 ");
    // Serial.print(web_system_state.probe4_seq);
    // Serial.print(" | CH5 ");
    // Serial.println(web_system_state.probe5_seq);

    Serial.println("System Sequence Status:");
    Serial.print("                    CH1 ");
    Serial.print(web_system_state.probe1_seq);
    Serial.print(" (active: ");
    Serial.print(CH1_seq_active ? "true" : "false");
    Serial.println(")");
    Serial.print("                    CH2 ");
    Serial.print(web_system_state.probe2_seq);
    Serial.print(" (active: ");
    Serial.print(CH2_seq_active ? "true" : "false");
    Serial.println(")");
    Serial.print("                    CH3 ");
    Serial.print(web_system_state.probe3_seq);
    Serial.print(" (active: ");
    Serial.print(CH3_seq_active ? "true" : "false");
    Serial.println(")");
    Serial.print("                    CH4 ");
    Serial.print(web_system_state.probe4_seq);
    Serial.print(" (active: ");
    Serial.print(CH4_seq_active ? "true" : "false");
    Serial.println(")");
    Serial.print("                    CH5 ");
    Serial.print(web_system_state.probe5_seq);
    Serial.print(" (active: ");
    Serial.print(CH5_seq_active ? "true" : "false");
    Serial.println(")");
}