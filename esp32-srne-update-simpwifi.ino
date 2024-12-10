const char compile_date[] = __DATE__;
const char compile_time[] = __TIME__;

#include <WiFi.h>
//#include <time.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>
#include <ArduinoOTA.h>
#include "FS.h"
#include "SPIFFS.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>
#include <ModbusMaster.h>
#include "esp_task_wdt.h"
#include <ESPmDNS.h>
#include "index_html.h"
#include "script_js.h"
#include "esp_wifi.h"


// Wi-Fi credentials
const char* primarySSID = "freedom";
const char* primaryPassword = "8888888";
const char* fallbackSSID = "littlesugar";
const char* fallbackPassword = "888888888";

// WiFi connection timing
const unsigned long PRIMARY_CONNECT_TIME = 20000; // 20sec for primary network
const unsigned long FALLBACK_CONNECT_TIME = 10000; // 10sec for fallback network
unsigned long wifiConnectStartMillis = 0;
bool tryingPrimary = true; // Start by trying the primary network
bool connectedToAnyNetwork = false;

// // NTP stuff
// WiFiUDP ntpUDP;
// NTPClient timeClient(ntpUDP, "pool.ntp.org");
// time_t lastSyncTime = 0;
// const time_t SECONDS_IN_DAY = 86400;  // Seconds in one day


// BLE
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

String readModbusRegisterHex(uint16_t address, uint32_t &decimalValue);

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

AsyncWebServer server(80);

Preferences preferences;
String currentBLEName = "ESP32toSRNE-ModBUS"; // Default BLE name
uint8_t modbusSlaveAddress = 1; // Default Modbus slave address

// Modbus
#define RX_PIN 16  // RX pin for Modbus communication
#define TX_PIN 17  // TX pin for Modbus communication
#define FLOW_CONTROL_PIN 5  // D5 for flow control
#define MODBUS_BAUDRATE 9600  // Assuming this is the correct baud rate for SRNE v1.96
//#define MODBUS_SLAVE_ID 1     // Assuming this is the correct slave ID
#define DEBUG_MODBUS 1  // Enable Modbus debug logging
ModbusMaster node;

// Global variable to track OTA progress
volatile bool otaProgress = false;

// Add global variables to store the latest Modbus readings
int latestBatterySOC = 0;
float latestBatteryVoltage = NAN;
float latestBatteryCurrent = NAN;

// Timers for non-blocking operations
unsigned long lastModbusRead = 0;
unsigned long lastModbusError = 0;
unsigned long lastBLECheck = 0;
const unsigned long MODBUS_READ_INTERVAL = 30000; // Changed to 30 seconds
const unsigned long MODBUS_RETRY_INTERVAL = 100; // 100ms for retry, similar to ESPHome's command_throttle
const unsigned long BLE_CHECK_INTERVAL = 1000;  // 1 second
const unsigned long MODBUS_SEND_WAIT_TIME = 100; // 100ms wait time before next send
unsigned long lastModbusSend = 0; // Time keeper for last send operation

// Structure to hold Modbus query information
struct ModbusQuery {
    String address;  // Hex address
    String hexResponse;  // Hex response
    uint32_t decimalResponse;  // Decimal response
};

std::vector<ModbusQuery> lastQueries;

struct ModbusData {
    uint16_t address;
    const char* name;
    float value;
    const char* unit;
    char description[60];
    bool rw; // Read/Write capability
    bool rt; // Real-time update
    bool isInteger; // Flag to handle integer vs float
    char ass; // Setting association of the value for layout and distro
};

#define MODBUS_DATA(addr, n, u, d, r, t, i, a) {addr, n, NAN, u, d, r, t, i, a}

static ModbusData modbusData[28] = {
    MODBUS_DATA(0x100, "Battery SOC", "%", "Battery State of Charge", false, true, true, 'b'),
    MODBUS_DATA(0x101, "Battery Voltage", "V", "Battery Voltage", false, true, false, 'b'),
    MODBUS_DATA(0x102, "Battery Current", "A", "Battery Current", false, true, false, 'b'),
    MODBUS_DATA(0x107, "PV1 Voltage", "V", "PV panel 1 voltage", false, true, false, 's'),
    MODBUS_DATA(0x108, "PV1 Current", "A", "PV panel 1 current", false, true, false, 's'),
    MODBUS_DATA(0x109, "PV1 Power", "W", "PV panel 1 power", false, true, true, 's'),
    MODBUS_DATA(0x10B, "Charge State", "", "Charge state", false, false, true, 'i'),
    MODBUS_DATA(0x10E, "Max Charge Power", "W", "Total charge power, include charge power by mains and PV", false, false, true, 'i'),
    MODBUS_DATA(0x210, "Machine State", "", "Machine state", false, false, true, 'i'),
    MODBUS_DATA(0x213, "Grid Voltage A", "V", "Mains voltage phase A", false, true, false, 'g'),
    MODBUS_DATA(0x214, "Grid Current A", "A", "Mains side input current phase A", false, true, false, 'g'),
    MODBUS_DATA(0x215, "Grid Frequency", "Hz", "Mains frequency", false, true, false, 'g'),
    MODBUS_DATA(0x216, "Inverter Voltage A", "V", "Inverter output voltage phase A", false, true, false, 'i'),
    MODBUS_DATA(0x217, "Inverter Current A", "A", "Inverter inductive current phase A", false, true, false, 'i'),
    MODBUS_DATA(0x218, "Inverter Frequency", "Hz", "Inverter frequency", false, true, false, 'i'),
    MODBUS_DATA(0x219, "Load Current A", "A", "Load side current phase A", false, true, false, 'i'),
    MODBUS_DATA(0x21B, "Load Active Power A", "W", "Load active power phase A", false, true, true, 'i'),
    MODBUS_DATA(0x21E, "Mains Charge Current", "A", "Battery side current when charging on mains", false, true, false, 'g'),
    MODBUS_DATA(0x21F, "Load Ratio A", "%", "Load percentage phase A", false, true, true, 'i'),
    MODBUS_DATA(0x220, "Heat Sink A Temp", "℃", "DC-DC heat sink temperature", false, true, false, 'i'),
    MODBUS_DATA(0x221, "Heat Sink B Temp", "℃", "DC-AC heat sink temperature", false, true, false, 'i'),
    MODBUS_DATA(0x222, "Transformer temperature", "℃", "Transformer temperature", false, true, false, 'i'),
    MODBUS_DATA(0x223, "Environment temperature", "℃", "Environment temperature", false, true, false, 'i'),
    MODBUS_DATA(0x224, "PV Charge Current", "A", "Battery side current by PV charging", false, true, false, 's'),
    MODBUS_DATA(0xE120, "PV Charge Current Set", "A", "Set Max PV Charger Current in Amps", true, false, false, 'i'),
    MODBUS_DATA(0xE205, "Grid Charge Current Limit", "A", "Maximum Grid charge current (0-40A)", true, false, false, 'i'),
    MODBUS_DATA(0xE20A, "Total Max Charge Limit", "A", "Total Maximum Charge Current Limit", true, false, false, 'i'),
    MODBUS_DATA(0xE204, "Output Priority", "", "Output Priority: solar, line, sbu", true, false, true, 'i')
};

struct SettingResponse {
  String setting;
  String response;
};

std::vector<SettingResponse> settingResponses;

void addSettingResponse(const String& setting, const String& response) {
  SettingResponse newResponse = {setting, response};
  settingResponses.insert(settingResponses.begin(), newResponse); // Insert at the beginning for most recent first
  if (settingResponses.size() > 10) {
    settingResponses.pop_back(); // Remove the oldest if we have more than 10
  }
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("BLE Client Connected");
    };

void onDisconnect(BLEServer* pServer) {
  deviceConnected = false;
  Serial.println("BLE Client Disconnected");
}
};

void changeBLEName(String newName) {
  currentBLEName = newName;
  preferences.begin("ble", false); // Open namespace "ble" for write
  preferences.putString("deviceName", currentBLEName);
  preferences.end();

  if (pServer) {
    pServer->getAdvertising()->stop();
    BLEDevice::deinit(false);
    BLEDevice::init(newName.c_str());
    pServer->getAdvertising()->start();
    Serial.println("BLE name changed to: " + newName);
  }

  // Update mDNS name
  MDNS.end(); // Stop current mDNS
  if (!MDNS.begin(newName.c_str())) {
    Serial.println("Error setting up MDNS responder after name change!");
  } else {
    Serial.println("mDNS name updated");
    MDNS.addService("ble", "tcp", 0); // Re-advertise service with new name
  }
}

// Functions for flow control
void preTransmission() {
  digitalWrite(FLOW_CONTROL_PIN, HIGH); // Switch to transmit mode
  delay(1); // Small delay for direction change
}

void postTransmission() {
  digitalWrite(FLOW_CONTROL_PIN, LOW); // Switch back to receive mode
  delay(1); // Small delay for direction change
}

// New global variable for modbus retry timing
unsigned long lastRetry = 0;

// Modbus lookup from data vector
int findIndexByAddress(uint16_t address) {
    for (size_t i = 0; i < 28; ++i) { // Use the known size of the array
        if (modbusData[i].address == address) {
            return i;
        }
    }
    return -1; // Return -1 if no match found, indicating an error
}

// Helper functions for conversion
int convertToInt(uint16_t value) {
  return static_cast<int>(value);
}

float convertToFloat(uint16_t value) {
  return static_cast<float>(value * 0.1);
}

// Modified Modbus read function with send_wait_time
uint16_t modbusReadRegister(uint16_t address) {
    static unsigned long lastRetry = 0;
    unsigned long now = millis();
    
    if (now - lastRetry >= MODBUS_RETRY_INTERVAL && now - lastModbusSend >= MODBUS_SEND_WAIT_TIME) {
        lastRetry = now;
        lastModbusSend = now;
        uint8_t result = node.readHoldingRegisters(address, 1);
        if (result == node.ku8MBSuccess) {
            lastModbusError = 0; // Reset error timer on success
            return node.getResponseBuffer(0);
        } else {
            Serial.printf("Failed to read from register 0x%X, error code: %d\n", address, result);
            if (lastModbusError == 0) {
                lastModbusError = millis(); // Record first error time if not already set
            }
            return 0; // or any other value indicating failure
        }
    }
    return 0; // Default return if not time to retry or if too soon after last send
}

// Function to read all Modbus registers
void readModbusRegisters() {
    if (otaProgress) return;
    char buffer[20]; // Define buffer with adequate size for formatting
    for (auto &data : modbusData) {
        uint32_t decimalValue;
        String hexResult = readModbusRegisterHex(data.address, decimalValue);
        vTaskDelay(3 / portTICK_PERIOD_MS); // Delay for 3 millisecond
        if (hexResult != "Timeout" && hexResult != "0x0") {
            switch (data.address) {
                case 0x100: // Battery SOC - Unsigned integer, no scaling needed
                    data.value = static_cast<float>(decimalValue);
                    break;
                case 0x101: // Battery Voltage - 0.1 V %.1fV
                    data.value = static_cast<float>(decimalValue) * 0.1f;
                    break;
                case 0x102: // Battery Current - 0.1 A %.1fA
                    data.value = static_cast<float>(static_cast<int16_t>(decimalValue)) * 0.1f;
                    break;
                case 0x107: // PV1 Voltage - 0.1 V %.1fV
                    data.value = static_cast<float>(decimalValue) * 0.1f;
                    break;
                case 0x108: // PV1 Current - 0.1 A %.1fA
                    data.value = static_cast<float>(static_cast<int16_t>(decimalValue)) * 0.1f;
                    break;
                case 0x109: // PV1 Power - 1 W %d
                    data.value = static_cast<float>(decimalValue); 
                    break;
                case 0x213: // Grid Voltage A - 0.1 V %.1fV
                    data.value = static_cast<float>(decimalValue) * 0.1f;
                    break;
                case 0x214: // Grid Current A - 0.1 A %.1fA
                    data.value = static_cast<float>(static_cast<int16_t>(decimalValue)) * 0.1f;
                    break;
                case 0x215: // Grid Frequency - 0.01 Hz %.2fHz
                    data.value = static_cast<float>(decimalValue) * 0.01f;
                    break;
                case 0x217: // Inverter Current A - 0.1 A %.1fA
                    data.value = static_cast<float>(static_cast<int16_t>(decimalValue)) * 0.1f;
                    break;
                case 0x219: // Load Current A - 0.1 A %.1fA
                    data.value = static_cast<float>(static_cast<int16_t>(decimalValue)) * 0.1f;
                    break;
                case 0x220: // Heat Sink A Temp - 0.1 °C %.1f°C
                    data.value = static_cast<float>(static_cast<int16_t>(decimalValue)) * 0.1f;
                    break;
                case 0x21E: // Mains Charge Current - 0.1 A %.1fA
                    data.value = static_cast<float>(static_cast<int16_t>(decimalValue)) * 0.1f;
                    break;
                case 0x216: // Inverter Voltage A - 0.1 V %.1fV
                    data.value = static_cast<float>(decimalValue) * 0.1f;
                    break;
                case 0x10E: // Charge Power - 1 W %dW
                    data.value = static_cast<float>(decimalValue); // Assuming no scaling for power
                    break;
                case 0x10B: // Charge State - %d
                    switch (decimalValue) {
                        case 0x0000: data.value = 0; strcpy(data.description, "Charge off"); break;
                        case 0x0001: data.value = 1; strcpy(data.description, "Quick charge"); break;
                        case 0x0002: data.value = 2; strcpy(data.description, "Constant voltage charge"); break;
                        case 0x0004: data.value = 4; strcpy(data.description, "Float charge"); break;
                        case 0x0005: data.value = 5; strcpy(data.description, "Reserved"); break;
                        case 0x0006: data.value = 6; strcpy(data.description, "Li battery activate"); break;
                        case 0x0007: data.value = 7; strcpy(data.description, "Reserved"); break;
                        default: data.value = NAN; strcpy(data.description, "Unknown"); break;
                    }
                    break;
                case 0x210: // Machine State - %d
                    switch (decimalValue) {
                        case 0:  data.value = 0; strcpy(data.description, "Power-up delay"); break;
                        case 1:  data.value = 1; strcpy(data.description, "Waiting state"); break;
                        case 2:  data.value = 2; strcpy(data.description, "Initialization"); break;
                        case 3:  data.value = 3; strcpy(data.description, "Soft start"); break;
                        case 4:  data.value = 4; strcpy(data.description, "Mains powered operation"); break;
                        case 5:  data.value = 5; strcpy(data.description, "Inverter powered operation"); break;
                        case 6:  data.value = 6; strcpy(data.description, "Inverter to mains"); break;
                        case 7:  data.value = 7; strcpy(data.description, "Mains to inverter"); break;
                        case 8:  data.value = 8; strcpy(data.description, "Battery activate"); break;
                        case 9:  data.value = 9; strcpy(data.description, "Shutdown by user"); break;
                        case 10: data.value = 10; strcpy(data.description, "Fault"); break;
                        default: data.value = NAN; strcpy(data.description, "Unknown"); break;
                    }
                    break;
                case 0x218: // Inverter Frequency - 0.01 Hz %.2fHz
                    data.value = static_cast<float>(decimalValue) * 0.01f;
                    break;
                case 0x21B: // Load Active Power A - 1 W %dW
                    data.value = static_cast<float>(decimalValue); // Assuming no scaling for power
                    break;
                case 0x21F: // Load Ratio A - 1 % %d%
                    data.value = static_cast<float>(decimalValue);
                    break;
                case 0x221: // Heat Sink B Temp - 0.1 °C %.1f°C
                    data.value = static_cast<float>(static_cast<int16_t>(decimalValue)) * 0.1f;
                    break;
                case 0x224: // PV Charge Current - 0.1 A %.1fA
                    data.value = static_cast<float>(static_cast<int16_t>(decimalValue)) * 0.1f;
                    break;
                case 0xE120: // PV Charge Current Set - 0.1 A %.1fA
                    data.value = static_cast<float>(decimalValue) * 0.1f; // Read as *10 for this register
                    break;
                case 0xE205: // Grid Charge Current Limit - 0.1 A %.1fA but read as *10
                    data.value = static_cast<float>(decimalValue) / 10.0f; // Divide by 10 to get actual current in A
                    break;
                case 0xE20A: // Total Max Charge Limit - 0.1 A %.1fA
                    data.value = static_cast<float>(decimalValue) * 0.1f; // Convert back from register value
                    break;
                case 0x222: // Transformer temperature - 0.1 °C %.1f℃
                    data.value = static_cast<float>(static_cast<int16_t>(decimalValue)) * 0.1f; // Convert to signed and scale by 0.1
                    break;
                case 0x223: // Environment temperature - 0.1 °C %.1f℃
                    data.value = static_cast<float>(static_cast<int16_t>(decimalValue)) * 0.1f; // Convert to signed and scale by 0.1
                    break;
                case 0xE204: // Output Priority - %d
                    switch (decimalValue) {
                        case 0: 
                            data.value = 0; 
                            strcpy(data.description, "Solar");
                            break;
                        case 1: 
                            data.value = 1; 
                            strcpy(data.description, "Utility");
                            break;
                        case 2: 
                            data.value = 2; 
                            strcpy(data.description, "Solar-Battery");
                            break;
                        default: 
                            data.value = NAN; 
                            strcpy(data.description, "Unknown");
                            break;
                    }
                    break;
                default:
                    data.value = static_cast<float>(decimalValue); // Default to no scaling
                    snprintf(buffer, sizeof(buffer), "%.2f", data.value); // Default formatting if not specified
                    strncpy(data.description, buffer, sizeof(data.description) - 1);
                    data.description[sizeof(data.description) - 1] = '\0';
                    break;
            }
        } else {
            data.value = NAN; // Indicate failure
          //  strcpy(data.description, "N/A");
        }
    }
}


// New function to read Modbus register in hex format with integrated logging and cooldown after 10 consecutive timeouts
String readModbusRegisterHex(uint16_t address, uint32_t &decimalValue) {
  static unsigned long lastModbusTimeout = 0;
  static uint8_t timeoutCount = 0;
  static bool inCooldown = false; 
  unsigned long start_time = millis();
  uint16_t rawValue = 0;
  const unsigned long COOLDOWN_TIME = 60000; // 1 minute cooldown

  if (millis() - lastModbusTimeout < COOLDOWN_TIME && timeoutCount >= 10) {
    if (!inCooldown) {
      inCooldown = true;
      Serial.println("Entering cooldown after 10 consecutive timeouts");
    }
    return "Timeout";
  } else {
    if (inCooldown) {
      inCooldown = false;
      Serial.println("Exiting Modbus cooldown period");
      timeoutCount = 0; // Reset timeout count when exiting cooldown
      lastModbusTimeout = 0; // Reset last timeout time to allow immediate read attempts
    }
  }

  while (millis() - start_time < 500) {
    rawValue = modbusReadRegister(address);
    if (rawValue != 0) {
      Serial.print("Successfully read value: ");
      Serial.println(rawValue, HEX);
      decimalValue = static_cast<uint32_t>(rawValue);
      timeoutCount = 0; // Reset on success
      return "0x" + String(rawValue, HEX);
    }
    delay(50); // Small delay to prevent busy-waiting
  }

  // If we reach here, there was a timeout
  Serial.println("Modbus read operation timed out.");
  decimalValue = 0;
  timeoutCount++;

  if (timeoutCount >= 10) {
    lastModbusTimeout = millis();
    if (!inCooldown) {
      Serial.println("Entering cooldown after 10 consecutive timeouts");
      inCooldown = true;
    }
  }

  return "Timeout"; 
}

void writeModbusRegister(uint16_t address, float value, String& responseMessage) {
    unsigned long now = millis();
    if (now - lastRetry >= MODBUS_RETRY_INTERVAL && now - lastModbusSend >= MODBUS_SEND_WAIT_TIME) {
        lastRetry = now;
        lastModbusSend = now;
        
        uint16_t scaledValue = 0;
        if (address == 0xE204) { 
            // Output Priority, no scaling needed
            scaledValue = static_cast<uint16_t>(value);
        } else if (address == 0xE205 || address == 0xE120 || address == 0xE20A) {
            // For E205, E120, and E20A, multiply by 10 for scaling
            scaledValue = static_cast<uint16_t>(value * 10);
        } else {
            responseMessage = "Unknown register address: " + String(address, HEX);
            return;  // Early exit for unknown registers
        }

        uint8_t result = node.writeSingleRegister(address, scaledValue);
        if (result != node.ku8MBSuccess) {
            Serial.print("Failed to write to register: ");
            Serial.println(address, HEX);
            responseMessage = "Failed to write to register: " + String(address, HEX) + ", Error code: " + String(result);
            if (lastModbusError == 0) {
                lastModbusError = millis(); // Record first error time if not already set
            }
        } else {
            Serial.print("Successfully wrote to register: ");
            Serial.println(address, HEX);
            responseMessage = "Successfully wrote to register: " + String(address, HEX);
            lastModbusError = 0; // Reset error timer on success
        }
    } else {
        responseMessage = "Modbus operation too soon after last command. Please wait.";
    }
}

//tracking and logging mem usage
void saveMemoryData() {
    size_t freeHeap = ESP.getFreeHeap();
    unsigned long currentTime = millis() / 1000; // convert to seconds

    if (!preferences.begin("memStats", false)) {
        Serial.println("Failed to open preferences");
        return;
    }

    int numberOfRecords = preferences.getInt("numRecords", 0);
    int nextRecord = preferences.getInt("nextRecord", 0);

    char record[50];  // Adjust size as needed
    sprintf(record, "%lu,%zu", currentTime, freeHeap);

    if (numberOfRecords < 864) {  // 3 days * 24 hours * 12 (5-minute intervals per hour)
        String key = "record_" + String(nextRecord);
        preferences.putString(key.c_str(), record);
        preferences.putInt("numRecords", ++numberOfRecords);
    } else {
        // Use circular buffer approach
        String key = "record_" + String(nextRecord);
        preferences.putString(key.c_str(), record);
    }

    nextRecord = (nextRecord + 1) % 864;  // Circular buffer for 864 records
    preferences.putInt("nextRecord", nextRecord);

    preferences.end();
    Serial.printf("Memory record saved: %s\n", record);
}

void handleFileUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){
  if(!index){
    Serial.printf("Upload Start: %s\n", filename.c_str());
    if(!SPIFFS.exists(filename)){
      request->_tempFile = SPIFFS.open("/" + filename, "w");
    } else {
      request->_tempFile = SPIFFS.open("/" + filename, "a");
    }
  }
  if(len){
    request->_tempFile.write(data, len);
  }
  if(final){
    request->_tempFile.close();
    Serial.printf("Upload End: %s, %u Bytes\n", filename.c_str(), index+len);
  }
}

// Make size of files human readable
String humanReadableSize(const size_t bytes) {
  if (bytes < 1024) return String(bytes) + " B";
  else if (bytes < (1024 * 1024)) return String(bytes / 1024.0) + " KB";
  else if (bytes < (1024 * 1024 * 1024)) return String(bytes / 1024.0 / 1024.0) + " MB";
  else return String(bytes / 1024.0 / 1024.0 / 1024.0) + " GB";
}

// list all of the files, if ishtml=true, return html rather than simple text
String listFiles(bool ishtml) {
  String returnText = "";
  Serial.println("Listing files stored on SPIFFS");
  File root = SPIFFS.open("/");
  File foundfile = root.openNextFile();
  
  if (ishtml) {
    returnText += "<table><tr><th align='left'>Name</th><th align='left'>Size</th><th align='left'>Modified</th></tr>";
  }
  
  while (foundfile) {
    if (ishtml) {
      returnText += "<tr align='left'>";
      returnText += "<td>" + String(foundfile.name()) + "</td>";
      returnText += "<td>" + humanReadableSize(foundfile.size()) + "</td>";
      returnText += "<td>" + getDateTime(foundfile) + "</td>";
      returnText += "</tr>";
    } else {
      returnText += "File: " + String(foundfile.name()) + ", Size: " + humanReadableSize(foundfile.size()) + ", Modified: " + getDateTime(foundfile) + "\n";
    }
    foundfile = root.openNextFile();
  }
  
  if (ishtml) {
    returnText += "</table>";
  }
  
  root.close();
  foundfile.close();
  return returnText;
}

// Helper function to get human-readable date and time
String getDateTime(File file) {
    time_t now;
    struct tm info;
    char buf[32];

    time(&now);
    localtime_r(&now, &info); // Get local time

    // Get file modification time
    time_t modTime = file.getLastWrite();
    localtime_r(&modTime, &info);

    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &info);
    return String(buf);
}

// void syncTime() {
//   // Force an update from NTP
//   timeClient.update();
  
//   time_t now;
//   struct tm timeinfo;
  
//   if(!getLocalTime(&timeinfo)) {
//     Serial.println("Failed to obtain time");
//     return;
//   }
  
//   time(&now);  // Get current time
  
//   // Apply the new time to the system
//   Serial.print("Current local time: ");
//   Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  
//   // This ensures the system time is updated
//   struct timeval tv = {now, 0};
//   settimeofday(&tv, NULL);
// }

// // Simplified Daylight Saving Time check for Central Time Zone
// bool isDaylightSaving() {
//   time_t now;
//   time(&now);
//   struct tm timeinfo;
//   localtime_r(&now, &timeinfo);
  
//   int month = timeinfo.tm_mon + 1; // tm_mon is 0-11, so we add 1
//   int day = timeinfo.tm_mday;
//   int dayOfWeek = timeinfo.tm_wday; // Sunday is 0

//   if (month < 3 || month > 11) return false; // No DST in January, February, December
//   if (month > 3 && month < 11) return true; // DST in April through October

//   // In March, check if it's after the second Sunday
//   if (month == 3) {
//     int firstSunday = 14 - dayOfWeek; // First Sunday in March
//     return day > firstSunday;
//   }

//   // In November, check if it's before the first Sunday
//   if (month == 11) {
//     int firstSunday = 7 - dayOfWeek; // First Sunday in November
//     return day < firstSunday;
//   }
//   return false;
// }

void setup() {
  Serial.begin(115200);

  esp_task_wdt_deinit(); //wdt is initialized by default. disable and reconfig
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 120000,  // 2m timeout
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, // All cores
    .trigger_panic = true  // Do not trigger panic on timeout, just warn
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);

  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  preferences.begin("ble", true); 
  currentBLEName = preferences.getString("deviceName", "ESP32toSRNE-ModBUS");
  preferences.end();
  
  preferences.begin("modbus", true); 
  modbusSlaveAddress = preferences.getUChar("slaveAddress", 1); // Default to 1 if not set
  preferences.end();

  preferences.begin("memStats", false); // Open for write
  int numRecords = preferences.getInt("numRecords", 0);
  int nextRecord = preferences.getInt("nextRecord", 0);
  
  // If we have more records than 3 days worth, clear old ones
  if (numRecords > 864) { // 864 records for 3 days at 5-minute intervals
      for (int i = 0; i < numRecords - 864; ++i) {
          String key = "record_" + String((nextRecord + i) % 864);
          preferences.remove(key.c_str());
      }
      preferences.putInt("numRecords", 864); // Reset to 3 days worth of records
  }

  preferences.end();

  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE); // Disable power save mode for WiFi

  // Attempt primary WiFi connection
  WiFi.begin(primarySSID, primaryPassword);
  wifiConnectStartMillis = millis();
  Serial.println("Connecting to primary WiFi...");
  unsigned long primaryAttemptEnd = millis() + PRIMARY_CONNECT_TIME;

  while (WiFi.status() != WL_CONNECTED && millis() < primaryAttemptEnd) {
    delay(1000);
    Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nPrimary WiFi connection failed, trying fallback...");
    WiFi.begin(fallbackSSID, fallbackPassword);
    unsigned long fallbackAttemptEnd = millis() + FALLBACK_CONNECT_TIME;
    
    while (WiFi.status() != WL_CONNECTED && millis() < fallbackAttemptEnd) {
      delay(1000);
      Serial.print(".");
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi.");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    connectedToAnyNetwork = true;
    tryingPrimary = WiFi.SSID() == primarySSID; // Determine if we are on primary or fallback
        // Initiate mDNS here since we've confirmed WiFi connection
    if (!MDNS.begin(currentBLEName.c_str())) {    
      Serial.println("Error setting up MDNS responder!");
    } else {
      Serial.println("mDNS responder started");
      MDNS.addService("ble", "tcp", 0);
    }
  } else {
    Serial.println("\nFailed to connect to any WiFi network.");
    connectedToAnyNetwork = false;
    tryingPrimary = true;  // Reset to attempt primary next time
  }

    // //NTP Setup and initial sync
    // configTime(0, 0, "pool.ntp.org"); // This will sync time once its called
    // setenv("TZ", "CST6CDT,M3.2.0,M11.1.0", 1); // Set timezone for Central Time with DST rules
    // tzset(); // Apply the timezone setting
    // // Initial sync
    // syncTime();

  ArduinoOTA.setHostname(currentBLEName.c_str());
  ArduinoOTA.setPassword("ota-7279");

  ArduinoOTA.onStart([]() {
    otaProgress = true;
    esp_task_wdt_deinit(); // Disable watchdog during update
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) 
      type = "sketch";
    else 
      type = "filesystem";
    Serial.println("Start updating " + type);
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
    esp_task_wdt_config_t wdt_config = {
      .timeout_ms = 120000,  // 2m timeout
      .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, // All cores
      .trigger_panic = true  // Do not trigger panic on timeout, just warn
    };
    esp_task_wdt_init(&wdt_config); // Re-enable watchdog
    esp_task_wdt_add(NULL);    
    otaProgress = false;
  });

  ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) {
          Serial.println("Receive Failed - Rebooting...");
          delay(1000);  // Small delay to ensure the message is sent
          ESP.restart();
      } else if (error == OTA_END_ERROR) {
          Serial.println("End Failed - Rebooting...");
          delay(1000);  // Small delay to ensure the message is sent
          ESP.restart();
      }
      // Re-enable watchdog only if it was disabled at the start of OTA
      esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 120000,  // 2m timeout
        .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, // All cores
        .trigger_panic = true  // Do not trigger panic on timeout, just warn
      };
      esp_task_wdt_init(&wdt_config); 
      esp_task_wdt_add(NULL); 
      otaProgress = false;  // Reset OTA in progress flag
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.begin();
  Serial.println("OTA Initialized");

  BLEDevice::init(currentBLEName.c_str());
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("Waiting for a client connection to notify...");

  pinMode(FLOW_CONTROL_PIN, OUTPUT);
  digitalWrite(FLOW_CONTROL_PIN, LOW);
  Serial2.begin(MODBUS_BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);
  node.begin(modbusSlaveAddress, Serial2);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  Serial.println("Modbus communication initialized with slave ID: " + String(modbusSlaveAddress));

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200, "text/html", index_html);
  });


  server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *request) {
      AsyncWebServerResponse *response = request->beginChunkedResponse("text/javascript", [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
          // Copy data into the buffer from PROGMEM
          size_t len = strlen_P(script_js);
          if (index >= len) {
              return 0; // No more data to send
          }
          size_t chunkSize = min(maxLen, len - index);
          memcpy_P((void*)buffer, script_js + index, chunkSize);
          return chunkSize;
      });
      response->addHeader("Cache-Control", "no-cache"); // Optional, adjust as needed
      request->send(response);
  });
  // server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *request){
  //     request->send_P(200, "text/javascript", script_js);
  // });
  // server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *request){
  //     File file = SPIFFS.open("/script.js", "r");
  //     if(file){
  //         AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/script.js", "text/javascript");
  //         // Set max content length to something larger than your script.js file size
  //         response->setContentLength(file.size()); // This effectively sets maxContentLength to the file size
  //         request->send(response);
  //         file.close();
  //     } else {
  //         request->send(404, "text/plain", "File not found");
  //     }
  // });

  server.on("/get_blename", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", currentBLEName);
  });

  server.on("/change_blename", HTTP_POST, [](AsyncWebServerRequest *request){
    if (request->hasParam("blename", true)) {
      String newName = request->getParam("blename", true)->value();
      changeBLEName(newName);
      request->send(200, "text/plain", "BLE Name Changed to: " + newName);
    } else {
      request->send(400, "text/plain", "No BLE name provided");
    }
  });

server.on("/battery_status", HTTP_GET, [](AsyncWebServerRequest *request){
    String response = "[";
    for (size_t i = 0; i < 28; ++i) { 
        response += "{\"name\":\"" + String(modbusData[i].name) + "\",";
        response += "\"value\":" + (isnan(modbusData[i].value) ? "null" : String(modbusData[i].value));
        response += ",\"unit\":\"" + String(modbusData[i].unit) + "\",";
        response += "\"description\":\"" + String(modbusData[i].description) + "\",";
        response += "\"isInteger\":" + String(modbusData[i].isInteger ? "true" : "false");
        response += ",\"ass\":\"" + String(modbusData[i].ass) + "\""; // Add association
        response += "}";
        if (i < 27) response += ","; 
    }
    response += "]";
    request->send(200, "application/json", response);
});

server.on("/update_registers", HTTP_POST, [](AsyncWebServerRequest *request){
    String responseMessage;
    bool retry = false;
    do {
        retry = false;
        if (request->hasParam("gridChargeCurrent", true)) {
            float gridChargeCurrent = request->getParam("gridChargeCurrent", true)->value().toFloat();
            if (gridChargeCurrent >= 0 && gridChargeCurrent <= 40) {
                writeModbusRegister(0xE205, gridChargeCurrent, responseMessage);
                if (responseMessage.startsWith("Modbus operation too soon after last command. Please wait.")) {
                    delay(1000); // Delay for 1 second
                    retry = true; // Flag to retry the operation
                } else if (responseMessage.startsWith("Successfully")) {
                    responseMessage = "Grid Charge Current Limit updated to: " + String(gridChargeCurrent, 1) + "A";
                } else {
                    responseMessage = "Error updating Grid Charge Current Limit: " + responseMessage;
                }
                if (!retry) addSettingResponse("Grid Charge Current", responseMessage);
            } else {
                responseMessage = "Invalid value for Grid Charge Current Limit. Must be between 0 and 40.";
                addSettingResponse("Grid Charge Current", responseMessage);
            }
        } else if (request->hasParam("outputPriority", true)) {
            int outputPriority = request->getParam("outputPriority", true)->value().toInt();
            if (outputPriority >= 0 && outputPriority <= 2) {
                writeModbusRegister(0xE204, static_cast<float>(outputPriority), responseMessage);
                if (responseMessage.startsWith("Modbus operation too soon after last command. Please wait.")) {
                    delay(1000); // Delay for 1 second
                    retry = true; // Flag to retry the operation
                } else if (responseMessage.startsWith("Successfully")) {
                    responseMessage = "Output Priority updated to: " + String(outputPriority);
                } else {
                    responseMessage = "Error updating Output Priority: " + responseMessage;
                }
                if (!retry) addSettingResponse("Output Priority", responseMessage);
            } else {
                responseMessage = "Invalid value for Output Priority.";
                addSettingResponse("Output Priority", responseMessage);
            }
        } else if (request->hasParam("e120", true)) {
            float e120Value = request->getParam("e120", true)->value().toFloat();
            if (e120Value >= 0 && e120Value <= 100) {
                writeModbusRegister(0xE120, e120Value, responseMessage);
                if (responseMessage.startsWith("Modbus operation too soon after last command. Please wait.")) {
                    delay(1000); // Delay for 1 second
                    retry = true; // Flag to retry the operation
                } else if (responseMessage.startsWith("Successfully")) {
                    responseMessage = "E120 PV Charge Current updated to: " + String(e120Value, 0) + "A";
                } else {
                    responseMessage = "Error updating E120 PV Charge Current: " + responseMessage;
                }
                if (!retry) addSettingResponse("E120", responseMessage);
            } else {
                responseMessage = "Invalid value for E120 PV Charge Current. Must be between 0 and 100.";
                addSettingResponse("E120", responseMessage);
            }
        } else if (request->hasParam("e20a", true)) {
            float e20aValue = request->getParam("e20a", true)->value().toFloat();
            if (e20aValue >= 0 && e20aValue <= 100) {
                writeModbusRegister(0xE20A, e20aValue, responseMessage);
                if (responseMessage.startsWith("Modbus operation too soon after last command. Please wait.")) {
                    delay(1000); // Delay for 1 second
                    retry = true; // Flag to retry the operation
                } else if (responseMessage.startsWith("Successfully")) {
                    responseMessage = "E20A Total Max Charge Limit updated to: " + String(e20aValue, 0) + "A";
                } else {
                    responseMessage = "Error updating E20A Total Max Charge Limit: " + responseMessage;
                }
                if (!retry) addSettingResponse("E20A", responseMessage);
            } else {
                responseMessage = "Invalid value for E20A Total Max Charge Limit. Must be between 0 and 100.";
                addSettingResponse("E20A", responseMessage);
            }
        } else {
            responseMessage = "No valid parameters for update.";
            addSettingResponse("Unknown", responseMessage);
        }
    } while (retry);

    request->send(200, "text/plain", responseMessage);
});

  // Add a new endpoint to get setting responses
  server.on("/get_setting_responses", HTTP_GET, [](AsyncWebServerRequest *request){
      String json = "[";
      for (size_t i = 0; i < settingResponses.size(); ++i) {
          json += "{\"setting\":\"" + settingResponses[i].setting + "\",\"response\":\"" + settingResponses[i].response + "\"}";
          if (i < settingResponses.size() - 1) json += ",";
      }
      json += "]";
      request->send(200, "application/json", json);
  });

  server.on("/current_settings", HTTP_GET, [](AsyncWebServerRequest *request){
      String json = "{";
      int gridChargeCurrentIndex = findIndexByAddress(0xE205);
      int outputPriorityIndex = findIndexByAddress(0xE204);
      int pvChargeCurrentIndex = findIndexByAddress(0xE120);
      int totalMaxChargeLimitIndex = findIndexByAddress(0xE20A);
      
      if (gridChargeCurrentIndex != -1 && outputPriorityIndex != -1 && pvChargeCurrentIndex != -1 && totalMaxChargeLimitIndex != -1) {
          json += "\"gridChargeCurrentLimit\":" + String(modbusData[gridChargeCurrentIndex].value, 1) + ",";
          json += "\"outputPriority\":" + String(static_cast<int>(modbusData[outputPriorityIndex].value)) + ",";
          json += "\"pvChargeCurrent\":" + String(modbusData[pvChargeCurrentIndex].value, 0) + ",";
          json += "\"totalMaxChargeLimit\":" + String(modbusData[totalMaxChargeLimitIndex].value, 0);
      } else {
          json += "\"error\":\"One or more settings not found\"";
      }
      json += "}";
      request->send(200, "application/json", json);
  });

  server.on("/read_modbus_register", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("address")) {
      String addressParam = request->getParam("address")->value();
      uint16_t address = (uint16_t)strtoul(addressParam.c_str(), 0, 16); 

      if (!otaProgress) {
        uint32_t decimalValue;
        String hexResult = readModbusRegisterHex(address, decimalValue);
        if (hexResult == "Timeout") {
          request->send(504, "text/plain", "Timeout: No response from Modbus device");
        } else if (hexResult == "0x0") {
          request->send(404, "text/plain", "Error: Register not implemented or invalid");
        } else {
          request->send(200, "text/plain", String(decimalValue)); // Send decimal by default
        }

        // Store the query with both hex and decimal
        ModbusQuery query = {addressParam, hexResult, decimalValue};
        lastQueries.insert(lastQueries.begin(), query);
        if (lastQueries.size() > 10) {
          lastQueries.resize(10);
        }
      } else {
        request->send(400, "text/plain", "OTA update in progress. Please try again later.");
      }
    } else {
      request->send(400, "text/plain", "Error: Address parameter is missing");
    }
  });

  server.on("/last_queries", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!otaProgress) {
      String response = "<ul>";
      for (const auto& query : lastQueries) {
        response += "<li>" + query.address + ": " + query.hexResponse + " (Decimal: " + String(query.decimalResponse) + ")</li>";
      }
      response += "</ul>";
      request->send(200, "text/html", response);
    } else {
      request->send(400, "text/plain", "OTA update in progress. Please try again later.");
    }
  });

  // In your setup function
  server.on("/uptime", HTTP_GET, [](AsyncWebServerRequest *request){
      unsigned long uptimeSeconds = millis() / 1000;
      unsigned long days = uptimeSeconds / 86400;
      unsigned long hours = (uptimeSeconds % 86400) / 3600;
      unsigned long minutes = (uptimeSeconds % 3600) / 60;
      unsigned long seconds = uptimeSeconds % 60;

      String uptimeStr = String(days) + " days, " + String(hours) + " hours, " + 
                        String(minutes) + " minutes, " + String(seconds) + " seconds";
      request->send(200, "text/plain", uptimeStr);
  });

  server.on("/get_build_info", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!otaProgress) {
      String buildInfo = "Built on: " + String(compile_date) + " at " + String(compile_time);
      request->send(200, "text/plain", buildInfo);
    } else {
      request->send(400, "text/plain", "OTA update in progress. Please try again later.");
    }
  });

  server.on("/restart_ota", HTTP_POST, [](AsyncWebServerRequest *request){
    if (request->hasParam("confirm", true)) {
      if (request->getParam("confirm", true)->value() == "doit") {
        request->send(200, "text/plain", "ArduinoOTA will be restarted. Please wait...");
        ArduinoOTA.end();
        ArduinoOTA.begin();
      } else {
        request->send(400, "text/plain", "Invalid confirmation.");
      }
    } else {
      request->send(400, "text/plain", "Confirmation parameter missing.");
    }
  });

  server.on("/restart_mdns", HTTP_POST, [](AsyncWebServerRequest *request){
      if (request->hasParam("confirm", true)) {
          if (request->getParam("confirm", true)->value() == "doit") {
              if (!otaProgress) { 
                  MDNS.end(); 
                  if (!MDNS.begin(currentBLEName.c_str())) {
                      request->send(500, "text/plain", "Failed to restart mDNS");
                  } else {
                      MDNS.addService("ble", "tcp", 0); 
                      request->send(200, "text/plain", "mDNS restarted successfully");
                  }
              } else {
                  request->send(400, "text/plain", "Cannot restart mDNS while OTA is in progress");
              }
          } else {
              request->send(400, "text/plain", "Invalid confirmation.");
          }
      } else {
          request->send(400, "text/plain", "Confirmation parameter missing.");
      }
  });

  server.on("/restart_esp", HTTP_POST, [](AsyncWebServerRequest *request){
    if (request->hasParam("confirm", true)) {
      if (request->getParam("confirm", true)->value() == "doit") {
        request->send(200, "text/plain", "ESP32 will restart now.");
        delay(1000);
        ESP.restart(); 
      } else {
        request->send(400, "text/plain", "Invalid confirmation.");
      }
    } else {
      request->send(400, "text/plain", "Confirmation parameter missing.");
    }
  });

  server.on("/set_modbus_slave", HTTP_POST, [](AsyncWebServerRequest *request){
      if (request->hasParam("slaveAddress", true)) {
          uint8_t newAddress = request->getParam("slaveAddress", true)->value().toInt();
          if (newAddress >= 1 && newAddress <= 254) {
              modbusSlaveAddress = newAddress;
              preferences.begin("modbus", false); // Open for write
              preferences.putUChar("slaveAddress", modbusSlaveAddress);
              preferences.end();
              
              node.begin(modbusSlaveAddress, Serial2); // Reinitialize with new address
              request->send(200, "text/plain", "Modbus slave address updated to: " + String(modbusSlaveAddress));
          } else {
              request->send(400, "text/plain", "Invalid Modbus slave address. Must be between 1 and 254.");
          }
      } else {
          request->send(400, "text/plain", "Modbus slave address parameter missing.");
      }
  });

  server.on("/get_modbus_slave", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(200, "text/plain", String(modbusSlaveAddress));
  });
  
  // server.on("/get_memory_data", HTTP_GET, [](AsyncWebServerRequest *request){
  //     preferences.begin("memStats", true); // Open for read
  //     int numRecords = preferences.getInt("numRecords", 0);
  //     int nextRecord = preferences.getInt("nextRecord", 0);
  //     DynamicJsonDocument doc(4096); // Adjust size based on expected data; this might need to be larger if there's a lot of data
  //     JsonArray data = doc.to<JsonArray>();

  //     unsigned long currentTime = millis() / 1000; // Convert to seconds
      
  //     for (int i = 0; i < numRecords; ++i) {
  //         int recordIndex = (nextRecord + i) % 864; // Use circular buffer logic
  //         String key = "record_" + String(recordIndex);
  //         String record = preferences.getString(key.c_str(), "");
  //         if (!record.isEmpty()) {
  //             int commaIndex = record.indexOf(',');
  //             if (commaIndex > 0) {
  //                 unsigned long timestamp = record.substring(0, commaIndex).toInt();
  //                 size_t freeHeap = record.substring(commaIndex + 1).toInt();

  //                 // Check if this record is within the last 24 hours
  //                 if (timestamp > currentTime - 86400) { // 86400 seconds = 24 hours
  //                     JsonObject obj = data.createNestedObject();
  //                     obj["timestamp"] = timestamp;
  //                     obj["freeHeap"] = freeHeap;
  //                 }
  //             }
  //         }
  //     }

  //     preferences.end();

  //     String response;
  //     serializeJson(doc, response);
  //     request->send(200, "application/json", response);
  // });

  //Get mem usage ondemand
  server.on("/get_memory_usage", HTTP_GET, [](AsyncWebServerRequest *request){
      size_t freeHeap = ESP.getFreeHeap();
      size_t totalHeap = ESP.getHeapSize();
      size_t usedHeap = totalHeap - freeHeap;
      float heapPercentage = (usedHeap * 100.0) / totalHeap;

      String response = "Free Heap: " + String(freeHeap) + " bytes\n";
      response += "Total Heap: " + String(totalHeap) + " bytes\n";
      response += "Used Heap: " + String(usedHeap) + " bytes\n";
      response += "Heap Usage: " + String(heapPercentage, 2) + "%";

      request->send(200, "text/plain", response);
  });

  server.on("/files", HTTP_GET, [](AsyncWebServerRequest *request){
      String html = "<!DOCTYPE HTML><html lang='en'><head><meta name='viewport' content='width=device-width, initial-scale=1'><meta charset='UTF-8'></head><body>";
      html += "<h1>Files on SPIFFS</h1>";
      html += "<p>Free Storage: " + humanReadableSize((SPIFFS.totalBytes() - SPIFFS.usedBytes())) + " | Used Storage: " + humanReadableSize(SPIFFS.usedBytes()) + " | Total Storage: " + humanReadableSize(SPIFFS.totalBytes()) + "</p>";
      html += listFiles(true);
      html += "</body></html>";
      request->send(200, "text/html", html);
  });

  server.on("/upfile", HTTP_POST, [](AsyncWebServerRequest *request){
      request->send(200, "text/plain", "File Upload Successful");
  }, handleFileUpload);

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
    if (esp_task_wdt_status(NULL) == ESP_ERR_TIMEOUT) {
        Serial.println("Warning: Task Watchdog Timer timeout detected!");
        esp_task_wdt_reset();
    } else {
        esp_task_wdt_reset();
    }
    ArduinoOTA.handle(); // Handle OTA updates

  if (!connectedToAnyNetwork) {
    unsigned long currentMillis = millis();
    if (currentMillis - wifiConnectStartMillis >= (tryingPrimary ? PRIMARY_CONNECT_TIME : FALLBACK_CONNECT_TIME)) {
      // Switch network if the current connection attempt has timed out
      if (tryingPrimary) {
        Serial.println("\nSwitching to fallback WiFi...");
        WiFi.begin(fallbackSSID, fallbackPassword);
        tryingPrimary = false;
      } else {
        Serial.println("\nSwitching back to primary WiFi...");
        WiFi.begin(primarySSID, primaryPassword);
        tryingPrimary = true;
      }
      wifiConnectStartMillis = currentMillis; // Reset timer for new connection attempt
    }

    // Check if connected and update status
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nConnected to WiFi!");
      connectedToAnyNetwork = true;
        // Here, restart mDNS when WiFi connects
        MDNS.end(); 
        if (!MDNS.begin(currentBLEName.c_str())) {    
            Serial.println("Error restarting MDNS responder!");
        } else {
            Serial.println("mDNS responder restarted");
            MDNS.addService("ble", "tcp", 0);
        }
      if (tryingPrimary) {
        Serial.println("Connected to primary network.");
      } else {
        Serial.println("Connected to fallback network.");
      }
      
      // Print IP Address
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
    } else {
      // Print dots while trying to connect
      static unsigned long lastPrint = 0;
      if (currentMillis - lastPrint > 500) {
        Serial.print(".");
        lastPrint = currentMillis;
      }
    }
  } else {
    // Check if we've lost connection
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("\nLost WiFi connection, restarting cycle...");
      connectedToAnyNetwork = false;
      tryingPrimary = true; // Start again with primary network
      wifiConnectStartMillis = millis();
      WiFi.begin(primarySSID, primaryPassword);
    }
  }
    esp_task_wdt_reset();
    
    // Modbus Reading every 30 seconds
    if (millis() - lastModbusRead >= MODBUS_READ_INTERVAL) {
        lastModbusRead = millis();

        if (!otaProgress) {
            readModbusRegisters();
        }
        esp_task_wdt_reset();

        // Logging to Serial for debugging if not in timeout or cooldown
        if (!otaProgress) { // Do not log during OTA to save resources
            bool canLog = true; // Assume we can log unless we find otherwise

            // Check if there's a cooldown or timeout in effect (simplified check)
            for (const auto &data : modbusData) {
                if (isnan(data.value)) { // If the value is NaN, it could indicate a timeout or error
                    canLog = false;
                    break;
                }
            }

            if (canLog) {
                for (const auto &data : modbusData) {
                    if (strcmp(data.unit, "-") == 0) {
                        Serial.printf("%s: %s\n", data.name, data.description);
                    } else {
                        Serial.printf("%s: %.2f %s\n", data.name, data.value, data.unit);
                    }
                }
            }
        }
    }
    
    esp_task_wdt_reset();

    // BLE Handling
    if (millis() - lastBLECheck >= BLE_CHECK_INTERVAL) {
        lastBLECheck = millis();

        if (deviceConnected) {
            // Here you can add functionality for when a BLE device is connected
        }

        // Disconnect handling
        if (!deviceConnected && oldDeviceConnected) {
            delay(500); // give the bluetooth stack the chance to get things ready
            pServer->startAdvertising(); // restart advertising
            Serial.println("Start advertising");
            oldDeviceConnected = deviceConnected;
        }
        if (deviceConnected && !oldDeviceConnected) {
            // do stuff here on connecting
            oldDeviceConnected = deviceConnected;
        }
    }

    esp_task_wdt_reset();
    // Save memory data every 5 minutes
    static unsigned long lastSave = 0;
    unsigned long currentMillis = millis();
    if (currentMillis - lastSave >= 300000 || currentMillis < lastSave) { // Check for millis() overflow
        lastSave = currentMillis;
        saveMemoryData();
        Serial.println("Memory data saved");
    }
    
    esp_task_wdt_reset();
    
    // //NTP
    // time_t now;
    // time(&now);
    
    // if (difftime(now, lastSyncTime) >= SECONDS_IN_DAY) {
    //   syncTime();
    //   lastSyncTime = now;
    // }
    delay(100); // Small delay to prevent watchdog timer issues
}
