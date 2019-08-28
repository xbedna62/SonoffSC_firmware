/*

Sonoff SC
Copyright (C) 2017 by Xose Pérez <xose dot perez at gmail dot com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <Arduino.h>
#include <EEPROM.h>
#include "config/all.h"

// -----------------------------------------------------------------------------
// METHODS
// -----------------------------------------------------------------------------

bool ledStatus(bool status) {
    digitalWrite(LED_PIN, status ? LOW : HIGH);
    return status;
}

bool ledStatus() {
    return (digitalRead(LED_PIN) == LOW);
}

bool ledToggle() {
    return ledStatus(!ledStatus());
}

void ledBlink(unsigned long delayOff, unsigned long delayOn) {
    static unsigned long next = millis();
    if (next < millis()) {
        next += (ledToggle() ? delayOn : delayOff);
    }
}

void showStatus() {
    if (wifiConnected()) {
        if (WiFi.getMode() == WIFI_AP) {
            ledBlink(2000, 2000);
        } else {
            ledBlink(5000, 500);
        }
    } else {
        ledBlink(500, 500);
    }
}

void hardwareSetup() {

    EEPROM.begin(EEPROM_SIZE);

    #if DEBUG_SERIAL_SUPPORT
        DEBUG_PORT.begin(SERIAL_BAUDRATE);
        #if DEBUG_ESP_WIFI
            DEBUG_PORT.setDebugOutput(true);
        #endif
    #elif defined(SERIAL_BAUDRATE)
        Serial.begin(SERIAL_BAUDRATE);
    #endif

    #if SPIFFS_SUPPORT
        SPIFFS.begin();
    #endif

    pinMode(LED_PIN, OUTPUT);

}

void hardwareLoop() {

    showStatus();

    // System check
    static bool checked = false;
    if (!checked && (millis() > CRASH_SAFE_TIME)) {
        // Check system as stable
        systemCheck(true);
        checked = true;
    }

    // Heartbeat
    static unsigned long last_heartbeat = 0;
    if (mqttConnected()) {
        if ((millis() - last_heartbeat > HEARTBEAT_INTERVAL) || (last_heartbeat == 0)) {
            last_heartbeat = millis();
            heartbeat();
        }
    }

}

// -----------------------------------------------------------------------------
// BOOTING
// -----------------------------------------------------------------------------

unsigned int sectors(size_t size) {
    return (int) (size + SPI_FLASH_SEC_SIZE - 1) / SPI_FLASH_SEC_SIZE;
}

void welcome() {

    DEBUG_MSG_P(PSTR("\n\n"));
    DEBUG_MSG_P(PSTR("[INIT] %s %s\n"), (char *) APP_NAME, (char *) APP_VERSION);
    DEBUG_MSG_P(PSTR("[INIT] %s\n"), (char *) APP_AUTHOR);
    DEBUG_MSG_P(PSTR("[INIT] %s\n\n"), (char *) APP_WEBSITE);
    DEBUG_MSG_P(PSTR("[INIT] CPU chip ID: 0x%06X\n"), ESP.getChipId());
    DEBUG_MSG_P(PSTR("[INIT] CPU frequency: %d MHz\n"), ESP.getCpuFreqMHz());
    DEBUG_MSG_P(PSTR("[INIT] SDK version: %s\n"), ESP.getSdkVersion());
    DEBUG_MSG_P(PSTR("[INIT] Core version: %s\n"), ESP.getCoreVersion().c_str());
    DEBUG_MSG_P(PSTR("\n"));

    // -------------------------------------------------------------------------

    FlashMode_t mode = ESP.getFlashChipMode();
    DEBUG_MSG_P(PSTR("[INIT] Flash chip ID: 0x%06X\n"), ESP.getFlashChipId());
    DEBUG_MSG_P(PSTR("[INIT] Flash speed: %u Hz\n"), ESP.getFlashChipSpeed());
    DEBUG_MSG_P(PSTR("[INIT] Flash mode: %s\n"), mode == FM_QIO ? "QIO" : mode == FM_QOUT ? "QOUT" : mode == FM_DIO ? "DIO" : mode == FM_DOUT ? "DOUT" : "UNKNOWN");
    DEBUG_MSG_P(PSTR("\n"));
    DEBUG_MSG_P(PSTR("[INIT] Flash sector size: %8u bytes\n"), SPI_FLASH_SEC_SIZE);
    DEBUG_MSG_P(PSTR("[INIT] Flash size (CHIP): %8u bytes\n"), ESP.getFlashChipRealSize());
    DEBUG_MSG_P(PSTR("[INIT] Flash size (SDK):  %8u bytes / %4d sectors\n"), ESP.getFlashChipSize(), sectors(ESP.getFlashChipSize()));
    DEBUG_MSG_P(PSTR("[INIT] Firmware size:     %8u bytes / %4d sectors\n"), ESP.getSketchSize(), sectors(ESP.getSketchSize()));
    DEBUG_MSG_P(PSTR("[INIT] OTA size:          %8u bytes / %4d sectors\n"), ESP.getFreeSketchSpace(), sectors(ESP.getFreeSketchSpace()));
    #if SPIFFS_SUPPORT
        FSInfo fs_info;
        bool fs = SPIFFS.info(fs_info);
        if (fs) {
            DEBUG_MSG_P(PSTR("[INIT] SPIFFS size:       %8u bytes / %4d sectors\n"), fs_info.totalBytes, sectors(fs_info.totalBytes));
        }
    #else
        DEBUG_MSG_P(PSTR("[INIT] SPIFFS size:       %8u bytes / %4d sectors\n"), 0, 0);
    #endif
    DEBUG_MSG_P(PSTR("[INIT] EEPROM size:       %8u bytes / %4d sectors\n"), settingsMaxSize(), sectors(settingsMaxSize()));
    DEBUG_MSG_P(PSTR("[INIT] Empty space:       %8u bytes /    4 sectors\n"), 4 * SPI_FLASH_SEC_SIZE);
    DEBUG_MSG_P(PSTR("\n"));

    // -------------------------------------------------------------------------

    #if SPIFFS_SUPPORT
        if (fs) {
            DEBUG_MSG_P(PSTR("[INIT] SPIFFS total size: %8u bytes\n"), fs_info.totalBytes);
            DEBUG_MSG_P(PSTR("[INIT]        used size:  %8u bytes\n"), fs_info.usedBytes);
            DEBUG_MSG_P(PSTR("[INIT]        block size: %8u bytes\n"), fs_info.blockSize);
            DEBUG_MSG_P(PSTR("[INIT]        page size:  %8u bytes\n"), fs_info.pageSize);
            DEBUG_MSG_P(PSTR("[INIT]        max files:  %8u\n"), fs_info.maxOpenFiles);
            DEBUG_MSG_P(PSTR("[INIT]        max length: %8u\n"), fs_info.maxPathLength);
        } else {
            DEBUG_MSG_P(PSTR("[INIT] No SPIFFS partition\n"));
        }
        DEBUG_MSG_P(PSTR("\n"));
    #endif

    // -------------------------------------------------------------------------

    DEBUG_MSG_P(PSTR("[INIT] MANUFACTURER: %s\n"), MANUFACTURER);
    DEBUG_MSG_P(PSTR("[INIT] DEVICE: %s\n"), DEVICE_NAME);
    DEBUG_MSG_P(PSTR("[INIT] SUPPORT:"));

    #if ALEXA_SUPPORT
        DEBUG_MSG_P(PSTR(" ALEXA"));
    #endif
    #if DEBUG_SERIAL_SUPPORT
        DEBUG_MSG_P(PSTR(" DEBUG_SERIAL"));
    #endif
    #if DEBUG_UDP_SUPPORT
        DEBUG_MSG_P(PSTR(" DEBUG_UDP"));
    #endif
    #if DOMOTICZ_SUPPORT
        DEBUG_MSG_P(PSTR(" DOMOTICZ"));
    #endif
    #if MDNS_SUPPORT
        DEBUG_MSG_P(PSTR(" MDNS"));
    #endif
    #if NOFUSS_SUPPORT
        DEBUG_MSG_P(PSTR(" NOFUSS"));
    #endif
    #if NTP_SUPPORT
        DEBUG_MSG_P(PSTR(" NTP"));
    #endif
    #if SPIFFS_SUPPORT
        DEBUG_MSG_P(PSTR(" SPIFFS"));
    #endif
    #if TERMINAL_SUPPORT
        DEBUG_MSG_P(PSTR(" TERMINAL"));
    #endif

    DEBUG_MSG_P(PSTR("\n\n"));

    // -------------------------------------------------------------------------

    unsigned char custom_reset = customReset();
    if (custom_reset > 0) {
        char buffer[32];
        strcpy_P(buffer, custom_reset_string[custom_reset-1]);
        DEBUG_MSG_P(PSTR("[INIT] Last reset reason: %s\n"), buffer);
    } else {
        DEBUG_MSG_P(PSTR("[INIT] Last reset reason: %s\n"), (char *) ESP.getResetReason().c_str());
    }

    DEBUG_MSG_P(PSTR("[INIT] Free heap: %u bytes\n"), ESP.getFreeHeap());
    #if ADC_VCC_ENABLED
        DEBUG_MSG_P(PSTR("[INIT] Power: %d mV\n"), ESP.getVcc());
    #endif
    DEBUG_MSG_P(PSTR("\n"));

}

void setup() {

    // Wait for the ATMEGA to be ready
    delay(2000);

    // Init EEPROM, Serial and SPIFFS
    hardwareSetup();

    // Question system stability
    systemCheck(false);

    // Show welcome message and system configuration
    welcome();

    // Init persistance and terminal features
    settingsSetup();
    if (getSetting("hostname").length() == 0) {
        setSetting("hostname", getIdentifier());
    }

    wifiSetup();
    otaSetup();
    #if TELNET_SUPPORT
        telnetSetup();
    #endif

    // Do not run the next services if system is flagged stable
    if (!systemCheck()) return;

    buttonSetup();
    mqttSetup();
    webSetup();
    commsSetup();
    lightsSetup();

    #if NTP_SUPPORT
        ntpSetup();
    #endif
    #if ALEXA_SUPPORT
	    alexaSetup();
	#endif
    #if NOFUSS_SUPPORT
        nofussSetup();
    #endif
    #if DOMOTICZ_SUPPORT
        domoticzSetup();
    #endif

    saveSettings();

    sendNotification(true, NOTIFICATION_TIME);

}

void loop() {

    hardwareLoop();
    settingsLoop();
    wifiLoop();
    otaLoop();

    // Do not run the next services if system is flagged stable
    if (!systemCheck()) return;

    buttonLoop();
    mqttLoop();
    commsLoop();

    #if NTP_SUPPORT
        ntpLoop();
    #endif
    #if ALEXA_SUPPORT
        alexaLoop();
    #endif
    #if NOFUSS_SUPPORT
        nofussLoop();
    #endif

}
