/**
 * @file      GPS.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2024  ShenZhen XinYuan Electronic Technology Co., Ltd
 * @date      2024-07-07
 *
 */
#include "GPS.h"
#include <cstring>

struct uBloxGnssModelInfo { // Structure to hold the module info (uses 341 bytes of RAM)
    char softVersion[30];
    char hardwareVersion[10];
    uint8_t extensionNo = 0;
    char extension[10][30];
} ;


GPS::GPS() : model("Unknown")
{
}

GPS::~GPS()
{
}

bool GPS::init(Stream *stream)
{
    struct uBloxGnssModelInfo info ;

    _stream = stream;
    assert(_stream);

    uint8_t buffer[256];

    int retry = 3;

    while (retry--) {
        // bool legacy_ubx_message = true;
        //  Get UBlox GPS module version
        uint8_t cfg_get_hw[] =  {0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34};
        _stream->write(cfg_get_hw, sizeof(cfg_get_hw));

        uint16_t len = getAck(buffer, 256, 0x0A, 0x04);
        if (len) {
            memset((void*)&info, 0, sizeof(info));
            uint16_t position = 0;
            for (int i = 0; i < 30; i++) {
                info.softVersion[i] = buffer[position];
                position++;
            }
            for (int i = 0; i < 10; i++) {
                info.hardwareVersion[i] = buffer[position];
                position++;
            }
            while (len >= position + 30) {
                for (int i = 0; i < 30; i++) {
                    info.extension[info.extensionNo][i] = buffer[position];
                    position++;
                }
                info.extensionNo++;
                if (info.extensionNo > 9)
                    break;
            }

            log_i("Module Info : ");
            log_i("Soft version: %s", info.softVersion);
            log_i("Hard version: %s", info.hardwareVersion);
            log_i("Extensions: %d", info.extensionNo);
            for (int i = 0; i < info.extensionNo; i++) {
                log_i("%s", info.extension[i]);
            }
            if (info.extensionNo > 2) {
                log_i("Model:%s", info.extension[2]);
            }

            for (int i = 0; i < info.extensionNo; ++i) {
                if (!strncmp(info.extension[i], "OD=", 3)) {
                    strcpy((char *)buffer, &(info.extension[i][3]));
                    log_i("GPS Model: %s", (char *)buffer);
                    model = (char *)buffer;
                }
            }
            return true;
        }
        delay(200);
    }

    log_e("Warning: Failed to find GPS.\n");
    return false;
}


int GPS::getAck(uint8_t *buffer, uint16_t size, uint8_t requestedClass, uint8_t requestedID)
{
    uint16_t    ubxFrameCounter = 0;
    uint32_t    startTime = millis();
    uint16_t    needRead =  0;
    assert(_stream);
    while (millis() - startTime < 800) {
        while (_stream->available()) {
            int c = _stream->read();
            switch (ubxFrameCounter) {
            case 0:
                if (c == 0xB5) {
                    ubxFrameCounter++;
                }
                break;
            case 1:
                if (c == 0x62) {
                    ubxFrameCounter++;
                } else {
                    ubxFrameCounter = 0;
                }
                break;
            case 2:
                if (c == requestedClass) {
                    ubxFrameCounter++;
                } else {
                    ubxFrameCounter = 0;
                }
                break;
            case 3:
                if (c == requestedID) {
                    ubxFrameCounter++;
                } else {
                    ubxFrameCounter = 0;
                }
                break;
            case 4:
                needRead = c;
                ubxFrameCounter++;
                break;
            case 5:
                needRead |=  (c << 8);
                ubxFrameCounter++;
                break;
            case 6:
                if (needRead >= size) {
                    ubxFrameCounter = 0;
                    break;
                }
                if (_stream->readBytes(buffer, needRead) != needRead) {
                    ubxFrameCounter = 0;
                } else {
                    return needRead;
                }
                break;

            default:
                break;
            }
        }
    }
    return 0;
}


bool GPS::factory()
{
    assert(_stream);

    uint8_t buffer[256];
    // Revert module Clear, save and load configurations
    // B5 62 06 09 0D 00 FF FF 00 00 00 00 00 00 FF FF 00 00 17 2F AE
    // Restore configuration to factory defaults using UBX-CFG-CFG as described in the
    // u-blox MIA-M10Q datasheet (clearMask 0xFFFF, loadMask 0xFFFF, deviceMask 0x17).
    uint8_t cfg_cfg_reset[] = {
        0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00,
        0xFF, 0xFF, 0x00, 0x00,             // clearMask
        0x00, 0x00, 0x00, 0x00,             // saveMask
        0xFF, 0xFF, 0x00, 0x00,             // loadMask
        0x17,                               // deviceMask (BBR, Flash, RAM)
        0x2F, 0xAE                          // CK_A, CK_B
    };
    _stream->write(cfg_cfg_reset, sizeof(cfg_cfg_reset));
    if (!getAck(buffer, sizeof(buffer), 0x05, 0x01) || buffer[0] != 0x06 || buffer[1] != 0x09) {
        return false;
    }
    delay(50);

    // UBX-CFG-RATE, Size 8, 'Navigation/measurement rate settings'
    // Configure the measurement rate to 1000 ms (1 Hz) with a navigation rate of 1 and
    // GPS time reference, which matches the module defaults documented in the datasheet.
    uint8_t cfg_rate[] = {
        0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
        0xE8, 0x03,                         // measRate = 1000 ms
        0x01, 0x00,                         // navRate = 1
        0x01, 0x00,                         // timeRef = GPS time
        0x01, 0x39                          // CK_A, CK_B
    };
    _stream->write(cfg_rate, sizeof(cfg_rate));
    if (!getAck(buffer, sizeof(buffer), 0x05, 0x01) || buffer[0] != 0x06 || buffer[1] != 0x08) {
        return false;
    }
    log_d("GPS reset successes!");
    return true;
}
