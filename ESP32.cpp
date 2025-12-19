#include <Arduino.h>
#line 1 "C:\\Users\\Lenovo\\Documents\\Arduino\\BLE\\BLE.ino"
#include <BLEMidi.h>
#include <HardwareSerial.h>

/*
 * 与 Raspberry Pi 通信的串口 (Serial2 : GPIO16 RX, GPIO17 TX)
 */
HardwareSerial RPiSerial(2);

void setup() {
  Serial.begin(115200);
  RPiSerial.begin(115200, SERIAL_8N1, 16, 17);
  Serial.println("ESP32 BLE-MIDI 桥接程序（解析并转发）已启动");

  BLEMidiServer.begin("KalimbaBLE");
  BLEMidiServer.setOnConnectCallback([](){ Serial.println("BLE-MIDI: Client connected"); });
  BLEMidiServer.setOnDisconnectCallback([](){ Serial.println("BLE-MIDI: Client disconnected"); });
}

uint8_t runningStatus = 0;
uint8_t dataBuf[2];
uint8_t dataCount = 0;
bool waitingForSysEx = false;

void handleMessage(uint8_t status, uint8_t *data, uint8_t len) {
  uint8_t type = status & 0xF0;
  uint8_t channel = status & 0x0F; // 0..15

  switch (type) {
    case 0x80: // Note Off
      if (len >= 2) BLEMidiServer.noteOff(channel, data[0], data[1]);
      break;

    case 0x90: // Note On
      if (len >= 2) {
        if (data[1] == 0)
          BLEMidiServer.noteOff(channel, data[0], data[1]);
        else
          BLEMidiServer.noteOn(channel, data[0], data[1]);
      }
      break;

    case 0xA0: // Polyphonic Aftertouch
      // 【兼容修复】库 v0.3.2 没有 note-specific aftertouch API (3 args).
      // 将 poly aftertouch 映射为 channel aftertouch：只发送 pressure（data[1]）。
      // 如果你需要保留 note 信息，需要后续查找或修改库以支持 note-specific aftertouch。
      if (len >= 2) BLEMidiServer.afterTouch(channel, data[1]);
      break;

    case 0xB0: // Control Change
      if (len >= 2) BLEMidiServer.controlChange(channel, data[0], data[1]);
      break;

    case 0xC0: // Program Change
      if (len >= 1) BLEMidiServer.programChange(channel, data[0]);
      break;

    case 0xD0: // Channel Aftertouch (single-byte)
      if (len >= 1) BLEMidiServer.afterTouch(channel, data[0]);
      break;

    case 0xE0: { // Pitch Bend
      if (len >= 2) {
        uint16_t value = ((uint16_t)data[1] << 7) | (data[0] & 0x7F);
        BLEMidiServer.pitchBend(channel, value);
      }
      break;
    }

    default:
      // 忽略系统消息/其他
      break;
  }
}

void loop() {
  while (RPiSerial.available() > 0) {
    uint8_t b = (uint8_t)RPiSerial.read();

    if (b >= 0xF8) { // realtime single-byte
      continue;
    }

    if (b & 0x80) { // status byte
      if (b == 0xF0) { waitingForSysEx = true; continue; }
      else if (b == 0xF7) { waitingForSysEx = false; continue; }
      else if (b >= 0xF0) { runningStatus = 0; dataCount = 0; continue; }
      else { // channel status
        runningStatus = b;
        dataCount = 0;
        continue;
      }
    } else { // data byte
      if (waitingForSysEx) continue;
      if (runningStatus == 0) continue;

      if (dataCount < 2) dataBuf[dataCount++] = b;
      else { dataCount = 1; dataBuf[0] = b; }

      uint8_t t = runningStatus & 0xF0;
      uint8_t needed = (t == 0xC0 || t == 0xD0) ? 1 : 2;
      if (dataCount >= needed) {
        handleMessage(runningStatus, dataBuf, needed);
        dataCount = 0;
      }
    }
  }
}

