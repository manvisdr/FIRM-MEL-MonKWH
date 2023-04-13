#include <Arduino.h>
#include <SPI.h>
#include <lorawan.h>
#include <Preferences.h>
#include <SettingsManager.h>
#include <SPIFFS.h>
#include <SpiffsFilePrint.h>
#include <TimeLib.h>
#include <TimeAlarms.h>
#include "EDMICmdLine.h"

#define FIRMWARE_VERSION "1.0"
#define TYPE_KWH 3

#define LOGGING_INTERVAL 5000
#define RXD2 16         // PIN RX2
#define TXD2 17         // PIN TX2
#define PIN_LED_PROC 27 // PIN RX2
#define PIN_LED_WARN 15 // PIN TX2
#define PIN_LED_LINK_KWH PIN_LED_PROC
#define PIN_LED_ACTIVE_KWH PIN_LED_WARN
const sRFM_pins RFM_pins = {
    .CS = 5,
    .RST = 26,
    .DIO0 = 25,
    .DIO1 = 32,
}; // DEF PIN LORA

struct settingStruct
{
  long devId;
  char firmVersion[10];
  long timeRecord;
  char loraDevAddr[10];
  char loraNwkSKey[35];
  char loraAppSKey[35];
};
settingStruct settingVar;

struct dataKWH
{
  char serialnumber[20];
  uint32_t timeUnix;
  float voltR;
  float voltS;
  float voltT;
  float currentR;
  float currentS;
  float currentT;
  float wattR;
  float wattS;
  float wattT;
  float pf;
  float freq;
  float kvar;
  float kwh_bp;
  float kwh_lpb;
  float kwh_total;
};
dataKWH currentDataKWH;

SettingsManager settings;
SpiffsFilePrint filePrint("/logger.csv", 1, 500, &Serial);
EdmiCMDReader edmiread(Serial2, RXD2, TXD2);

Preferences preferences;
char settingFile[] = "/config.json";
unsigned long lastLog = 0;
char bufferLog[300];
unsigned int fcnt;

bool kwhReadReady = false;
bool kwhSend = false;
bool kwhGetSerialNum = false;

bool SettingsInit()
{
  settings.readSettings(settingFile);

  settingVar.devId = settings.getLong("device.id");

  strcpy(settingVar.firmVersion, settings.getChar("device.firmVer"));
  if (strcmp(settingVar.firmVersion, FIRMWARE_VERSION) == 0)
  {
    strcpy(settingVar.firmVersion, FIRMWARE_VERSION);
    settings.setChar("device.firmVer", settingVar.firmVersion);
    settings.writeSettings(settingFile);
    strcpy(settingVar.firmVersion, settings.getChar("device.firmVer"));
  }

  settingVar.timeRecord = settings.getLong("device.timeRecord");

  strcpy(settingVar.loraDevAddr, settings.getChar("lora.devid"));
  strcpy(settingVar.loraNwkSKey, settings.getChar("lora.nwks"));
  strcpy(settingVar.loraAppSKey, settings.getChar("lora.apps"));

  Serial.printf("--------------------------------------------------------\n");
  Serial.printf("| %-16s | %-33s |\n", "CONFIG", "VALUE");
  Serial.printf("--------------------------------------------------------\n");
  Serial.printf("| %-16s | %-33d |\n", "ID DEVICE", settingVar.devId);
  Serial.printf("| %-16s | %-33s |\n", "FIRMWARE VERSION", settingVar.firmVersion);
  Serial.printf("| %-16s | %-33d |\n", "TIMERECORD", settingVar.timeRecord);
  Serial.printf("| %-16s | %-33s |\n", "LORA DEVADDR", settingVar.loraDevAddr);
  Serial.printf("| %-16s | %-33s |\n", "LORA NWKSKEY", settingVar.loraNwkSKey);
  Serial.printf("| %-16s | %-33s |\n", "LORA APPSKEY", settingVar.loraAppSKey);
  Serial.printf("--------------------------------------------------------\n");
  return true;
}

bool loraConf()
{
  lora.setDeviceClass(CLASS_C);
  lora.setDataRate(SF7BW125);
  lora.setChannel(MULTI);
  lora.setNwkSKey(settingVar.loraNwkSKey);
  lora.setAppSKey(settingVar.loraAppSKey);
  lora.setDevAddr(settingVar.loraDevAddr);
  lora.setFrameCounter(fcnt);
  return true;
}

void blinkLINKOn()
{
  digitalWrite(PIN_LED_LINK_KWH, HIGH);
}
void blinkLINKOff()
{
  digitalWrite(PIN_LED_LINK_KWH, LOW);
}
void blinkACTIVEOn()
{
  digitalWrite(PIN_LED_ACTIVE_KWH, HIGH);
}
void blinkACTIVEOff()
{
  digitalWrite(PIN_LED_ACTIVE_KWH, LOW);
}

void backgroundTask(void)
{
  if (edmiread.keepAlive())
    blinkLINKOn();
  else
    blinkLINKOff();
}

void BackgroundDelay()
{
  kwhReadReady = true;
  edmiread.acknowledge();
}

void printFileToConsole(String filename)
{
  Serial.println("--------------------");
  File file = SPIFFS.open(filename, FILE_READ);
  Serial.printf("filename: %s - size: %d\n", filename, file.size());
  while (file.available())
  {
    Serial.write(file.read());
  }
  Serial.println("--------------------");
  file.close();
}

void printDigits(int digits)
{
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void digitalClockDisplay()
{

  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.println();
}

void KWHInit()
{
  strcpy(currentDataKWH.serialnumber, edmiread.serialNumber().c_str());
  edmiread.read_time();
  setTime(
      edmiread.timeEdmi.hour,
      edmiread.timeEdmi.minute,
      edmiread.timeEdmi.seconds,
      edmiread.timeEdmi.day,
      edmiread.timeEdmi.month,
      edmiread.timeEdmi.year);
  digitalClockDisplay();
}

void KWHUpdateData(dataKWH &datakwh)
{
  datakwh.voltR = edmiread.voltR();
  datakwh.voltS = edmiread.voltS();
  datakwh.voltT = edmiread.voltT();
  datakwh.currentR = edmiread.currentR();
  datakwh.currentS = edmiread.currentS();
  datakwh.currentT = edmiread.currentT();
  datakwh.wattR = edmiread.wattR();
  datakwh.wattS = edmiread.wattS();
  datakwh.wattT = edmiread.wattT();
  datakwh.pf = edmiread.pf();
  datakwh.freq = edmiread.frequency();
  datakwh.kvar = edmiread.kVarh();
  datakwh.kwh_bp = edmiread.kwhWBP();
  datakwh.kwh_lpb = edmiread.kwhLWBP();
  datakwh.kwh_total = edmiread.kwhTotal();
}

void KWHDataSend(char *buffer)
{
  sprintf(buffer, "~*%d*%s*%d*%s*%s*%s*%s*%s*%s*%s*%s*%s*%s*%s*%s*%s*%s*%s#",
          TYPE_KWH,
          settingVar.devId,
          currentDataKWH.serialnumber,
          String(currentDataKWH.voltR * 1000, 4).c_str(),
          String(currentDataKWH.voltS * 1000, 4).c_str(),
          String(currentDataKWH.voltT * 1000, 4).c_str(),
          String(currentDataKWH.currentR, 4).c_str(),
          String(currentDataKWH.currentS, 4).c_str(),
          String(currentDataKWH.currentT, 4).c_str(),
          String(currentDataKWH.wattR / 10.0, 4).c_str(),
          String(currentDataKWH.wattS / 10.0, 4).c_str(),
          String(currentDataKWH.wattT / 10.0, 4).c_str(),
          String(currentDataKWH.pf, 4).c_str(),
          String(currentDataKWH.freq, 4).c_str(),
          String(currentDataKWH.kvar / 1000.0, 2).c_str(),
          String(currentDataKWH.kwh_bp / 1000.0, 2).c_str(),
          String(currentDataKWH.kwh_lpb / 1000.0, 2).c_str(),
          String(currentDataKWH.kwh_total / 1000.0, 2).c_str());

  Serial.println(buffer);
  // free(bufferdata);
}

void formatLogging(/* char *buffer */)
{
  char timeFormat[20];
  char buffer[255];

  sprintf(timeFormat, "%d/%d/%d %d:%d%d\n", year(), month(), day(), hour(), minute(), second());
  sprintf(buffer, "%d,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s",
          now(),
          currentDataKWH.serialnumber,
          String(currentDataKWH.voltR, 2).c_str(),
          String(currentDataKWH.voltS, 2).c_str(),
          String(currentDataKWH.voltT, 2).c_str(),
          String(currentDataKWH.currentR, 2).c_str(),
          String(currentDataKWH.currentS, 2).c_str(),
          String(currentDataKWH.currentT, 2).c_str(),
          String(currentDataKWH.wattR / 10.0, 2).c_str(),
          String(currentDataKWH.wattS / 10.0, 2).c_str(),
          String(currentDataKWH.wattT / 10.0, 2).c_str(),
          String(currentDataKWH.pf, 2).c_str(),
          String(currentDataKWH.freq, 2).c_str(),
          String(currentDataKWH.kvar / 1000.0, 2).c_str(),
          String(currentDataKWH.kwh_bp / 1000.0, 2).c_str(),
          String(currentDataKWH.kwh_lpb / 1000.0, 2).c_str(),
          String(currentDataKWH.kwh_total / 1000.0, 2).c_str());
  // Serial.println(buffer);
  Serial.println(timeFormat);
}

void KWHLogging()
{
}

void setup()
{
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  SPIFFS.begin();
  SettingsInit();
  pinMode(PIN_LED_PROC, OUTPUT);
  pinMode(PIN_LED_WARN, OUTPUT);
  preferences.begin("settings", false);
  fcnt = preferences.getUInt("lora_fcnt", 0);
  if (!lora.init())
  {
    Serial.println("RFM95 not detected");
  }
  loraConf();
  KWHInit();
  Alarm.timerRepeat(15, BackgroundDelay); // timer for every 15 seconds
  Alarm.timerRepeat(20, formatLogging);

  // filePrint.print("Millis since start: ");
  // filePrint.println(millis());
  // filePrint.open();
  // filePrint.close();
}

void loop()
{
  if (!kwhGetSerialNum)
  {
    Serial.println("get data serial in loop");
    strcpy(currentDataKWH.serialnumber, edmiread.serialNumber().c_str());
    kwhGetSerialNum = true;
  }
  backgroundTask();
  edmiread.read_looping();

  EdmiCMDReader::Status status = edmiread.status(); // Serial.printf("%s\n", kwhReadReady ? "true" : "false");
  if (status == EdmiCMDReader::Status::Ready and kwhReadReady)
  {
    blinkACTIVEOn();
    edmiread.step_start();
  }

  else if (status == EdmiCMDReader::Status::Finish)
  {
    char bufferdata[255];
    KWHUpdateData(currentDataKWH);
    KWHDataSend(bufferdata);
    // lora.sendUplink(myStr, strlen(myStr), 1, 1);
    // lora.update();
    preferences.putUInt("lora_fcnt", lora.getFrameCounter());
    fcnt = preferences.getUInt("lora_fcnt", 0);
    Serial.printf("Frame Counter: %d\n", fcnt);
    kwhReadReady = false;
    formatLogging
    blinkACTIVEOff();
  }
  else if (status != EdmiCMDReader::Status::Busy)
  {
  }
  Alarm.delay(0);
}