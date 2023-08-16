#include <Arduino.h>
// #include <EEPROM.h>
// #include "GXDLMSClient.h"
#include "dlms_funct.h"
#include <TimeLib.h>
#include <TimeAlarms.h>
#include <SPI.h>
#include <lorawan.h>
#include <Preferences.h>
#include <SettingsManager.h>

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
  uint32_t timeUnix;
  char serialnumber[12];
  float volt;
  float current;
  float watt;
  float pf;
  float freq;
  int kvar;
  int kvar2;
  int kwh1;
  int kwh2;
};
dataKWH datameter;

SettingsManager settings;
// SpiffsFilePrint filePrint("/logger.csv", 1, 500, &Serial);

Preferences preferences;
char settingFile[] = "/config.json";
unsigned long lastLog = 0;
char bufferLog[300];
unsigned int fcnt;
char buffData[255];

String customerID;
String kwhID;

gxByteBuffer frameData;
uint32_t runTime = 0;
bool kwhReadReady = false;

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
  strcpy(settingVar.loraNwkSKey, settings.getChar("lora.nwsk"));
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

bool InitLora()
{
  if (!lora.init())
  {
    Serial.println("LoraInit(): Unsuccessfull");
    return false;
  }
  lora.setDeviceClass(CLASS_C);
  lora.setDataRate(SF7BW125);
  lora.setChannel(MULTI);
  lora.setNwkSKey(settingVar.loraNwkSKey);
  lora.setAppSKey(settingVar.loraAppSKey);
  lora.setDevAddr(settingVar.loraDevAddr);
  lora.setFrameCounter(fcnt);
  Serial.println("LoraInit(): Successfull");
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

void blinkWarning(int times)
{
  if (times == 0)
    digitalWrite(PIN_LED_WARN, 0);
  else
  {
    for (int x = 0; x <= times; x++)
    {
      static long prevMill = 0;
      if (((long)millis() - prevMill) >= 500)
      {
        prevMill = millis();
        digitalWrite(PIN_LED_WARN, !digitalRead(PIN_LED_WARN));
      }
    }
  }
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

void blinkProcces(int times)
{
  for (int x = 0; x <= times; x++)
  {
    digitalWrite(PIN_LED_PROC, !digitalRead(PIN_LED_PROC));
    delay(50);
    digitalWrite(PIN_LED_PROC, !digitalRead(PIN_LED_PROC));
    delay(50);
  }
}

byte aNibble(char in)
{
  if (in >= '0' && in <= '9')
    return in - '0';
  else if (in >= 'a' && in <= 'f')
    return in - 'a' + 10;
  else if (in >= 'A' && in <= 'F')
    return in - 'A' + 10;
  return 0;
}

int com_readStaticOnce()
{
  int ret;
  // Initialize connection.
  ret = com_initializeConnection();
  if (ret != DLMS_ERROR_CODE_OK)
  {
    GXTRACE_INT(GET_STR_FROM_EEPROM("com_initializeConnection failed"), ret);
    return ret;
  }
  // GXTRACE_INT(GET_STR_FROM_EEPROM("com_initializeConnection SUCCEEDED"), ret);
  // GXTRACE_INT(GET_STR_FROM_EEPROM("\n"), ret);

  // Read just wanted objects withour serializating the association view.
  char *data = NULL;
  gxByteBuffer snBuff;
  gxClock clock1;
  gxData sn;
  bb_Init(&snBuff);

  // Read clock
  cosem_init(BASE(clock1), DLMS_OBJECT_TYPE_CLOCK, "0.0.1.0.0.255");
  com_read(BASE(clock1), 3);
  com_read(BASE(clock1), 2);
  datameter.timeUnix = time_toUnixTime2(&clock1.time);
  obj_toString(BASE(clock1), &data);
  obj_clear(BASE(clock1));
  free(data);

  // Read serial number
  cosem_init(BASE(sn), DLMS_OBJECT_TYPE_DATA, "0.0.96.1.1.255");
  com_read(BASE(sn), 2);
  obj_toString(BASE(sn), &data);
  var_toString(&sn.value, &snBuff);
  char *pStr = bb_toString(&snBuff);

  String aa;
  const char s[2] = " ";
  char result[100] = "";
  char output[12];
  char buff[50];
  char *token;
  char buf = 0;
  token = strtok(pStr, s);
  // Serial.println(F("----------------------------"));

  while (token != NULL)
  {
    // Serial.print(token);
    strcat(result, token);
    token = strtok(NULL, s);
  }
  // Serial.println();
  char *prtst = result;
  for (byte i = 0; i < 12; i++)
  {
    output[i] = aNibble(*prtst++);
    output[i] <<= 4;
    output[i] |= aNibble(*prtst++);
  }
  strcpy(datameter.serialnumber, output);
  // Serial.println(F("----------------------------"));

  free(pStr);
  obj_clear(BASE(sn));
  free(data);
  GuruxClient.ReleaseObjects();
  return ret;
}

// This function reads ALL objects that meter have. It will loop all object's attributes.
int com_readAllObjects() // const char *invocationCounter)
{
  int ret;
  ret = com_initializeConnection();
  if (ret != DLMS_ERROR_CODE_OK)
  {
    GXTRACE_INT(GET_STR_FROM_EEPROM("com_initializeConnection failed"), ret);
    return ret;
  }
  gxRegister objVolt;
  gxRegister objCurrent;
  gxRegister objWatt;
  gxRegister objpF;
  gxRegister objFreq;
  gxRegister objKvar1;
  gxRegister objKvar2;
  gxRegister objKwh1;
  gxRegister objKwh2;

  char *data = NULL;

  cosem_init(BASE(objVolt), DLMS_OBJECT_TYPE_REGISTER, "1.0.32.7.0.255");
  ret = com_read(BASE(objVolt), 3);
  ret = com_read(BASE(objVolt), 2);
  if (ret != DLMS_ERROR_CODE_OK)
  {
    blinkLINKOff();
    return ret;
  }
  blinkLINKOn();

  datameter.volt = var_toDouble(&objVolt.value) * pow(10, objVolt.scaler);
  Serial.printf("Volt : %.3f\n", datameter.volt);
  obj_clear(BASE(objVolt));

  cosem_init(BASE(objCurrent), DLMS_OBJECT_TYPE_REGISTER, "1.0.31.7.0.255");
  ret = com_read(BASE(objCurrent), 3);
  ret = com_read(BASE(objCurrent), 2);
  if (ret != DLMS_ERROR_CODE_OK)
  {
    blinkLINKOff();
    return ret;
  }
  blinkLINKOn();

  datameter.current = var_toDouble(&objCurrent.value) * pow(10, objCurrent.scaler);
  Serial.printf("Current : %.3f\n", datameter.current);
  obj_clear(BASE(objCurrent));

  cosem_init(BASE(objWatt), DLMS_OBJECT_TYPE_REGISTER, "1.0.21.7.0.255");
  ret = com_read(BASE(objWatt), 3);
  ret = com_read(BASE(objWatt), 2);
  if (ret != DLMS_ERROR_CODE_OK)
  {
    blinkLINKOff();
    return ret;
  }
  blinkLINKOn();

  datameter.watt = var_toDouble(&objWatt.value) * pow(10, objWatt.scaler);
  Serial.printf("Watt : %.3f\n", datameter.watt);
  obj_clear(BASE(objWatt));

  cosem_init(BASE(objpF), DLMS_OBJECT_TYPE_REGISTER, "1.0.33.7.0.255");
  ret = com_read(BASE(objpF), 3);
  ret = com_read(BASE(objpF), 2);
  if (ret != DLMS_ERROR_CODE_OK)
  {
    blinkLINKOff();
    return ret;
  }
  blinkLINKOn();

  datameter.pf = var_toDouble(&objpF.value) * pow(10, objpF.scaler);
  Serial.printf("pF : %.3f\n", datameter.pf);
  obj_clear(BASE(objpF));

  cosem_init(BASE(objFreq), DLMS_OBJECT_TYPE_REGISTER, "1.0.14.7.0.255");
  ret = com_read(BASE(objFreq), 3);
  ret = com_read(BASE(objFreq), 2);
  if (ret != DLMS_ERROR_CODE_OK)
  {
    blinkLINKOff();
    return ret;
  }
  blinkLINKOn();

  datameter.freq = var_toDouble(&objFreq.value) * pow(10, objFreq.scaler);
  Serial.printf("Freq : %.3f\n", datameter.freq);
  obj_clear(BASE(objFreq));

  cosem_init(BASE(objKwh1), DLMS_OBJECT_TYPE_REGISTER, "1.0.1.8.0.255");
  ret = com_read(BASE(objKwh1), 3);
  ret = com_read(BASE(objKwh1), 2);
  if (ret != DLMS_ERROR_CODE_OK)
  {
    blinkLINKOff();
    return ret;
  }
  blinkLINKOn();

  datameter.kwh1 = var_toInteger(&objKwh1.value);
  Serial.printf("KWH1 : %d\n", datameter.kwh1);
  obj_clear(BASE(objKwh1));

  cosem_init(BASE(objKwh2), DLMS_OBJECT_TYPE_REGISTER, "1.0.2.8.0.255");
  ret = com_read(BASE(objKwh2), 3);
  ret = com_read(BASE(objKwh2), 2);
  if (ret != DLMS_ERROR_CODE_OK)
  {
    blinkLINKOff();
    return ret;
  }
  blinkLINKOn();

  datameter.kwh2 = var_toInteger(&objKwh2.value);
  Serial.printf("KWH2 : %d\n", datameter.kwh2);
  obj_clear(BASE(objKwh2));

  cosem_init(BASE(objKvar1), DLMS_OBJECT_TYPE_REGISTER, "1.0.3.8.0.255");
  ret = com_read(BASE(objKvar1), 3);
  ret = com_read(BASE(objKvar1), 2);
  if (ret != DLMS_ERROR_CODE_OK)
  {
    blinkLINKOff();
    return ret;
  }
  blinkLINKOn();

  datameter.kvar = var_toInteger(&objKvar1.value);
  Serial.printf("KVAR1 : %d\n", datameter.kvar);
  obj_clear(BASE(objKvar1));

  cosem_init(BASE(objKvar2), DLMS_OBJECT_TYPE_REGISTER, "1.0.4.8.0.255");
  ret = com_read(BASE(objKvar2), 3);
  ret = com_read(BASE(objKvar2), 2);
  if (ret != DLMS_ERROR_CODE_OK)
  {
    blinkLINKOff();
    return ret;
  }
  blinkLINKOn();

  datameter.kvar2 = var_toInteger(&objKvar2.value);
  Serial.printf("KVAR2 : %d\n", datameter.kvar2);
  obj_clear(BASE(objKvar2));

  // Release dynamically allocated objects.
  GuruxClient.ReleaseObjects();
  // com_close();
  return ret;
}

void InitKWHDLMS()
{
  BYTE_BUFFER_INIT(&frameData);
  // Set frame capacity.
  bb_capacity(&frameData, 128);
  GuruxClient.init(true, 4, 163, DLMS_AUTHENTICATION_HIGH_GMAC, NULL, DLMS_INTERFACE_TYPE_HDLC);
  GuruxClient.SetSecurity(DLMS_SECURITY_AUTHENTICATION_ENCRYPTION);
  gxByteBuffer bb;
  bb_Init(&bb);
  bb_addHexString(&bb, "41 42 43 44 45 46 47 48");
  GuruxClient.SetSystemTitle(&bb);
  bb_clear(&bb);
  bb_addHexString(&bb, "46 46 46 46 46 46 46 46 46 46 46 46 46 46 46 46");
  GuruxClient.SetAuthenticationKey(&bb);
  bb_clear(&bb);
  bb_addHexString(&bb, "46 46 46 46 46 46 46 46 46 46 46 46 46 46 46 46");
  GuruxClient.SetBlockCipherKey(&bb);
}

void InitKWHReading()
{
  if (!com_readStaticOnce())
  {
    blinkLINKOn();
    Serial.println("InitKWHReading(): Successfull");
    Serial.printf("KWH Serial Number: %s\n", datameter.serialnumber);
    Serial.printf("KWH Time: %d\n", datameter.timeUnix);
    kwhReadReady = true;
  }
  else
  {
    blinkLINKOff();
    Serial.println("InitKWHReading(): UnSuccessfull");
  }
  blinkLINKOff();
}

int KWHUpdateData()
{
  int ret_readAll = com_readAllObjects();
  blinkLINKOff();
  return ret_readAll;
}

void KWHDataSend(char *buffer)
{
  sprintf(buffer, "~*%d*%d*%s*%.3f*%d*%d*%.3f*%d*%d*%.3f*%d*%d*%.3f*%.3f*%d*%d*%d*%d#",
          1,
          settingVar.devId,
          datameter.serialnumber,
          datameter.volt,
          -1,
          -1,
          datameter.current,
          -1,
          -1,
          datameter.watt,
          -1,
          -1,
          datameter.pf,
          datameter.freq,
          datameter.kvar,
          datameter.kvar2,
          datameter.kwh1,
          datameter.kwh2);
  Serial.println(buffer);
}

void formatLogging(/* char *buffer */)
{
  char timeFormat[20];
  char buffer[255];

  sprintf(timeFormat, "%d/%d/%d,%d:%d:%d", year(), month(), day(), hour(), minute(), second());
  sprintf(buffer, "%s,%s,%s,%s,%s,%s,%s,%d,%d,%d,%d",
          timeFormat,
          datameter.serialnumber,
          String(datameter.volt, 2).c_str(),
          String(datameter.current, 2).c_str(),
          String(datameter.watt, 2).c_str(),
          String(datameter.pf, 2).c_str(),
          String(datameter.freq, 2).c_str(),
          datameter.kvar,
          datameter.kvar2,
          datameter.kwh1,
          datameter.kwh2);
  Serial.println(buffer);
}

void BackgroundDelay()
{
  kwhReadReady = true;
  // Serial.printf("TIME TO READ\n");
}

void setup()
{

  gxArray captureObjects;
  gxTarget CAPTURE_OBJECT[10] = {0};
  ARR_ATTACH(captureObjects, CAPTURE_OBJECT, 0);

  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  SPIFFS.begin();
  SettingsInit();
  pinMode(PIN_LED_WARN, OUTPUT);
  pinMode(PIN_LED_PROC, OUTPUT);
  preferences.begin("settings", false);
  fcnt = preferences.getUInt("lora_fcnt", 0);

  InitLora();
  InitKWHDLMS();
  InitKWHReading();

  setTime(datameter.timeUnix);
  Alarm.timerRepeat(60, BackgroundDelay); // timer for every 15 seconds
}

void loop()
{
  int ret_readAll;

  if (kwhReadReady)
  {
    ret_readAll = KWHUpdateData();
    // ret_com = com_close();
    if (ret_readAll == DLMS_ERROR_CODE_OK)
    {
      blinkACTIVEOn();
      char bufferData[255];
      KWHDataSend(bufferData);
      lora.sendUplink(bufferData, strlen(bufferData), 1, 1);
      lora.update();
      preferences.putUInt("lora_fcnt", lora.getFrameCounter());
      formatLogging();
      blinkACTIVEOff();
    }
    fcnt = preferences.getUInt("lora_fcnt", 0);
    Serial.printf("Frame Counter: %d\n", fcnt);
    kwhReadReady = false;
  }

  Alarm.delay(0);
}