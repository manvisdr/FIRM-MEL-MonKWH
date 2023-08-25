#include <Arduino.h>
#include <EEPROM.h>
#include "GXDLMSClient.h"
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
const uint32_t WAIT_TIME = 2000;
const uint8_t RESEND_COUNT = 3;
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

int com_read(gxObject *object, unsigned char attributeOrdinal);
int com_readSerialPort(unsigned char eop);
int com_initializeConnection();
uint16_t EEPROM_SIZE();
int EEPROM_READ(uint16_t index, unsigned char *value);
int EEPROM_WRITE(uint16_t index, unsigned char value);
void GXTRACE(const char *str, const char *data);
void GXTRACE_INT(const char *str, int32_t value);
uint32_t time_elapsed(void);
void time_now(gxtime *value);
int com_readSerialPort(unsigned char eop);
int readDLMSPacket(gxByteBuffer *data, gxReplyData *reply);
int com_readDataBlock(message *messages, gxReplyData *reply);
int com_close();
int com_updateInvocationCounter(const char *invocationCounter);
int com_initializeConnection();
int com_getAssociationView();
int com_read(gxObject *object, unsigned char attributeOrdinal);
int com_write(gxObject *object, unsigned char attributeOrdinal);
int com_method(gxObject *object, unsigned char attributeOrdinal, dlmsVARIANT *params);
int com_readList(gxArray *list);
int com_readRowsByEntry(gxProfileGeneric *object, unsigned long index, unsigned long count);
int com_readRowsByRange(gxProfileGeneric *object, gxtime *start, gxtime *end);
int com_readScalerAndUnits();
int com_readProfileGenericColumns();
int com_readProfileGenerics();
int com_readValues();
int com_readAllObjects(const char *invocationCounter);

uint16_t EEPROM_SIZE()
{
  return EEPROM.length();
}

int EEPROM_READ(uint16_t index, unsigned char *value)
{
  *value = EEPROM.read(index);
  return 0;
}

int EEPROM_WRITE(uint16_t index, unsigned char value)
{
  EEPROM.write(index, value);
  return 0;
}

///////////////////////////////////////////////////////////////////////
// Write trace to the serial port.
//
// This can be used for debugging.
///////////////////////////////////////////////////////////////////////
void GXTRACE(const char *str, const char *data)
{
  // Send trace to the serial port.
  byte c;
  Serial.write("\t:");
  while ((c = pgm_read_byte(str++)) != 0)
  {
    Serial.write(c);
  }
  if (data != NULL)
  {
    Serial.write(data);
  }
  Serial.write("\0");
  // Serial.flush();
  delay(10);
}

///////////////////////////////////////////////////////////////////////
// Write trace to the serial port.
//
// This can be used for debugging.
///////////////////////////////////////////////////////////////////////
void GXTRACE_INT(const char *str, int32_t value)
{
  char data[10];
  sprintf(data, " %ld", value);
  GXTRACE(str, data);
}

///////////////////////////////////////////////////////////////////////
// Returns the approximate processor time in ms.
///////////////////////////////////////////////////////////////////////
uint32_t time_elapsed(void)
{
  return millis();
}

///////////////////////////////////////////////////////////////////////
// Returns current time.
// Get current time.
// Because there is no clock, clock object keeps base time and uptime is added to that.
///////////////////////////////////////////////////////////////////////
void time_now(gxtime *value)
{
  time_addSeconds(value, time_elapsed() / 1000);
}

int com_readSerialPort(
    unsigned char eop)
{
  // Read reply data.
  uint16_t pos;
  uint16_t available;
  unsigned char eopFound = 0;
  uint16_t lastReadIndex = frameData.position;
  uint32_t start = millis();
  do
  {
    available = Serial2.available();
    if (available != 0)
    {
      if (frameData.size + available > frameData.capacity)
      {
        bb_capacity(&frameData, 20 + frameData.size + available);
      }
      Serial2.readBytes((char *)(frameData.data + frameData.size), available);
      frameData.size += available;
      // Search eop.
      if (frameData.size > 5)
      {
        // Some optical strobes can return extra bytes.
        for (pos = frameData.size - 1; pos != lastReadIndex; --pos)
        {
          if (frameData.data[pos] == eop)
          {
            eopFound = 1;
            break;
          }
        }
        lastReadIndex = pos;
      }
    }
    else
    {
      delay(50);
    }
    // If the meter doesn't reply in given time.
    if (!(millis() - start < WAIT_TIME))
    {
      GXTRACE_INT(GET_STR_FROM_EEPROM("Received bytes: \n"), frameData.size);
      return DLMS_ERROR_CODE_RECEIVE_FAILED;
    }
  } while (eopFound == 0);
  return DLMS_ERROR_CODE_OK;
}

// Read DLMS Data frame from the device.
int readDLMSPacket(
    gxByteBuffer *data,
    gxReplyData *reply)
{
  int resend = 0, ret = DLMS_ERROR_CODE_OK;
  if (data->size == 0)
  {
    return DLMS_ERROR_CODE_OK;
  }
  reply->complete = 0;
  frameData.size = 0;
  frameData.position = 0;
  // Send data.
  if ((ret = Serial2.write(data->data, data->size)) != data->size)
  {
    // If failed to write all bytes.
    GXTRACE(GET_STR_FROM_EEPROM("Failed to write all data to the serial port.\n"), NULL);
  }
  // Loop until packet is complete.
  do
  {
    if ((ret = com_readSerialPort(0x7E)) != 0)
    {
      if (ret == DLMS_ERROR_CODE_RECEIVE_FAILED && resend == RESEND_COUNT)
      {
        return DLMS_ERROR_CODE_SEND_FAILED;
      }
      ++resend;
      GXTRACE_INT(GET_STR_FROM_EEPROM("Data send failed. Try to resend."), resend);
      if ((ret = Serial2.write(data->data, data->size)) != data->size)
      {
        // If failed to write all bytes.
        GXTRACE(GET_STR_FROM_EEPROM("Failed to write all data to the serial port.\n"), NULL);
      }
    }
    ret = GuruxClient.GetData(&frameData, reply);
    if (ret != 0 && ret != DLMS_ERROR_CODE_FALSE)
    {
      break;
    }
  } while (reply->complete == 0);
  return ret;
}

int com_readDataBlock(
    message *messages,
    gxReplyData *reply)
{
  gxByteBuffer rr;
  int pos, ret = DLMS_ERROR_CODE_OK;
  // If there is no data to send.
  if (messages->size == 0)
  {
    return DLMS_ERROR_CODE_OK;
  }
  BYTE_BUFFER_INIT(&rr);
  // Send data.
  for (pos = 0; pos != messages->size; ++pos)
  {
    // Send data.
    if ((ret = readDLMSPacket(messages->data[pos], reply)) != DLMS_ERROR_CODE_OK)
    {
      return ret;
    }
    // Check is there errors or more data from server
    while (reply_isMoreData(reply))
    {
      if ((ret = GuruxClient.ReceiverReady(reply->moreData, &rr)) != DLMS_ERROR_CODE_OK)
      {
        bb_clear(&rr);
        return ret;
      }
      if ((ret = readDLMSPacket(&rr, reply)) != DLMS_ERROR_CODE_OK)
      {
        bb_clear(&rr);
        return ret;
      }
      bb_clear(&rr);
    }
  }
  return ret;
}

// Close connection to the meter.
int com_close()
{
  int ret = DLMS_ERROR_CODE_OK;
  gxReplyData reply;
  message msg;
  reply_init(&reply);
  mes_init(&msg);
  if ((ret = GuruxClient.ReleaseRequest(true, &msg)) != 0 ||
      (ret = com_readDataBlock(&msg, &reply)) != 0)
  {
    // Show error but continue close.
  }
  reply_clear(&reply);
  mes_clear(&msg);

  if ((ret = GuruxClient.DisconnectRequest(&msg)) != 0 ||
      (ret = com_readDataBlock(&msg, &reply)) != 0)
  {
    // Show error but continue close.
  }
  reply_clear(&reply);
  mes_clear(&msg);
  return ret;
}

// Read Invocation counter (frame counter) from the meter and update it.
int com_updateInvocationCounter(const char *invocationCounter)
{
  int ret = DLMS_ERROR_CODE_OK;
  // Read frame counter if security is used.
  if (invocationCounter != NULL && GuruxClient.GetSecurity() != DLMS_SECURITY_NONE)
  {
    uint32_t ic = 0;
    message messages;
    gxReplyData reply;
    unsigned short add = GuruxClient.GetClientAddress();
    DLMS_AUTHENTICATION auth = GuruxClient.GetAuthentication();
    DLMS_SECURITY security = GuruxClient.GetSecurity();
    GuruxClient.SetClientAddress(16);
    GuruxClient.SetAuthentication(DLMS_AUTHENTICATION_NONE);
    GuruxClient.SetSecurity(DLMS_SECURITY_NONE);
    if ((ret = com_initializeConnection()) == DLMS_ERROR_CODE_OK)
    {
      gxData d;
      cosem_init(BASE(d), DLMS_OBJECT_TYPE_DATA, invocationCounter);
      if ((ret = com_read(BASE(d), 2)) == 0)
      {
        GXTRACE_INT(GET_STR_FROM_EEPROM("Invocation Counter:"), var_toInteger(&d.value));
        ic = 1 + var_toInteger(&d.value);
      }
      obj_clear(BASE(d));
    }
    // Close connection. It's OK if this fails.
    com_close();
    // Return original settings.
    GuruxClient.SetClientAddress(add);
    GuruxClient.SetAuthentication(auth);
    GuruxClient.SetSecurity(security);
    GuruxClient.SetInvocationCounter(ic);
  }
  return ret;
}

// Initialize connection to the meter.
int com_initializeConnection()
{
  int ret = DLMS_ERROR_CODE_OK;
  message messages;
  gxReplyData reply;
  mes_init(&messages);
  reply_init(&reply);

#ifndef DLMS_IGNORE_HDLC
  // Get meter's send and receive buffers size.
  if ((ret = GuruxClient.SnrmRequest(&messages)) != 0 ||
      (ret = com_readDataBlock(&messages, &reply)) != 0 ||
      (ret = GuruxClient.ParseUAResponse(&reply.data)) != 0)
  {
    mes_clear(&messages);
    reply_clear(&reply);
    return ret;
  }
  mes_clear(&messages);
  reply_clear(&reply);
#endif // DLMS_IGNORE_HDLC

  if ((ret = GuruxClient.AarqRequest(&messages)) != 0 ||
      (ret = com_readDataBlock(&messages, &reply)) != 0 ||
      (ret = GuruxClient.ParseAAREResponse(&reply.data)) != 0)
  {
    mes_clear(&messages);
    reply_clear(&reply);
    if (ret == DLMS_ERROR_CODE_APPLICATION_CONTEXT_NAME_NOT_SUPPORTED)
    {
      return ret;
    }
    return ret;
  }
  mes_clear(&messages);
  reply_clear(&reply);
  // Get challenge Is HLS authentication is used.
  if (GuruxClient.GetAuthentication() > DLMS_AUTHENTICATION_LOW)
  {
    if ((ret = GuruxClient.GetApplicationAssociationRequest(&messages)) != 0 ||
        (ret = com_readDataBlock(&messages, &reply)) != 0 ||
        (ret = GuruxClient.ParseApplicationAssociationResponse(&reply.data)) != 0)
    {
      mes_clear(&messages);
      reply_clear(&reply);
      return ret;
    }
    mes_clear(&messages);
    reply_clear(&reply);
  }
  return DLMS_ERROR_CODE_OK;
}

// Get Association view.
int com_getAssociationView()
{
  int ret;
  message data;
  gxReplyData reply;
  mes_init(&data);
  reply_init(&reply);
  if ((ret = GuruxClient.GetObjectsRequest(&data)) != 0 ||
      (ret = com_readDataBlock(&data, &reply)) != 0 ||
      (ret = GuruxClient.ParseObjects(&reply.data)) != 0)
  {
  }
  mes_clear(&data);
  reply_clear(&reply);
  return ret;
}

// Read object.
int com_read(
    gxObject *object,
    unsigned char attributeOrdinal)
{
  int ret;
  message data;
  gxReplyData reply;
  mes_init(&data);
  reply_init(&reply);
  if ((ret = GuruxClient.Read(object, attributeOrdinal, &data)) != 0 ||
      (ret = com_readDataBlock(&data, &reply)) != 0 ||
      (ret = GuruxClient.UpdateValue(object, attributeOrdinal, &reply.dataValue)) != 0)
  {
    GXTRACE_INT(GET_STR_FROM_EEPROM("com_read failed."), ret);
  }
  mes_clear(&data);
  reply_clear(&reply);
  return ret;
}

int com_write(
    gxObject *object,
    unsigned char attributeOrdinal)
{
  int ret;
  message data;
  gxReplyData reply;
  mes_init(&data);
  reply_init(&reply);
  if ((ret = GuruxClient.Write(object, attributeOrdinal, &data)) != 0 ||
      (ret = com_readDataBlock(&data, &reply)) != 0)
  {
    GXTRACE_INT(GET_STR_FROM_EEPROM("com_write failed."), ret);
  }
  mes_clear(&data);
  reply_clear(&reply);
  return ret;
}

int com_method(
    gxObject *object,
    unsigned char attributeOrdinal,
    dlmsVARIANT *params)
{
  int ret;
  message messages;
  gxReplyData reply;
  mes_init(&messages);
  reply_init(&reply);
  if ((ret = GuruxClient.Method(object, attributeOrdinal, params, &messages)) != 0 ||
      (ret = com_readDataBlock(&messages, &reply)) != 0)
  {
    GXTRACE_INT(GET_STR_FROM_EEPROM("com_method failed."), ret);
  }
  mes_clear(&messages);
  reply_clear(&reply);
  return ret;
}

// Read objects.
int com_readList(
    gxArray *list)
{
  int pos, ret = DLMS_ERROR_CODE_OK;
  gxByteBuffer bb, rr;
  message messages;
  gxReplyData reply;
  if (list->size != 0)
  {
    mes_init(&messages);
    if ((ret = GuruxClient.ReadList(list, &messages)) != 0)
    {
      GXTRACE_INT(GET_STR_FROM_EEPROM("com_readList failed."), ret);
    }
    else
    {
      reply_init(&reply);
      BYTE_BUFFER_INIT(&rr);
      BYTE_BUFFER_INIT(&bb);
      // Send data.
      for (pos = 0; pos != messages.size; ++pos)
      {
        // Send data.
        reply_clear(&reply);
        if ((ret = readDLMSPacket(messages.data[pos], &reply)) != DLMS_ERROR_CODE_OK)
        {
          break;
        }
        // Check is there errors or more data from server
        while (reply_isMoreData(&reply))
        {
          if ((ret = GuruxClient.ReceiverReady(reply.moreData, &rr)) != DLMS_ERROR_CODE_OK ||
              (ret = readDLMSPacket(&rr, &reply)) != DLMS_ERROR_CODE_OK)
          {
            break;
          }
          bb_clear(&rr);
        }
        bb_set2(&bb, &reply.data, reply.data.position, -1);
      }
      if (ret == 0)
      {
        ret = GuruxClient.UpdateValues(list, &bb);
      }
      bb_clear(&bb);
      bb_clear(&rr);
      reply_clear(&reply);
    }
    mes_clear(&messages);
  }
  return ret;
}

int com_readRowsByEntry(
    gxProfileGeneric *object,
    unsigned long index,
    unsigned long count)
{
  int ret;
  message data;
  gxReplyData reply;
  mes_init(&data);
  reply_init(&reply);
  if ((ret = GuruxClient.ReadRowsByEntry(object, index, count, &data)) != 0 ||
      (ret = com_readDataBlock(&data, &reply)) != 0 ||
      (ret = GuruxClient.UpdateValue((gxObject *)object, 2, &reply.dataValue)) != 0)
  {
    GXTRACE_INT(GET_STR_FROM_EEPROM("com_readRowsByEntry failed."), ret);
  }
  mes_clear(&data);
  reply_clear(&reply);
  return ret;
}

///////////////////////////////////////////////////////////////////////////////////
int com_readRowsByRange(
    gxProfileGeneric *object,
    gxtime *start,
    gxtime *end)
{
  int ret;
  message data;
  gxReplyData reply;
  mes_init(&data);
  reply_init(&reply);
  if ((ret = GuruxClient.ReadRowsByRange(object, start, end, &data)) != 0 ||
      (ret = com_readDataBlock(&data, &reply)) != 0 ||
      (ret = GuruxClient.UpdateValue((gxObject *)object, 2, &reply.dataValue)) != 0)
  {
    GXTRACE_INT(GET_STR_FROM_EEPROM("com_readRowsByRange failed."), ret);
  }
  mes_clear(&data);
  reply_clear(&reply);
  return ret;
}

///////////////////////////////////////////////////////////////////////////////////
// Read scalers and units. They are static so they are read only once.
int com_readScalerAndUnits()
{
  gxObject *obj;
  int ret, pos;
  objectArray objects;
  gxArray list;
  gxObject *object;
  DLMS_OBJECT_TYPE types[] = {DLMS_OBJECT_TYPE_EXTENDED_REGISTER, DLMS_OBJECT_TYPE_REGISTER, DLMS_OBJECT_TYPE_DEMAND_REGISTER};
  oa_init(&objects);
  // Find registers and demand registers and read them.
  ret = oa_getObjects2(GuruxClient.GetObjects(), types, 3, &objects);
  if (ret != DLMS_ERROR_CODE_OK)
  {
    return ret;
  }
  if ((GuruxClient.GetNegotiatedConformance() & DLMS_CONFORMANCE_MULTIPLE_REFERENCES) != 0)
  {
    arr_init(&list);
    // Try to read with list first. All meters do not support it.
    for (pos = 0; pos != GuruxClient.GetObjects()->size; ++pos)
    {
      ret = oa_getByIndex(GuruxClient.GetObjects(), pos, &obj);
      if (ret != DLMS_ERROR_CODE_OK)
      {
        oa_empty(&objects);
        arr_clear(&list);
        return ret;
      }
      if (obj->objectType == DLMS_OBJECT_TYPE_REGISTER ||
          obj->objectType == DLMS_OBJECT_TYPE_EXTENDED_REGISTER)
      {
        arr_push(&list, key_init(obj, (void *)3));
      }
      else if (obj->objectType == DLMS_OBJECT_TYPE_DEMAND_REGISTER)
      {
        arr_push(&list, key_init(obj, (void *)4));
      }
    }
    ret = com_readList(&list);
    arr_clear(&list);
  }
  // If read list failed read items one by one.
  if (ret != 0)
  {
    for (pos = 0; pos != objects.size; ++pos)
    {
      ret = oa_getByIndex(&objects, pos, &object);
      if (ret != DLMS_ERROR_CODE_OK)
      {
        oa_empty(&objects);
        return ret;
      }
      ret = com_read(object, object->objectType == DLMS_OBJECT_TYPE_DEMAND_REGISTER ? 4 : 3);
      if (ret != DLMS_ERROR_CODE_OK)
      {
        oa_empty(&objects);
        return ret;
      }
    }
  }
  // Do not clear objects list because it will free also objects from association view list.
  oa_empty(&objects);
  return ret;
}

///////////////////////////////////////////////////////////////////////////////////
// Read profile generic columns. They are static so they are read only once.
int com_readProfileGenericColumns()
{
  int ret, pos;
  objectArray objects;
  gxObject *object;
  oa_init(&objects);
  ret = oa_getObjects(GuruxClient.GetObjects(), DLMS_OBJECT_TYPE_PROFILE_GENERIC, &objects);
  if (ret != DLMS_ERROR_CODE_OK)
  {
    oa_empty(&objects);
    return ret;
  }
  for (pos = 0; pos != objects.size; ++pos)
  {
    ret = oa_getByIndex(&objects, pos, &object);
    if (ret != DLMS_ERROR_CODE_OK)
    {
      break;
    }
    ret = com_read(object, 3);
    if (ret != DLMS_ERROR_CODE_OK)
    {
      break;
    }
  }
  // Do not clear objects list because it will free also objects from association view list.
  oa_empty(&objects);
  return ret;
}

///////////////////////////////////////////////////////////////////////////////////
// Read profile generics rows.
int com_readProfileGenerics()
{
  gxtime startTime, endTime;
  int ret, pos;
  char str[50];
  char ln[25];
  char *data = NULL;
  gxByteBuffer ba;
  objectArray objects;
  gxProfileGeneric *pg;
  oa_init(&objects);
  ret = oa_getObjects(GuruxClient.GetObjects(), DLMS_OBJECT_TYPE_PROFILE_GENERIC, &objects);
  if (ret != DLMS_ERROR_CODE_OK)
  {
    // Do not clear objects list because it will free also objects from association view list.
    oa_empty(&objects);
    return ret;
  }
  BYTE_BUFFER_INIT(&ba);
  for (pos = 0; pos != objects.size; ++pos)
  {
    ret = oa_getByIndex(&objects, pos, (gxObject **)&pg);
    if (ret != DLMS_ERROR_CODE_OK)
    {
      // Do not clear objects list because it will free also objects from association view list.
      oa_empty(&objects);
      return ret;
    }
    // Read entries in use.
    ret = com_read((gxObject *)pg, 7);
    if (ret != DLMS_ERROR_CODE_OK)
    {
      printf("Failed to read object %s %s attribute index %d\r\n", str, ln, 7);
      // Do not clear objects list because it will free also objects from association view list.
      oa_empty(&objects);
      return ret;
    }
    // Read entries.
    ret = com_read((gxObject *)pg, 8);
    if (ret != DLMS_ERROR_CODE_OK)
    {
      printf("Failed to read object %s %s attribute index %d\r\n", str, ln, 8);
      // Do not clear objects list because it will free also objects from association view list.
      oa_empty(&objects);
      return ret;
    }
    printf("Entries: %ld/%ld\r\n", pg->entriesInUse, pg->profileEntries);
    // If there are no columns or rows.
    if (pg->entriesInUse == 0 || pg->captureObjects.size == 0)
    {
      continue;
    }
    // Read first row from Profile Generic.
    ret = com_readRowsByEntry(pg, 1, 1);
    // Read last day from Profile Generic.
    time_now(&startTime);
    endTime = startTime;
    time_clearTime(&startTime);
    ret = com_readRowsByRange(pg, &startTime, &endTime);
  }
  // Do not clear objects list because it will free also objects from association view list.
  oa_empty(&objects);
  return ret;
}

// This function reads ALL objects that meter have excluded profile generic objects.
// It will loop all object's attributes.
int com_readValues()
{
  gxByteBuffer attributes;
  unsigned char ch;
  char *data = NULL;
  gxObject *object;
  unsigned long index;
  int ret, pos;
  BYTE_BUFFER_INIT(&attributes);

  for (pos = 0; pos != GuruxClient.GetObjects()->size; ++pos)
  {
    ret = oa_getByIndex(GuruxClient.GetObjects(), pos, &object);
    if (ret != DLMS_ERROR_CODE_OK)
    {
      bb_clear(&attributes);
      return ret;
    }
    ///////////////////////////////////////////////////////////////////////////////////
    // Profile generics are read later because they are special cases.
    // (There might be so lots of data and we so not want waste time to read all the data.)
    if (object->objectType == DLMS_OBJECT_TYPE_PROFILE_GENERIC)
    {
      continue;
    }
    ret = obj_getAttributeIndexToRead(object, &attributes);
    if (ret != DLMS_ERROR_CODE_OK)
    {
      bb_clear(&attributes);
      return ret;
    }
    for (index = 0; index < attributes.size; ++index)
    {
      ret = bb_getUInt8ByIndex(&attributes, index, &ch);
      if (ret != DLMS_ERROR_CODE_OK)
      {
        bb_clear(&attributes);
        return ret;
      }
      ret = com_read(object, ch);
      if (ret != DLMS_ERROR_CODE_OK)
      {
        // Return error if not DLMS error.
        if (ret != DLMS_ERROR_CODE_READ_WRITE_DENIED)
        {
          bb_clear(&attributes);
          return ret;
        }
        ret = 0;
      }
    }
    bb_clear(&attributes);
  }
  bb_clear(&attributes);
  return ret;
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
  // cosem_init(BASE(sn), DLMS_OBJECT_TYPE_DATA, "0.0.96.1.1.255");
  cosem_init(BASE(sn), DLMS_OBJECT_TYPE_DATA, "0.0.96.1.0.255");
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

int readProfileGenerics()
{
  int ret;

  ret = com_initializeConnection();
  if (ret != DLMS_ERROR_CODE_OK)
  {
    GXTRACE_INT(GET_STR_FROM_EEPROM("com_initializeConnection failed"), ret);
    return ret;
  }
  GXTRACE_INT(GET_STR_FROM_EEPROM("com_initializeConnection SUCCEEDED"), ret);

  char* data = NULL;
  gxProfileGeneric pg;
  cosem_init(BASE(pg), DLMS_OBJECT_TYPE_PROFILE_GENERIC, "1.0.99.1.0.255");
  com_read(BASE(pg), 3);
  com_readRowsByEntry(&pg,4, 5);
  obj_toString(BASE(pg), &data);
  GXTRACE(GET_STR_FROM_EEPROM("Load profile"), data);
  obj_clear(BASE(pg));
  free(data);

  // Release dynamically allocated objects.
  GuruxClient.ReleaseObjects();
  return ret;
}

// void InitKWHDLMS()
// {
//   BYTE_BUFFER_INIT(&frameData);
//   // Set frame capacity.
//   bb_capacity(&frameData, 128);
//   GuruxClient.init(true, 4, 163, DLMS_AUTHENTICATION_HIGH_GMAC, NULL, DLMS_INTERFACE_TYPE_HDLC);
//   GuruxClient.SetSecurity(DLMS_SECURITY_AUTHENTICATION_ENCRYPTION);
//   gxByteBuffer bb;
//   bb_Init(&bb);
//   bb_addHexString(&bb, "41 42 43 44 45 46 47 48");
//   GuruxClient.SetSystemTitle(&bb);
//   bb_clear(&bb);
//   bb_addHexString(&bb, "46 46 46 46 46 46 46 46 46 46 46 46 46 46 46 46");
//   GuruxClient.SetAuthenticationKey(&bb);
//   bb_clear(&bb);
//   bb_addHexString(&bb, "46 46 46 46 46 46 46 46 46 46 46 46 46 46 46 46");
//   GuruxClient.SetBlockCipherKey(&bb);
// }

void InitKWHDLMS()
{
  BYTE_BUFFER_INIT(&frameData);
  // Set frame capacity.
  bb_capacity(&frameData, 128);
  GuruxClient.init(true, 48, 28672, DLMS_AUTHENTICATION_LOW, "22222222", DLMS_INTERFACE_TYPE_HDLC);
  GuruxClient.SetSecurity(DLMS_SECURITY_NONE);
  // GuruxClient.init(true, 4, 163, DLMS_AUTHENTICATION_HIGH_GMAC, NULL, DLMS_INTERFACE_TYPE_HDLC);
  // GuruxClient.SetSecurity(DLMS_SECURITY_AUTHENTICATION_ENCRYPTION);

  gxByteBuffer bb;
  // bb_Init(&bb);
  // bb_addHexString(&bb, "41 42 43 44 45 46 47 48");
  // GuruxClient.SetSystemTitle(&bb);
  // bb_clear(&bb);
  // bb_addHexString(&bb, "46 46 46 46 46 46 46 46 46 46 46 46 46 46 46 46");
  // GuruxClient.SetAuthenticationKey(&bb);
  // bb_clear(&bb);
  // bb_addHexString(&bb, "46 46 46 46 46 46 46 46 46 46 46 46 46 46 46 46");
  // GuruxClient.SetBlockCipherKey(&bb);
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
  // SPIFFS.begin();
  // SettingsInit();
  // pinMode(PIN_LED_WARN, OUTPUT);
  // pinMode(PIN_LED_PROC, OUTPUT);
  // preferences.begin("settings", false);
  // fcnt = preferences.getUInt("lora_fcnt", 0);

  // InitLora();
  InitKWHDLMS();
  InitKWHReading();

  setTime(datameter.timeUnix);
  Alarm.timerRepeat(20, BackgroundDelay); // timer for every 15 seconds
}

void loop()
{
  int ret_readAll;

  if (kwhReadReady)
  {
    // ret_readAll = KWHUpdateData();
    readProfileGenerics();
    // ret_com = com_close();
    if (ret_readAll == DLMS_ERROR_CODE_OK)
    {
      blinkACTIVEOn();
      char bufferData[255];
      // KWHDataSend(bufferData);
      // lora.sendUplink(bufferData, strlen(bufferData), 1, 1);
      // lora.update();
      // preferences.putUInt("lora_fcnt", lora.getFrameCounter());
      blinkACTIVEOff();
    }
    // fcnt = preferences.getUInt("lora_fcnt", 0);
    // Serial.printf("Frame Counter: %d\n", fcnt);
    kwhReadReady = false;
  }

  Alarm.delay(0);
}