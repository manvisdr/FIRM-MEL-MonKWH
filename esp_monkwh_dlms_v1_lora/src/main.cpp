#include <Arduino.h>
#include <EEPROM.h>
#include "GXDLMSClient.h"
#include <TimeLib.h>
#include <TimeAlarms.h>
#include <SPI.h>
#include <lorawan.h>
#include <Preferences.h>

Preferences preferences;

struct dataMeter
{
  uint32_t timeUnix;
  String serialnumber;
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

dataMeter datameter;

uint32_t unixTime;
ulong deviceNumber;

char buffData[255];

#define csLora 5    // PIN CS LORA
#define MODE_LORA 0 // LORA
// #define US_915
// DEF PIN
#define RXD2 16         // PIN RX2
#define TXD2 17         // PIN TX2
#define PIN_LED_PROC 27 // PIN RX2
#define PIN_LED_WARN 15 // PIN TX2
const sRFM_pins RFM_pins = {
    .CS = 5,
    .RST = 26,
    .DIO0 = 25,
    .DIO1 = 32,
}; // DEF PIN LORA

String customerID;
String kwhID;
const String channelId = "monKWH/";
const char *devAddr = "29CBEBB1";
// const char *devAddr = "01511111"; monkwh mk7 legi
const char *nwkSKey = "927B1EF8E776A54185C1A51FDD43DFCB";
const char *appSKey = "AFF8978601D9C13C3FB153C5EF2DF551";

gxByteBuffer frameData;
const uint32_t WAIT_TIME = 2000;
const uint8_t RESEND_COUNT = 3;
uint32_t runTime = 0;
bool kwhReadReady = false;
unsigned int fcnt;

bool loraConf()
{
  Serial.println(F("LORA CONFIG"));
  lora.setDeviceClass(CLASS_C);
  lora.setDataRate(SF7BW125);
  lora.setChannel(MULTI);
  lora.setNwkSKey(nwkSKey);
  lora.setAppSKey(appSKey);
  lora.setDevAddr(devAddr);
  lora.setFrameCounter(fcnt);
  return true;
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
    ret = Client.GetData(&frameData, reply);
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
      if ((ret = Client.ReceiverReady(reply->moreData, &rr)) != DLMS_ERROR_CODE_OK)
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
  if ((ret = Client.ReleaseRequest(true, &msg)) != 0 ||
      (ret = com_readDataBlock(&msg, &reply)) != 0)
  {
    // Show error but continue close.
  }
  reply_clear(&reply);
  mes_clear(&msg);

  if ((ret = Client.DisconnectRequest(&msg)) != 0 ||
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
  if (invocationCounter != NULL && Client.GetSecurity() != DLMS_SECURITY_NONE)
  {
    uint32_t ic = 0;
    message messages;
    gxReplyData reply;
    unsigned short add = Client.GetClientAddress();
    DLMS_AUTHENTICATION auth = Client.GetAuthentication();
    DLMS_SECURITY security = Client.GetSecurity();
    Client.SetClientAddress(16);
    Client.SetAuthentication(DLMS_AUTHENTICATION_NONE);
    Client.SetSecurity(DLMS_SECURITY_NONE);
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
    Client.SetClientAddress(add);
    Client.SetAuthentication(auth);
    Client.SetSecurity(security);
    Client.SetInvocationCounter(ic);
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
  if ((ret = Client.SnrmRequest(&messages)) != 0 ||
      (ret = com_readDataBlock(&messages, &reply)) != 0 ||
      (ret = Client.ParseUAResponse(&reply.data)) != 0)
  {
    mes_clear(&messages);
    reply_clear(&reply);
    return ret;
  }
  mes_clear(&messages);
  reply_clear(&reply);
#endif // DLMS_IGNORE_HDLC

  if ((ret = Client.AarqRequest(&messages)) != 0 ||
      (ret = com_readDataBlock(&messages, &reply)) != 0 ||
      (ret = Client.ParseAAREResponse(&reply.data)) != 0)
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
  if (Client.GetAuthentication() > DLMS_AUTHENTICATION_LOW)
  {
    if ((ret = Client.GetApplicationAssociationRequest(&messages)) != 0 ||
        (ret = com_readDataBlock(&messages, &reply)) != 0 ||
        (ret = Client.ParseApplicationAssociationResponse(&reply.data)) != 0)
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
  if ((ret = Client.GetObjectsRequest(&data)) != 0 ||
      (ret = com_readDataBlock(&data, &reply)) != 0 ||
      (ret = Client.ParseObjects(&reply.data)) != 0)
  {
  }
  // Parse object one at the time. This can be used if there is a limited amount of the memory available.
  /*
    //Value is not parsed for decrease the memory usage.
    reply.ignoreValue = 1;
    if ((ret = Client.GetObjectsRequest(&data)) == 0 &&
      (ret = com_readDataBlock(&data, &reply)) == 0)
    {
    uint16_t pos, count;
    if ((ret = Client.ParseObjectCount(&reply.data, &count)) != 0)
    {
      GXTRACE_INT("cl_parseObjectCount failed", ret);
    }
    gxObject obj;
    for (pos = 0; pos != count; ++pos)
    {
      memset(&obj, 0, sizeof(gxObject));
      if ((ret = Client.ParseNextObject(&reply.data, &obj)) != 0)
      {
        break;
      }
      //Only data and register objects are added to the association view.
      if (obj.objectType == DLMS_OBJECT_TYPE_DATA)
      {
        gxData* data = (gxData*) malloc(sizeof(gxData));
        INIT_OBJECT((*data), (DLMS_OBJECT_TYPE) obj.objectType, obj.logicalName);
        data->base.shortName = obj.shortName;
        oa_push(Client.GetObjects(), BASE((*data)));
      }
      else if (obj.objectType == DLMS_OBJECT_TYPE_REGISTER)
      {
        gxRegister* r = (gxRegister*) malloc(sizeof(gxRegister));
        INIT_OBJECT((*r), (DLMS_OBJECT_TYPE) obj.objectType, obj.logicalName);
        r->base.shortName = obj.shortName;
        oa_push(Client.GetObjects(), BASE((*r)));
      }
    }
    }
  */
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
  if ((ret = Client.Read(object, attributeOrdinal, &data)) != 0 ||
      (ret = com_readDataBlock(&data, &reply)) != 0 ||
      (ret = Client.UpdateValue(object, attributeOrdinal, &reply.dataValue)) != 0)
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
  if ((ret = Client.Write(object, attributeOrdinal, &data)) != 0 ||
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
  if ((ret = Client.Method(object, attributeOrdinal, params, &messages)) != 0 ||
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
    if ((ret = Client.ReadList(list, &messages)) != 0)
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
          if ((ret = Client.ReceiverReady(reply.moreData, &rr)) != DLMS_ERROR_CODE_OK ||
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
        ret = Client.UpdateValues(list, &bb);
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
  if ((ret = Client.ReadRowsByEntry(object, index, count, &data)) != 0 ||
      (ret = com_readDataBlock(&data, &reply)) != 0 ||
      (ret = Client.UpdateValue((gxObject *)object, 2, &reply.dataValue)) != 0)
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
  if ((ret = Client.ReadRowsByRange(object, start, end, &data)) != 0 ||
      (ret = com_readDataBlock(&data, &reply)) != 0 ||
      (ret = Client.UpdateValue((gxObject *)object, 2, &reply.dataValue)) != 0)
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
  ret = oa_getObjects2(Client.GetObjects(), types, 3, &objects);
  if (ret != DLMS_ERROR_CODE_OK)
  {
    return ret;
  }
  if ((Client.GetNegotiatedConformance() & DLMS_CONFORMANCE_MULTIPLE_REFERENCES) != 0)
  {
    arr_init(&list);
    // Try to read with list first. All meters do not support it.
    for (pos = 0; pos != Client.GetObjects()->size; ++pos)
    {
      ret = oa_getByIndex(Client.GetObjects(), pos, &obj);
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
  ret = oa_getObjects(Client.GetObjects(), DLMS_OBJECT_TYPE_PROFILE_GENERIC, &objects);
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
  ret = oa_getObjects(Client.GetObjects(), DLMS_OBJECT_TYPE_PROFILE_GENERIC, &objects);
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

  for (pos = 0; pos != Client.GetObjects()->size; ++pos)
  {
    ret = oa_getByIndex(Client.GetObjects(), pos, &object);
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
  GXTRACE_INT(GET_STR_FROM_EEPROM("com_initializeConnection SUCCEEDED"), ret);
  GXTRACE_INT(GET_STR_FROM_EEPROM("\n"), ret);

  // Read just wanted objects withour serializating the association view.
  char *data = NULL;
  gxByteBuffer snBuff;
  bb_Init(&snBuff);

  // Read clock
  gxClock clock1;
  cosem_init(BASE(clock1), DLMS_OBJECT_TYPE_CLOCK, "0.0.1.0.0.255");
  com_read(BASE(clock1), 3);
  com_read(BASE(clock1), 2);
  datameter.timeUnix = time_toUnixTime2(&clock1.time);
  obj_toString(BASE(clock1), &data);
  // GXTRACE(GET_STR_FROM_EEPROM("Clock"), data);
  Serial.println(datameter.timeUnix);
  obj_clear(BASE(clock1));
  free(data);
  blinkProcces(1);

  gxData sn;
  cosem_init(BASE(sn), DLMS_OBJECT_TYPE_DATA, "0.0.96.1.1.255");
  com_read(BASE(sn), 2);
  obj_toString(BASE(sn), &data);
  // GXTRACE(GET_STR_FROM_EEPROM("Serial Number"), data);
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
  Serial.println(F("----------------------------"));

  while (token != NULL)
  {
    Serial.print(token);
    strcat(result, token);
    token = strtok(NULL, s);
  }
  Serial.println();
  char *prtst = result;
  for (byte i = 0; i < 12; i++)
  {
    output[i] = aNibble(*prtst++);
    output[i] <<= 4;
    output[i] |= aNibble(*prtst++);
  }
  datameter.serialnumber = output;
  Serial.println(datameter.serialnumber);
  Serial.println(F("----------------------------"));

  free(pStr);
  obj_clear(BASE(sn));
  free(data);
  Client.ReleaseObjects();
  com_close();
  blinkProcces(1);
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
  blinkProcces(3);
  GXTRACE_INT(GET_STR_FROM_EEPROM("com_initializeConnection SUCCEEDED"), ret);
  GXTRACE_INT(GET_STR_FROM_EEPROM("\n"), ret);

  // Read just wanted objects withour serializating the association view.
  char *data = NULL;

  gxRegister volt;
  cosem_init(BASE(volt), DLMS_OBJECT_TYPE_REGISTER, "1.0.32.7.0.255");
  com_read(BASE(volt), 3);
  com_read(BASE(volt), 2);
  obj_toString(BASE(volt), &data);
  // GXTRACE(GET_STR_FROM_EEPROM("volt >> "), data);
  // Serial.println(var_toDouble(&volt.value) * pow(10, volt.scaler));
  datameter.volt = var_toDouble(&volt.value) * pow(10, volt.scaler);
  Serial.printf("Volt : %.3f\n", datameter.volt);
  obj_clear(BASE(volt));
  free(data);
  blinkProcces(1);

  gxRegister current;
  cosem_init(BASE(current), DLMS_OBJECT_TYPE_REGISTER, "1.0.31.7.0.255");
  com_read(BASE(current), 3);
  com_read(BASE(current), 2);
  obj_toString(BASE(current), &data);
  // GXTRACE(GET_STR_FROM_EEPROM("current >> "), data);
  // Serial.println(var_toDouble(&current.value) * pow(10, current.scaler));
  datameter.current = var_toDouble(&current.value) * pow(10, current.scaler);
  Serial.printf("Current : %.3f\n", datameter.current);
  obj_clear(BASE(current));
  free(data);
  blinkProcces(1);

  gxRegister watt;
  cosem_init(BASE(watt), DLMS_OBJECT_TYPE_REGISTER, "1.0.21.7.0.255");
  com_read(BASE(watt), 3);
  com_read(BASE(watt), 2);
  obj_toString(BASE(watt), &data);
  // GXTRACE(GET_STR_FROM_EEPROM("watt >> "), data);
  // Serial.println(var_toDouble(&watt.value) * pow(10, watt.scaler));
  datameter.watt = var_toDouble(&watt.value) * pow(10, watt.scaler);
  Serial.printf("Watt : %.3f\n", datameter.watt);
  obj_clear(BASE(watt));
  free(data);
  blinkProcces(1);

  gxRegister pF;
  cosem_init(BASE(pF), DLMS_OBJECT_TYPE_REGISTER, "1.0.33.7.0.255");
  com_read(BASE(pF), 3);
  com_read(BASE(pF), 2);
  obj_toString(BASE(pF), &data);
  // GXTRACE(GET_STR_FROM_EEPROM("pF >> "), data);
  // Serial.println(var_toDouble(&pF.value) * pow(10, pF.scaler));
  datameter.pf = var_toDouble(&pF.value) * pow(10, pF.scaler);
  Serial.printf("pF : %.3f\n", datameter.pf);
  obj_clear(BASE(pF));
  free(data);
  blinkProcces(1);

  gxRegister freq;
  cosem_init(BASE(freq), DLMS_OBJECT_TYPE_REGISTER, "1.0.14.7.0.255");
  com_read(BASE(freq), 3);
  com_read(BASE(freq), 2);
  obj_toString(BASE(freq), &data);
  // GXTRACE(GET_STR_FROM_EEPROM("freq >> "), data);
  // Serial.println(var_toDouble(&freq.value) * pow(10, freq.scaler));
  datameter.freq = var_toDouble(&freq.value) * pow(10, freq.scaler);
  Serial.printf("Freq : %.3f\n", datameter.freq);
  obj_clear(BASE(freq));
  free(data);
  blinkProcces(1);

  gxRegister kwh1;
  cosem_init(BASE(kwh1), DLMS_OBJECT_TYPE_REGISTER, "1.0.1.8.0.255");
  com_read(BASE(kwh1), 3);
  com_read(BASE(kwh1), 2);
  obj_toString(BASE(kwh1), &data);
  // GXTRACE(GET_STR_FROM_EEPROM("kwh1 >> "), data);
  datameter.kwh1 = var_toInteger(&kwh1.value);
  Serial.printf("KWH1 : %d\n", datameter.kwh1);
  obj_clear(BASE(kwh1));
  free(data);
  blinkProcces(1);

  gxRegister kwh2;
  cosem_init(BASE(kwh2), DLMS_OBJECT_TYPE_REGISTER, "1.0.2.8.0.255");
  com_read(BASE(kwh2), 3);
  com_read(BASE(kwh2), 2);
  obj_toString(BASE(kwh2), &data);
  // GXTRACE(GET_STR_FROM_EEPROM("kwh2 >> "), data);
  datameter.kwh2 = var_toInteger(&kwh2.value);
  Serial.printf("KWH2 : %d\n", datameter.kwh2);
  obj_clear(BASE(kwh2));
  free(data);
  blinkProcces(1);

  gxRegister kvar1;
  cosem_init(BASE(kvar1), DLMS_OBJECT_TYPE_REGISTER, "1.0.3.8.0.255");
  com_read(BASE(kvar1), 3);
  com_read(BASE(kvar1), 2);
  obj_toString(BASE(kvar1), &data);
  // GXTRACE(GET_STR_FROM_EEPROM("kvar1 >> "), data);
  datameter.kvar = var_toInteger(&kvar1.value);
  Serial.printf("KVAR1 : %d\n", datameter.kvar);
  obj_clear(BASE(kvar1));
  free(data);
  blinkProcces(1);

  gxRegister kvar2;
  cosem_init(BASE(kvar2), DLMS_OBJECT_TYPE_REGISTER, "1.0.4.8.0.255");
  com_read(BASE(kvar2), 3);
  com_read(BASE(kvar2), 2);
  obj_toString(BASE(kvar2), &data);
  // GXTRACE(GET_STR_FROM_EEPROM("kvar2 >> "), data);
  datameter.kvar2 = var_toInteger(&kvar2.value);
  Serial.printf("KVAR2 : %d\n", datameter.kvar2);
  obj_clear(BASE(kvar2));
  free(data);
  blinkProcces(1);

  // Release dynamically allocated objects.
  Client.ReleaseObjects();
  com_close();
  return ret;
}

void BackgroundDelay()
{
  kwhReadReady = true;
  Serial.printf("TIME TO READ\n");
}

void setup()
{
  Serial.begin(115200);
  pinMode(PIN_LED_WARN, OUTPUT);
  pinMode(PIN_LED_PROC, OUTPUT);
  preferences.begin("settings", false);
  fcnt = preferences.getUInt("lora_fcnt", 0);
  Serial.printf("Frame Counter: %d\n", fcnt);
  GXTRACE(GET_STR_FROM_EEPROM("Start application"), NULL);
  BYTE_BUFFER_INIT(&frameData);
  // Set frame capacity.
  bb_capacity(&frameData, 128);
  Client.init(true, 4, 163, DLMS_AUTHENTICATION_HIGH_GMAC, NULL, DLMS_INTERFACE_TYPE_HDLC);

  Client.SetSecurity(DLMS_SECURITY_AUTHENTICATION_ENCRYPTION);
  gxByteBuffer bb;

  bb_Init(&bb);
  bb_addHexString(&bb, "41 42 43 44 45 46 47 48");
  Client.SetSystemTitle(&bb);
  bb_clear(&bb);
  bb_addHexString(&bb, "46 46 46 46 46 46 46 46 46 46 46 46 46 46 46 46");
  Client.SetAuthenticationKey(&bb);
  bb_clear(&bb);
  bb_addHexString(&bb, "46 46 46 46 46 46 46 46 46 46 46 46 46 46 46 46");
  Client.SetBlockCipherKey(&bb);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  if (!lora.init())
  {
    Serial.println("LORA not detected");
  }
  else
    loraConf();
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  com_readStaticOnce();
  setTime(datameter.timeUnix);
  Alarm.timerRepeat(60, BackgroundDelay); // timer for every 15 seconds
  customerID = devAddr;
  deviceNumber = strtoul(customerID.c_str(), NULL, 16);
}

void loop()
{
  // com_readStaticOnce();
  int ret_readAll;
  int ret_com;
  char bufferData[255];
  blinkWarning(1);
  if (kwhReadReady)
  {
    blinkWarning(0);
    runTime = millis();
    GXTRACE(GET_STR_FROM_EEPROM("Start reading"), NULL);
    ret_readAll = com_readAllObjects();
    Serial.printf("com_readAllObjects() : %d -- ", ret_readAll);
    ret_com = com_close();
    Serial.printf("com_close() : %d\n", ret_com);
    if (ret_readAll == DLMS_ERROR_CODE_OK and ret_com == DLMS_ERROR_CODE_OK)
    {
      // sprintf(bufferData, "TimeUnix: %d\nSerial Number: %s\nVolt: %.3f\nCurrent: %.3f\nWatt: %.3f\npF: %.3f\nfreq: %.2f\nkvar1: %d\nkvar2: %d\nkwh1: %d\nkhw2: %d\n",
      //         datameter.timeUnix,
      //         datameter.serialnumber,
      //         datameter.volt,
      //         datameter.current,
      //         datameter.watt,
      //         datameter.pf,
      //         datameter.freq,
      //         datameter.kvar,
      //         datameter.kvar2,
      //         datameter.kwh1,
      //         datameter.kwh2);
      // Serial.print(bufferData);
      blinkProcces(5);
      sprintf(bufferData, "~*%d*%d*%s*%.3f*%d*%d*%.3f*%d*%d*%.3f*%d*%d*%.3f*%.3f*%d*%d*%d*%d#",
              1,
              deviceNumber,
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
      Serial.println(bufferData);
      lora.sendUplink(bufferData, strlen(bufferData), 1, 1);
      lora.update();
      preferences.putUInt("lora_fcnt", lora.getFrameCounter());
    }

    fcnt = preferences.getUInt("lora_fcnt", 0);
    Serial.printf("Frame Counter: %d\n", fcnt);
    kwhReadReady = false;
  }
  digitalClockDisplay();
  Alarm.delay(1000);
}

#ifdef DLMS_DEBUG
// Print detailed information to the serial port.
void svr_trace(const char *str, const char *data)
{
  GXTRACE(str, data);
}
#endif // DLMS_DEBUG
