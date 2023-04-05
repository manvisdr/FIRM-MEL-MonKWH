#include <Arduino.h>
#include <EEPROM.h>
#include "GXDLMSClient.h"
#include <SPI.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <StreamUtils.h>

#include <TimeLib.h>
#include <TimeAlarms.h>
#include <timeout.h>

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>

#define FIRMWARE_VERSION 0.0

WiFiClient wificlient;
PubSubClient mqtt(wificlient);
IPAddress mqttServer(203, 194, 112, 238);
Preferences preferences;

StaticJsonDocument<256> deviceDoc;
StaticJsonDocument<256> meterDoc;
StaticJsonDocument<256> transactionDocResp;
StaticJsonDocument<256> transactionDocReqs;

enum class StatusDevice : uint8_t
{
  NONE,
  IDLE,
  WORKING
};

StatusDevice devStatus = StatusDevice::NONE;

enum class StatusTransaction : uint8_t
{
  NONE,
  CHECK,
  TOPUP,
  TESTING
};

StatusTransaction transStatus = StatusTransaction::NONE;

struct dataCreditMeter
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

dataCreditMeter datameter;

struct dataCurrentMeter
{
  String serialnumber;
  uint32_t timeUnix;
  int contactState;
  int currentStan;
  int32_t currentCredit;
};

dataCurrentMeter currentDataMeter;

struct tokenStruct
{
  char tokenState[50];
  char tokenToken[25];
  char tokenTime[50];
};

tokenStruct tokenMeterResp;

const std::map<DLMS_CONTROL_STATE, std::string> CONTACT_STATUS_MAP = {
    {DLMS_CONTROL_STATE::DLMS_CONTROL_STATE_CONNECTED, "CONTACT_CONNECTED"},
    {DLMS_CONTROL_STATE::DLMS_CONTROL_STATE_DISCONNECTED, "CONTACT_DISCONNECTED"},
    {DLMS_CONTROL_STATE::DLMS_CONTROL_STATE_READY_FOR_RECONNECTION, "CONTACT_READYFOR_RECCONNECTION"},
};

const std::map<DLMS_TOKEN_DELIVERY, std::string> TOKEN_DELIVERY_MAP = {
    {DLMS_TOKEN_DELIVERY::DLMS_TOKEN_DELIVERY_LOCAL, "DELIVERY_LOCAL"},
    {DLMS_TOKEN_DELIVERY::DLMS_TOKEN_DELIVERY_MANUAL, "DELIVERY_MANUAL"},
    {DLMS_TOKEN_DELIVERY::DLMS_TOKEN_DELIVERY_REMOTE, "DELIVERY_REMOTE"},
};

const std::map<DLMS_TOKEN_STATUS_CODE, std::string> TOKEN_STATUS_MAP = {
    {DLMS_TOKEN_STATUS_CODE::FORMAT_OK, "STATUS_FORMAT_OK"},
    {DLMS_TOKEN_STATUS_CODE::AUTHENTICATION_OK, "STATUS_AUTHENTICATION_OK"},
    {DLMS_TOKEN_STATUS_CODE::VALIDATION_OK, "STATUS_VALIDATION_OK"},
    {DLMS_TOKEN_STATUS_CODE::TOKEN_EXECUTION_OK, "STATUS_TOKEN_EXECUTION_OK"},
    {DLMS_TOKEN_STATUS_CODE::DLMS_TOKEN_STATUS_CODE_TOKEN_FORMAT_FAILURE, "STATUS_TOKEN_FORMAT_FAILURE"},
    {DLMS_TOKEN_STATUS_CODE::DLMS_TOKEN_STATUS_CODE_AUTHENTICATION_FAILURE, "STATUS_AUTHENTICATION_FAILURE"},
    {DLMS_TOKEN_STATUS_CODE::DLMS_TOKEN_STATUS_CODE_VALIDATION_RESULT_FAILURE, "STATUS_VALIDATION_RESULT_FAILURE"},
    {DLMS_TOKEN_STATUS_CODE::DLMS_TOKEN_STATUS_CODE_TOKEN_EXECUTION_RESULT_FAILURE, "STATUS_TOKEN_EXECUTION_RESULT_FAILURE"},
    {DLMS_TOKEN_STATUS_CODE::DLMS_TOKEN_STATUS_CODE_TOKEN_RECEIVED, "STATUS_TOKEN_RECEIVED"},
};

char topicDevice[50];
char topicTransactionReqs[50];
char topicTransactionResp[50];
char topicMeter[50];
uint32_t unixTime;
ulong deviceNumber;
char buffData[255];
char idDevice[] = "0000";
unsigned char tokenByteArr[20];

// MQTT BUFFER INIT
const int payloadSize = 100;
const int topicSize = 128;
char received_topic[topicSize];
char received_payload[payloadSize];
unsigned int received_length;
bool received_msg = false;
// MQTT BUFFER INIT

// DEF PIN
#define RXD2 16         // PIN RX2
#define TXD2 17         // PIN TX2
#define PIN_LED_PROC 27 // PIN RX2
#define PIN_LED_WARN 15 // PIN TX2

String customerID;
String kwhID;

gxByteBuffer frameData;
const uint32_t WAIT_TIME = 2000;
const uint8_t RESEND_COUNT = 3;
uint32_t runTime = 0;
bool kwhReadReady = false;
unsigned int fcnt;

DLMS_TOKEN_STATUS_CODE TOKEN_STATUS_TO_IN(unsigned int D)
{
  return static_cast<DLMS_TOKEN_STATUS_CODE>(D);
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

std::string MapContactStatus(DLMS_CONTROL_STATE contactState)
{
  auto it = CONTACT_STATUS_MAP.find(contactState);
  if (it == CONTACT_STATUS_MAP.end())
    return "Unknown State";

  return it->second;
}

std::string MapTokenStatus(DLMS_TOKEN_STATUS_CODE tokenState)
{
  auto it = TOKEN_STATUS_MAP.find(tokenState);
  if (it == TOKEN_STATUS_MAP.end())
    return "Unknown State";
  return it->second;
}

int charArrayToByteArray(char *charArray, unsigned char *outByte)
{
  for (int i = 0; i < 20; i++)
  {
    char char_digit = charArray[i];
    char string_for_atoi[2] = {char_digit, '\0'};
    int number = atoi(string_for_atoi);
    outByte[i] = number;
  }
  return 0;
}

int com_readTokenPrev()
{
  int ret;
  char *tokenRead;
  char *timeRead;
  char *descRead;
  int statusRead;
  // Initialize connection.
  ret = com_initializeConnection();
  if (ret != DLMS_ERROR_CODE_OK)
  {
    GXTRACE_INT(GET_STR_FROM_EEPROM("com_initializeConnection failed"), ret);
    return ret;
  }
  gxByteBuffer bb;
  gxByteBuffer tokenBuff;
  gxByteBuffer tokenTimeBuff;
  gxTokenGateway objToken;

  BYTE_BUFFER_INIT(&bb);

  dlmsVARIANT params;
  var_init(&params);
  var_setInt8(&params, 0);
  var_addOctetString(&params, &bb);
  cosem_init(BASE(objToken), DLMS_OBJECT_TYPE_TOKEN_GATEWAY, "0.0.19.40.0.255");
  com_read(BASE(objToken), 3);
  com_read(BASE(objToken), 2);
  com_read(BASE(objToken), 1);

  tokenRead = bb_toHexString(&objToken.token);
  Serial.println(" ");
  GXTRACE(PSTR("Token : "), tokenRead);

  time_toString(&objToken.time, &tokenTimeBuff);
  timeRead = bb_toString(&tokenTimeBuff);
  Serial.println(" ");
  GXTRACE(PSTR("TokenTime : "), timeRead);

  Serial.printf("\nenum Value : %d\n", (int)&objToken.status);
  // GXTRACE(PSTR("TokenTime : "), statusRead);
  // &objToken.status;
  // time_toString(&objToken.time, &tokenTimeBuff);
  // timeRead = bb_toString(&tokenTimeBuff);
  // Serial.println(" ");
  // GXTRACE(PSTR("TokenTime : "), timeRead);

  // &objToken.descriptions;
  arr_getByIndex(&objToken.descriptions, 5, (void **)descRead);
  GXTRACE(PSTR("TokenDescription : "), descRead);

  obj_clear(BASE(objToken));
  free(tokenRead);
  free(timeRead);
  GuruxClient.ReleaseObjects();
  com_close();
  return 1;
}

int com_writeToken(unsigned char *token)
{
  const size_t lenToken = 20;
  char *data;
  char *responseTokenTokens;
  char *responseTokenTime;
  int responseTokenStatus;
  std::string tokenStateStr;
  gxTokenGateway objToken;

  // unsigned char token[20] = {0x31, 0x37, 0x36, 0x32, 0x32, 0x34, 0x34, 0x39, 0x37, 0x30, 0x36, 0x31, 0x31, 0x35, 0x30, 0x34, 0x39, 0x32, 0x35, 0x30};
  gxByteBuffer bb;
  int ret;
  ret = com_initializeConnection();
  if (ret != DLMS_ERROR_CODE_OK)
  {
    GXTRACE_INT(GET_STR_FROM_EEPROM("com_initializeConnection failed"), ret);
    return ret;
  }
  BYTE_BUFFER_INIT(&bb);
  if ((ret = bb_setUInt8(&bb, DLMS_DATA_TYPE_OCTET_STRING)) == 0 &&
      (ret = hlp_setObjectCount(lenToken, &bb)) == 0 &&
      (ret = bb_set(&bb, token, lenToken)) == 0)
  {
  }
  dlmsVARIANT params;
  var_init(&params);
  var_setInt8(&params, 0);
  var_addOctetString(&params, &bb);
  var_addBytes(&params, token, lenToken);
  cosem_init(BASE(objToken), DLMS_OBJECT_TYPE_TOKEN_GATEWAY, "0.0.19.40.0.255");
  com_method(BASE(objToken), 1, &params);
  obj_toString(BASE(objToken), &data);
  Serial.println("-------RESPONSE--------");
  // GXTRACE(PSTR("PRINT DATA: "), data);

  auto it = TOKEN_STATUS_MAP.find(objToken.status);
  if (it == TOKEN_STATUS_MAP.end())
    tokenStateStr = "Unknown State";
  else
    tokenStateStr = it->second;
  // Serial.printf("tokenState : %s\n", tokenStateStr.c_str());

  strcpy(tokenMeterResp.tokenState, tokenStateStr.c_str());
  // tokenMeterResp.tokenTime = time_toUnixTime2(&objToken.time);

  char buff[50];
  time_toString2(&objToken.time, tokenMeterResp.tokenTime, 50);

  obj_clear(BASE(objToken));
  free(data);
  free(responseTokenTokens);
  free(responseTokenTime);
  Serial.println("-------RESPONSE--------");
  return 1;
}

int com_disconnectRelay()
{
  char *data;
  gxDisconnectControl dc;
  dlmsVARIANT params;
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
  var_init(&params);
  var_setInt8(&params, 0);
  cosem_init(BASE(dc), DLMS_OBJECT_TYPE_DISCONNECT_CONTROL, "0.0.96.3.10.255");
  com_method(BASE(dc), 1, &params);
  obj_toString(BASE(dc), &data);

  GXTRACE(PSTR("Disconnect : "), data);
  obj_clear(BASE(dc));
  free(data);
  return 1;
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
  // Read just wanted objects withour serializating the association view.
  char *data = NULL;
  gxByteBuffer snBuff;
  bb_Init(&snBuff);

  // Read clock
  gxClock clock1;
  cosem_init(BASE(clock1), DLMS_OBJECT_TYPE_CLOCK, "0.0.1.0.0.255");
  com_read(BASE(clock1), 3);
  com_read(BASE(clock1), 2);
  currentDataMeter.timeUnix = time_toUnixTime2(&clock1.time);
  // datameter.timeUnix = time_toUnixTime2(&clock1.time);

  obj_toString(BASE(clock1), &data);
  // GXTRACE(GET_STR_FROM_EEPROM("Clock"), data);
  Serial.println(datameter.timeUnix);
  obj_clear(BASE(clock1));
  free(data);

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
  // Serial.println(F("----------------------------"));

  while (token != NULL)
  {
    // Serial.print(token);
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
  currentDataMeter.serialnumber = output;
  // datameter.serialnumber = output;
  // Serial.println(datameter.serialnumber);
  // Serial.println(F("----------------------------"));

  free(pStr);
  obj_clear(BASE(sn));
  free(data);
  GuruxClient.ReleaseObjects();
  com_close();

  return ret;
}

int32_t com_readCredit()
{
  int ret;

  ret = com_initializeConnection();
  if (ret != DLMS_ERROR_CODE_OK)
  {
    GXTRACE_INT(GET_STR_FROM_EEPROM("com_initializeConnection failed"), ret);
    return ret;
  }

  gxCredit objCredit;
  cosem_init(BASE(objCredit), DLMS_OBJECT_TYPE_CREDIT, "0.0.19.10.0.255");
  com_read(BASE(objCredit), 3);
  com_read(BASE(objCredit), 2);
  // obj_toString(BASE(objCredit), &data);
  // Serial.println(objCredit.currentCreditAmount);
  // Serial.println(objCredit.warningThreshold);
  // Serial.println(objCredit.status);
  // Serial.println(objCredit.creditAvailableThreshold);

  // uint32_t timeCredit = time_toUnixTime2(&objCredit.period);
  // Serial.println(timeCredit);
  // obj_clear(objCredit);
  // free(data);
  return objCredit.currentCreditAmount;
}

int com_readCurrentDataMeter()
{
  int ret;
  ret = com_initializeConnection();
  if (ret != DLMS_ERROR_CODE_OK)
  {
    GXTRACE_INT(GET_STR_FROM_EEPROM("com_initializeConnection failed"), ret);
    return ret;
  }
  gxDisconnectControl objContact;
  gxCredit objCredit;
  gxRegister objKWHExport;
  DLMS_CONTROL_STATE contactState;
  std::string contactStateStr;
  cosem_init(BASE(objContact), DLMS_OBJECT_TYPE_DISCONNECT_CONTROL, "0.0.96.3.10.255");
  com_read(BASE(objContact), 1);
  // obj_toString(BASE(objContact), &data);
  contactState = objContact.controlState;
  auto it = CONTACT_STATUS_MAP.find(contactState);
  if (it == CONTACT_STATUS_MAP.end())
    contactStateStr = "Unknown State";
  else
    contactStateStr = it->second;
  Serial.printf("currentContactState : %s\n", contactStateStr.c_str());
  obj_clear(BASE(objContact));

  cosem_init(BASE(objCredit), DLMS_OBJECT_TYPE_CREDIT, "0.0.19.10.0.255");
  com_read(BASE(objCredit), 3);
  com_read(BASE(objCredit), 2);
  currentDataMeter.currentCredit = objCredit.currentCreditAmount;
  Serial.printf("currentCredit : %d\n", currentDataMeter.currentCredit);
  obj_clear(BASE(objCredit));

  cosem_init(BASE(objKWHExport), DLMS_OBJECT_TYPE_REGISTER, "1.0.2.8.0.255");
  com_read(BASE(objKWHExport), 3);
  com_read(BASE(objKWHExport), 2);
  // GXTRACE(GET_STR_FROM_EEPROM("kwh2 >> "), data);
  currentDataMeter.currentStan = var_toInteger(&objKWHExport.value);
  Serial.printf("currentStan : %d\n", currentDataMeter.currentStan);
  obj_clear(BASE(objKWHExport));

  // Serial.println(objCredit.currentCreditAmount);
  // Serial.println(objCredit.warningThreshold);
  // Serial.println(objCredit.status);
  // Serial.println(objCredit.creditAvailableThreshold);

  // uint32_t timeCredit = time_toUnixTime2(&objCredit.period);
  // Serial.println(timeCredit);
  // obj_clear(objCredit);
  // free(data);
  return 1;
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

  // Release dynamically allocated objects.
  GuruxClient.ReleaseObjects();
  com_close();
  return ret;
}

void KWHDLMS_init()
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

void BackgroundDelay()
{
  kwhReadReady = true;
  Serial.printf("TIME TO READ\n");
}

void MqttCallback(char *topic, byte *payload, unsigned int length)
{
  memset(received_payload, '\0', payloadSize); // clear payload char buffer
  memset(received_topic, '\0', topicSize);
  int i = 0; // extract payload
  for (i; i < length; i++)
  {
    received_payload[i] = ((char)payload[i]);
  }
  received_payload[i] = '\0';
  strcpy(received_topic, topic);
  received_msg = true;
  received_length = length;

  // Serial.println("---------------MQTT------------------");
  // Serial.printf("received_topic %s received_length = %d \n", received_topic, received_length);
  // Serial.printf("received_payload %s \n", received_payload);
  // Serial.println("---------------MQTT------------------");
}

void MqttTopicInit()
{
  sprintf(topicDevice, "%s/%s/%s", "KWHControl", idDevice, "device");
  sprintf(topicTransactionReqs, "%s/%s/%s", "KWHControl", idDevice, "transaction/request");
  sprintf(topicTransactionResp, "%s/%s/%s", "KWHControl", idDevice, "transaction/response");
  sprintf(topicMeter, "%s/%s/%s", "KWHControl", idDevice, "meter");
}

void MqttReconnect()
{
  while (!mqtt.connected() and WiFi.status() == WL_CONNECTED)
  {
    Serial.print("Attempting MQTT connection...");
    String clientId = "MonKWHControl_" + String(idDevice);
    if (mqtt.connect(clientId.c_str()))
    {
      Serial.println("connected");
      // mqtt.subscribe(topicDevice);
      mqtt.subscribe(topicTransactionReqs);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" try again in 5 seconds");
    }
  }
}

void JsonInit()
{
  deviceDoc["id"] = idDevice;
  deviceDoc["state"] = 1;
  deviceDoc["firmversion"] = String(FIRMWARE_VERSION, 1);
  deviceDoc["internet"] = true;
  deviceDoc["mac"] = WiFi.macAddress();
}

void JsonDevice(int state)
{
  deviceDoc["state"] = 1;
  deviceDoc["internet"] = true;
}

void JsonKWH(char *val1)
{
  // meterDoc["type"] = 1;
  meterDoc["id"] = currentDataMeter.serialnumber;
  meterDoc["status"] = currentDataMeter.contactState;
  meterDoc["stanNow"] = currentDataMeter.currentStan;
  // meterDoc["stanPrev"] = val1;
  meterDoc["balance"] = currentDataMeter.currentCredit;
}

void JsonTransaction(char *status, char *token, char *time)
{
  // Serial.println(status);
  // Serial.println(token);
  transactionDocResp["state"] = status;
  transactionDocResp["token"] = token;
  transactionDocResp["time"] = time;
}

void sendMqttBuffer(char *topic, char *buffer)
{
  mqtt.publish(topic, buffer);
}

void GetDevice()
{
  if (received_msg and strcmp(received_topic, topicDevice) == 0 and strcmp(received_payload, "get") == 0)
  {
    serializeJson(deviceDoc, Serial);
    received_msg = false;
  }
}

bool validateJson(const char *input)
{
  StaticJsonDocument<0> doc, filter;
  return deserializeJson(doc, input, DeserializationOption::Filter(filter)) == DeserializationError::Ok;
}

void TopupProcess()
{
  if (received_msg and strcmp(received_topic, topicTransactionReqs) == 0)
  {
    DeserializationError error = deserializeJson(transactionDocReqs, received_payload);
    if (error)
    {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
      received_msg = false;
      // return -1;
    }
    // JsonObject root = transactionDocReqs.to<JsonObject>();
    // JsonVariant bufferVariant = root["state"];
    // if (!bufferVariant.isNull())
    // {
    //   Serial.println(bufferVariant.as<const char *>());
    //   received_msg = false;
    //   return -1;
    // }
    const char *state = transactionDocReqs["state"]; // "topup"
    const char *token = transactionDocReqs["token"]; // 2312312312312312300
    // Serial.println(state);
    // Serial.println(token);

    if (strcmp(state, "topup") == 0)
    {
      Serial.printf("TOPUP COMMAND\nTOKEN: %s\n", token);

      char buffer[256];
      charArrayToByteArray((char *)token, tokenByteArr);
      strcpy(tokenMeterResp.tokenToken, token);

      com_writeToken(tokenByteArr);
      JsonTransaction(tokenMeterResp.tokenState, tokenMeterResp.tokenToken, tokenMeterResp.tokenTime);

      serializeJson(transactionDocResp, buffer);
      sendMqttBuffer(topicTransactionResp, buffer);
      received_msg = false;
      // return 0;
    }
    received_msg = false;
    // return 0;
  }
}

void updateData()
{
  char buffer[100];
  com_readCurrentDataMeter();
  JsonDevice(1);
  JsonKWH("500");
  // serializeJson(deviceDoc, buffer);

  mqtt.beginPublish(topicDevice, measureJson(deviceDoc), false);
  BufferingPrint bufferedClient(mqtt, 32);
  serializeJson(deviceDoc, bufferedClient);
  bufferedClient.flush();
  mqtt.endPublish();

  serializeJson(meterDoc, buffer);
  mqtt.publish(topicMeter, buffer);
}

void setup()
{
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  pinMode(PIN_LED_WARN, OUTPUT);
  pinMode(PIN_LED_PROC, OUTPUT);

  // WiFi.disconnect();
  WiFi.mode(WIFI_MODE_STA);
  // WiFi.begin("MGI-MNC", "#neurixmnc#");
  WiFi.begin("lepi", "1234567890");
  // WiFi.begin("WiFi kost", "rumahkos");

  mqtt.setServer(mqttServer, 1883);
  mqtt.setCallback(MqttCallback);
  MqttTopicInit();
  JsonInit();

  KWHDLMS_init();
  com_readStaticOnce();
  setTime(currentDataMeter.timeUnix);
  Alarm.timerRepeat(15, updateData /* CheckPingSend */);
}

long mil = 0;

void loop()
{
  if (!mqtt.connected())
  {
    MqttReconnect();
  }

  mqtt.loop();
  GetDevice();
  TopupProcess();
  Alarm.delay(0);
  // if (millis() > mil + 10000)
  // {
  //   // com_disconnectRelay();
  //   // com_readTokenPrev();
  //   // com_readCredit();
  //   // com_readCurrentDataMeter();
  //   // com_writeToken();
  //   mil = millis();
  // }
}
