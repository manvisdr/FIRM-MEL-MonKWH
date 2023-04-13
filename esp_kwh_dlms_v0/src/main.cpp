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
#include <EthernetSPI2.h>
#include <SettingsManager.h>

#include <TimeLib.h>
#include <TimeAlarms.h>
#include <timeout.h>

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>

#define FIRMWARE_VERSION 0.0
#define USE_WIFI

#ifdef USE_WIFI
WiFiClient wificlient;
PubSubClient mqtt(wificlient);
#else
EthernetClient ethclient;
PubSubClient mqtt(ethclient);
#endif

IPAddress mqttServer(203, 194, 112, 238);
Preferences PREFS;
SettingsManager settings;

uint32_t readKWH();

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

struct dataCurrentMeter
{
  String serialnumber;
  int contactState;
  int32_t currentStan;
  int32_t currentBalance;
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

struct ethernetConfig
{
  bool intStatus;
  IPAddress ipaddr;
  IPAddress ipsubnet;
  IPAddress ipgateway;
  IPAddress ipdns;
  byte mac[4];
};
ethernetConfig ethConf;

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
char topicEthernetConfigRequest[50];
char topicMeter[50];

uint32_t unixTime;
ulong deviceNumber;
char buffData[255];
char idDevice[] = "1000";
int32_t initStan;
uint32_t kwhStanPrev;
uint32_t kwhStanNow;
int balanceNow;
int kwhUnit = 1000;
int PREFSstan;
int PREFSbalance;
int PREFSCredit;

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
#define PIN_ETHERNET_CS 4
#define PIN_CONTACT 15
#define PIN_LED_LINK_KWH PIN_LED_PROC
#define PIN_LED_ACTIVE_KWH PIN_LED_WARN

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

String mac2String(byte ar[])
{
  String s;
  for (byte i = 0; i < 6; ++i)
  {
    char buf[3];
    sprintf(buf, "%2X", ar[i]);
    s += buf;
    if (i < 5)
      s += ':';
  }
  return s;
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

int loadEthConfig(/* char *payload, */ ethernetConfig &config)
{
  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, received_payload);
  if (error)
  {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    received_msg = false;
    return -1;
  }
  JsonObject ethernet = doc["ethernet"];
  const char *ethernet_ipaddress = ethernet["ipaddress"]; // "192.168.2.2"
  const char *ethernet_ipgateway = ethernet["ipgateway"]; // "192.168.2.2"
  const char *ethernet_ipsubnet = ethernet["ipsubnet"];   // "192.168.2.2"
  const char *ethernet_ipdns = ethernet["ipdns"];         // "8.8.8.8"

  if (config.ipaddr.fromString(ethernet_ipaddress))
  {                                                             // try to parse into the IPAddress
    Serial.printf("IPADRRESS: %s\n", config.ipaddr.toString()); // print the parsed IPAddress
  }

  if (config.ipsubnet.fromString(ethernet_ipsubnet))
  {                                                              // try to parse into the IPAddress
    Serial.printf("IPSUBNET: %s\n", config.ipsubnet.toString()); // print the parsed IPAddress
  }

  if (config.ipgateway.fromString(ethernet_ipgateway))
  {                                                                // try to parse into the IPAddress
    Serial.printf("IPGATEWAY: %s\n", config.ipgateway.toString()); // print the parsed IPAddress
  }

  if (config.ipdns.fromString(ethernet_ipdns))
  {                                                        // try to parse into the IPAddress
    Serial.printf("IPDNS: %s\n", config.ipdns.toString()); // print the parsed IPAddress
  }
  return 1;
}

void ETHinit()
{
  esp_read_mac(ethConf.mac, ESP_MAC_ETH);
  Ethernet.init(PIN_ETHERNET_CS);
  if (Ethernet.begin(ethConf.mac) == 0)
  {
    Serial.println("Failed Ethernet using DHCP");
    if (Ethernet.hardwareStatus() == EthernetNoHardware)
    {
      Serial.println("Ethernet shield ERROR");
      while (true)
      {
        delay(1); // do nothing, no point running without Ethernet hardware
      }
    }
    if (Ethernet.linkStatus() == LinkOFF)
    {
      Serial.println("Ethernet cable is not connected.");
    }
    // try to configure using IP address instead of DHCP:
    Ethernet.begin(ethConf.mac, ethConf.ipaddr, ethConf.ipdns, ethConf.ipgateway, ethConf.ipsubnet);
  }
  else
  {
    Serial.print("  DHCP assigned IP ");
    Serial.println(Ethernet.localIP());
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
int com_read(gxObject *object, unsigned char attributeOrdinal);
int com_write(gxObject *object, unsigned char attributeOrdinal);
int com_method(gxObject *object, unsigned char attributeOrdinal, dlmsVARIANT *params);

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

  // // Read clock
  // gxClock clock1;
  // cosem_init(BASE(clock1), DLMS_OBJECT_TYPE_CLOCK, "0.0.1.0.0.255");
  // com_read(BASE(clock1), 3);
  // com_read(BASE(clock1), 2);
  // currentDataMeter.timeUnix = time_toUnixTime2(&clock1.time);
  // obj_toString(BASE(clock1), &data);
  // obj_clear(BASE(clock1));
  // free(data);

  gxData sn;
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
  Serial.println();
  char *prtst = result;
  for (byte i = 0; i < 12; i++)
  {
    output[i] = aNibble(*prtst++);
    output[i] <<= 4;
    output[i] |= aNibble(*prtst++);
  }
  currentDataMeter.serialnumber = output;

  free(pStr);
  obj_clear(BASE(sn));
  free(data);
  GuruxClient.ReleaseObjects();
  // com_close();
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

  return 1;
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

void KWHReadingInit()
{
  PREFS.begin("meter", false);
  PREFSstan = PREFS.getUInt("stan", 0);
  PREFSbalance = PREFS.getUInt("balance", 0);
  PREFSCredit = PREFS.getUInt("credit", 0);
  Serial.printf("MEMORY DATA -> stan: %d balance: %d credit: %d\n", PREFSstan, PREFSbalance, PREFSCredit);
  com_readStaticOnce();
  Serial.printf("%s\n", currentDataMeter.serialnumber);
  currentDataMeter.currentCredit = PREFSCredit;
  initStan = readKWH();
  if (PREFSstan == 0)
  {
    PREFSstan = initStan;
    PREFS.putUInt("stan", PREFSstan);
  }
  kwhStanNow = initStan;
  balanceNow = PREFSbalance;
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
    // String clientId = "MonKWHControl_" + String(idDevice);
    String clientId = "MonKWHControl-";
    clientId += String(random(0xffff), HEX);
    if (mqtt.connect(clientId.c_str()))
    {
      Serial.println("connected");
      mqtt.subscribe(topicEthernetConfigRequest);
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

uint32_t readKWH()
{
  int ret;
  ret = com_initializeConnection();
  if (ret != DLMS_ERROR_CODE_OK)
  {
    GXTRACE_INT(GET_STR_FROM_EEPROM("com_initializeConnection failed"), ret);
    return ret;
  }
  blinkLINKOn();
  gxRegister objKwhTotal;
  cosem_init(BASE(objKwhTotal), DLMS_OBJECT_TYPE_REGISTER, "1.0.2.8.0.255");
  com_read(BASE(objKwhTotal), 3);
  com_read(BASE(objKwhTotal), 2);
  uint32_t stanNow = var_toInteger(&objKwhTotal.value);
  obj_clear(BASE(objKwhTotal));

  blinkLINKOff();
  return stanNow;
}

void JsonInit()
{
  deviceDoc["id"] = idDevice;
  deviceDoc["state"] = 1;
  deviceDoc["firmversion"] = String(FIRMWARE_VERSION, 1);

  JsonObject ethernet = deviceDoc.createNestedObject("ethernet");
  const char *ethernet_ipaddress = ethernet["ipaddress"]; // "192.168.2.2"
  const char *ethernet_ipgateway = ethernet["ipgateway"]; // "192.168.2.2"
  const char *ethernet_ipsubnet = ethernet["ipsubnet"];   // "192.168.2.2"
  const char *ethernet_ipdns = ethernet["ipdns"];         // "8.8.8.8"

  ethernet["ipaddress"] = "0.0.0.0";
  ethernet["ipgateway"] = "0.0.0.0";
  ethernet["ipsubnet"] = "0.0.0.0";
  ethernet["ipdns"] = "0.0.0.0";

  deviceDoc["internet"] = true;
#ifdef USE_WIFI
  esp_read_mac(ethConf.mac, ESP_MAC_WIFI_STA);
#else
  esp_read_mac(ethConf.mac, ESP_MAC_ETH);
#endif
  deviceDoc["mac"] = mac2String(ethConf.mac);
}

void JsonDevice(int state)
{
  deviceDoc["state"] = 1;
  deviceDoc["internet"] = true;
}

void JsonKWH()
{
  meterDoc["id"] = currentDataMeter.serialnumber;
  meterDoc["status"] = currentDataMeter.contactState;
  meterDoc["stanNow"] = currentDataMeter.currentStan;
  meterDoc["balance"] = currentDataMeter.currentBalance;
  meterDoc["credit"] = currentDataMeter.currentCredit;
}

void JsonTransaction(char *status, char *credit)
{
  transactionDocResp["state"] = status;
  transactionDocResp["credit"] = credit;
}

void sendMqttBuffer(char *topic, char *buffer)
{
  mqtt.publish(topic, buffer);
}

void ProcessLooping()
{
  kwhStanPrev = kwhStanNow;
  Serial.printf("kwhprev: %d ", kwhStanPrev);
  kwhStanNow = readKWH();
  Serial.printf("kwhnow: %d ", kwhStanNow);
  currentDataMeter.currentStan = kwhStanNow;

  if (kwhStanNow != kwhStanPrev)
  {
    balanceNow = balanceNow - (kwhStanNow - kwhStanPrev);
    PREFS.putUInt("balance", balanceNow);
    PREFS.putUInt("stan", kwhStanNow);
  }
  Serial.printf("balance: %d\n", balanceNow);
  currentDataMeter.currentBalance = balanceNow;
  if (currentDataMeter.currentBalance <= 0)
  {
    currentDataMeter.contactState = 0;
    // digitalWrite(PIN_CONTACT, LOW);
  }
  else
  {
    currentDataMeter.contactState = 1;
    // digitalWrite(PIN_CONTACT, HIGH);
  }
}

void GetDevice()
{
  if (received_msg and strcmp(received_topic, topicDevice) == 0 and strcmp(received_payload, "get") == 0)
  {
    serializeJson(deviceDoc, Serial);
    received_msg = false;
  }
}

void saveEthConfigMqtt()
{
  if (received_msg and strcmp(received_topic, topicEthernetConfigRequest) == 0)
  {
    loadEthConfig(ethConf);
    received_msg = false;
  }
}

int TopupProcess()
{
  if (received_msg and strcmp(received_topic, topicTransactionReqs) == 0)
  {
    Serial.printf("%s\n", received_payload);
    DeserializationError error = deserializeJson(transactionDocReqs, received_payload);
    if (error)
    {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
      received_msg = false;
      return -1;
    }

    const char *state = transactionDocReqs["state"]; // "topup"
    int credit = transactionDocReqs["credit"];
    currentDataMeter.currentCredit = credit;
    // 2312312312312312300
    Serial.println(state);
    Serial.println(credit);

    if (strcmp(state, "topup") == 0)
    {
      Serial.printf("TOPUP COMMAND\nTOKEN: %d\n", credit);

      balanceNow = balanceNow + credit;
      char buffer[256];
      PREFS.putUInt("credit", credit);
      PREFS.putUInt("balance", balanceNow);
      received_msg = false;
      return 1;
    }
    received_msg = false;
  }
  else
    return 1;
  return 1;
}

void updateData()
{
  char buffer[100];
  // com_readCurrentDataMeter();
  JsonDevice(1);
  JsonKWH();

#ifdef USE_WIFI
  deviceDoc["ethernet"]["ipaddress"] = WiFi.localIP().toString();
  deviceDoc["ethernet"]["ipgateway"] = WiFi.gatewayIP().toString();
  deviceDoc["ethernet"]["ipsubnet"] = WiFi.subnetMask().toString();
  deviceDoc["ethernet"]["ipdns"] = WiFi.dnsIP().toString();
#endif

  blinkACTIVEOn();
  mqtt.beginPublish(topicDevice, measureJson(deviceDoc), false);
  BufferingPrint bufferedClient(mqtt, 32);
  serializeJson(deviceDoc, bufferedClient);
  bufferedClient.flush();
  mqtt.endPublish();

  serializeJson(meterDoc, buffer);
  mqtt.publish(topicMeter, buffer);
  blinkACTIVEOff();
}

void setup()
{
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  pinMode(PIN_CONTACT, OUTPUT);
  pinMode(PIN_LED_LINK_KWH, OUTPUT);
  pinMode(PIN_LED_ACTIVE_KWH, OUTPUT);

  WiFi.disconnect();
  WiFi.mode(WIFI_MODE_STA);
  // WiFi.begin("MGI-MNC", "#neurixmnc#");
  WiFi.begin("lepi", "1234567890");
  // WiFi.begin("WiFi kost", "rumahkos");

  mqtt.setServer(mqttServer, 1883);
  mqtt.setCallback(MqttCallback);
  MqttTopicInit();
  // JsonInit();

  // KWHDLMS_init();
  // KWHReadingInit();

  // Alarm.timerRepeat(15, updateData);
}

long mil = 0;

void loop()
{
  if (!mqtt.connected())
  {
    MqttReconnect();
  }

  // saveEthConfigMqtt();
  // ProcessLooping();
  // TopupProcess();

  mqtt.loop();
  // Alarm.delay(0);
}
