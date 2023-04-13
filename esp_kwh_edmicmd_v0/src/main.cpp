#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <StreamUtils.h>
#include <EthernetSPI2.h>
#include <TimeLib.h>
#include <TimeAlarms.h>
#include "EDMICmdLine.h"
#include <SettingsManager.h>

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

struct dataCurrentMeter
{
  char serialnumber[20];
  int contactState;
  int32_t currentStan;
  int32_t currentBalance;
  int32_t currentCredit;
};
dataCurrentMeter currentDataMeter;

// PIN CONFIG
#define RXD2 16 // PIN RX2
#define TXD2 17 // PIN TX2
#define PIN_CONTACT 15
#define PIN_ETHERNET_CS 4
#define PIN_LED_LINK_KWH 27
#define PIN_LED_ACTIVE_KWH 15
// PIN CONFIG

struct dataMeter
{
  char serialnumber[20];
  float kwhtotal;
};
dataMeter datameter;

EdmiCMDReader edmiread(Serial2, RXD2, TXD2);

char topicDevice[50];
char topicEthernetConfigRequest[50];
char topicTransactionReqs[50];
char topicTransactionResp[50];
char topicMeter[50];

// MQTT BUFFER INIT
const int payloadSize = 256;
const int topicSize = 128;
char received_topic[topicSize];
char received_payload[payloadSize];
unsigned int received_length;
bool received_msg = false;
// MQTT BUFFER INIT

char idDevice[] = "3000";
int32_t initCredit = 20;
int32_t initStan;
uint32_t kwhStanPrev;
uint32_t kwhStanNow;
int balanceNow;
int kwhUnit = 1000;
int PREFSstan;
int PREFSbalance;
int PREFSCredit;

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

void PRINTDBG_meter(const char *)
{
  // Serial.printf();
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

void meterKeepAlive(void)
{
  edmiread.keepAlive();
}

void KWHInit()
{
  PREFS.begin("meter", false);
  PREFSstan = PREFS.getUInt("stan", 0);
  PREFSbalance = PREFS.getUInt("balance", 0);
  PREFSCredit = PREFS.getUInt("credit", 0);
  Serial.printf("MEMORY DATA -> stan: %d balance: %d credit: %d\n", PREFSstan, PREFSbalance, PREFSCredit);
  strcpy(currentDataMeter.serialnumber, edmiread.serialNumber().c_str());
  Serial.printf("%s\n", currentDataMeter.serialnumber);
  currentDataMeter.currentCredit = PREFSCredit;
  edmiread.manual_login();
  initStan = (int32_t)edmiread.read_kwhTotal();
  initStan = initStan / kwhUnit;
  if (PREFSstan == 0)
  {
    PREFSstan = initStan;
    PREFS.putUInt("stan", PREFSstan);
  }
  kwhStanNow = initStan;
  balanceNow = PREFSbalance - (initStan - PREFSstan);
  edmiread.manual_logout();
  Serial.println(PREFSstan);
  Serial.println(PREFSbalance);
  Serial.println(PREFSbalance);
}

uint32_t readKWH()
{
  uint32_t stanNow;
  if (edmiread.manual_login() == 1)
  {
    blinkLINKOn();
    stanNow = (uint32_t)edmiread.read_kwhTotal() / kwhUnit;
  }
  edmiread.manual_logout();
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

void updateInternetData()
{

  // ethernet["ipaddress"] = "0.0.0.0";
  // ethernet["ipgateway"] = "0.0.0.0";
  // ethernet["ipsubnet"] = "0.0.0.0";
  // ethernet["ipdns"] = "0.0.0.0";
}

void updateData()
{
  char buffer[100];
  JsonDevice(1);
  JsonKWH();

#ifdef USE_WIFI
  deviceDoc["ethernet"]["ipaddress"] = WiFi.localIP().toString();
  deviceDoc["ethernet"]["ipgateway"] = WiFi.gatewayIP().toString();
  deviceDoc["ethernet"]["ipsubnet"] = WiFi.subnetMask().toString();
  deviceDoc["ethernet"]["ipdns"] = WiFi.dnsIP().toString();
#endif
  // ethernet["ipaddress"] = "0.0.0.0";
  // ethernet["ipgateway"] = "0.0.0.0";
  // ethernet["ipsubnet"] = "0.0.0.0";
  // ethernet["ipdns"] = "0.0.0.0";

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

void ProcessLooping()
{
  kwhStanPrev = kwhStanNow;
  // Serial.printf("kwhprev: %d ", kwhStanPrev);
  kwhStanNow = readKWH();
  // Serial.printf("kwhnow: %d ", kwhStanNow);
  currentDataMeter.currentStan = kwhStanNow;

  if (kwhStanNow != kwhStanPrev)
  {
    balanceNow = balanceNow - (kwhStanNow - kwhStanPrev);
    PREFS.putUInt("balance", balanceNow);
    PREFS.putUInt("stan", kwhStanNow);
  }

  // Serial.printf("balance: %d\n", balanceNow);
  currentDataMeter.currentBalance = balanceNow;
  if (currentDataMeter.currentBalance == 0)
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
  sprintf(topicEthernetConfigRequest, "%s/%s/%s", "KWHControl", idDevice, "device/request");
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
}

bool getSerial = false;

void setup()
{
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  pinMode(PIN_CONTACT, OUTPUT);
  pinMode(PIN_LED_LINK_KWH, OUTPUT);
  pinMode(PIN_LED_ACTIVE_KWH, OUTPUT);

  // WiFi.disconnect();
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin("MGI-MNC", "#neurixmnc#");
  // WiFi.begin("lepi", "1234567890");
  // WiFi.begin("WiFi kost", "rumahkos");

  mqtt.setServer(mqttServer, 1883);
  mqtt.setCallback(MqttCallback);
  MqttTopicInit();
  JsonInit();

  KWHInit();

  Alarm.timerRepeat(15, updateData);
}

long mil;

void loop()
{
  if (!getSerial)
  {
    strcpy(currentDataMeter.serialnumber, edmiread.serialNumber().c_str());
    getSerial = true;
  }

  if (!mqtt.connected())
  {
    MqttReconnect();
  }
  saveEthConfigMqtt();
  TopupProcess();
  ProcessLooping();
  mqtt.loop();
  Alarm.delay(0);
}