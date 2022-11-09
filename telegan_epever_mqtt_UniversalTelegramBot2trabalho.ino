#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>

#define WIFI_SSID "xxxxxx"
#define WIFI_PASSWORD "xxxxxxx"
#define BOT_TOKEN "xxxxxxxxxx"
#include <ModbusMaster.h>
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

const int defaultBaudRate = 115200;
int timerTask1, timerTask2, timerTask3;
float battChargeCurrent, battDischargeCurrent, battOverallCurrent, battChargePower;
float bvoltage, ctemp, btemp, bremaining, lpower, lcurrent, pvvoltage, pvcurrent, pvpower;
float stats_today_pv_volt_min, stats_today_pv_volt_max;
uint8_t result;
bool rs485DataReceived = true;
bool loadPoweredOn = true;


ModbusMaster node;

const unsigned long BOT_MTBS = 1000; 
unsigned long bot_lasttime; 
WiFiClientSecure secured_client;
UniversalTelegramBot bot(BOT_TOKEN, secured_client);

void handleNewMessages(int numNewMessages)
{
  Serial.print("handleNewMessages ");
  Serial.println(numNewMessages);
  
  String answer;
  for (int i = 0; i < numNewMessages; i++)
  {
    telegramMessage &msg = bot.messages[i];
    Serial.println("Received " + msg.text);
    if (msg.text == "??")
    {
        Serial.println(msg.chat_id);
      bot.sendMessage(msg.chat_id, "EPEVER DADOS" );
        String pvVoltage;
        pvVoltage = pvvoltage;
        bot.sendMessage(msg.chat_id, "Tensão fotovoltaica: ");
        bot.sendMessage(msg.chat_id, pvVoltage); 
        String pvCurrent;
        pvCurrent = pvcurrent;
        bot.sendMessage(msg.chat_id, "Corrente fotovoltaica: ");
        bot.sendMessage(msg.chat_id, pvCurrent);
        
        String pvPower;
        pvPower = pvpower;
        bot.sendMessage(msg.chat_id, "Potência fotovoltaica: ");
        bot.sendMessage(msg.chat_id, pvPower);
        
        String bVoltage;
        bVoltage = bvoltage;
        bot.sendMessage(msg.chat_id, "Voltagem da bateria: ");
        bot.sendMessage(msg.chat_id, bVoltage);
        
        String BattChargeCurrent;
        BattChargeCurrent = battChargeCurrent;
        bot.sendMessage(msg.chat_id, "Corrente de carga da bateria: ");
        bot.sendMessage(msg.chat_id, BattChargeCurrent);

        String BattChargePower;
        BattChargePower = battChargePower;
        bot.sendMessage(msg.chat_id, "Potência de carga da bateria: ");
        bot.sendMessage(msg.chat_id, BattChargePower);

        String lCurrent;
        lCurrent = lcurrent;
        bot.sendMessage(msg.chat_id, "Corrente da carga na saida load: ");
        bot.sendMessage(msg.chat_id, lCurrent);

        String lPower;
        lPower = lpower;
        bot.sendMessage(msg.chat_id, "Potência da carga na saida load: ");
        bot.sendMessage(msg.chat_id, lPower);

        String bRemaining;
        bRemaining = bremaining;
        bot.sendMessage(msg.chat_id, "Bateria restante %: ");
        bot.sendMessage(msg.chat_id, bRemaining);

        String bTemp;
        bTemp = btemp;
        bot.sendMessage(msg.chat_id, "Temperatura da bateria: ");
        bot.sendMessage(msg.chat_id, bTemp);
       }
    
  }
}

void bot_setup()
{
  const String commands = F( );
  bot.setMyCommands(commands);
}


void preTransmission() {
}

void postTransmission() {
}

typedef void (*RegistryList[])();

RegistryList Registries = {
  AddressRegistry_3100,
  AddressRegistry_3106,
  AddressRegistry_310D,
  AddressRegistry_311A,
  AddressRegistry_331B,
};

uint8_t currentRegistryNumber = 0;

void nextRegistryNumber() {
  currentRegistryNumber++;
  if (currentRegistryNumber >= ARRAY_SIZE(Registries)) {
    currentRegistryNumber = 0;
  }
}


void executeCurrentRegistryFunction() {
  Registries[currentRegistryNumber]();
}

uint8_t setOutputLoadPower(uint8_t state) {
  Serial.print("Writing coil 0x0006 value to: ");
  Serial.println(state);

  delay(10);
  result = node.writeSingleCoil(0x0006, state);

  if (result == node.ku8MBSuccess) {
    node.getResponseBuffer(0x00);
    Serial.println("Success.");
  }

  return result;
}

uint8_t readOutputLoadState() {
  delay(10);
  result = node.readHoldingRegisters(0x903D, 1);

  if (result == node.ku8MBSuccess) {
    loadPoweredOn = (node.getResponseBuffer(0x00) & 0x02) > 0;

    Serial.print("Set success. Load: ");
    Serial.println(loadPoweredOn);
  } else {
    Serial.println("readHoldingRegisters(0x903D, 1) failed!");
  }
  return result;
}

uint8_t checkLoadCoilState() {
  Serial.print("Reading coil 0x0006... ");

  delay(10);
  result = node.readCoils(0x0006, 1);

  Serial.print("Result: ");
  Serial.println(result);

  if (result == node.ku8MBSuccess) {
    loadPoweredOn = (node.getResponseBuffer(0x00) > 0);

    Serial.print(" Value: ");
    Serial.println(loadPoweredOn);
  } else {
    Serial.println("Failed to read coil 0x0006!");
  }

  return result;
}

// -----------------------------------------------------------------

void AddressRegistry_3100() {
  result = node.readInputRegisters(0x3100, 6);

  if (result == node.ku8MBSuccess) {

    pvvoltage = node.getResponseBuffer(0x00) / 100.0f;
    Serial.print("PV Voltage: ");
    Serial.println(pvvoltage);

    pvcurrent = node.getResponseBuffer(0x01) / 100.0f;
    Serial.print("PV Current: ");
    Serial.println(pvcurrent);

    pvpower = (node.getResponseBuffer(0x02) | node.getResponseBuffer(0x03) << 16) / 100.0f;
    Serial.print("PV Power: ");
    Serial.println(pvpower);

    bvoltage = node.getResponseBuffer(0x04) / 100.0f;
    Serial.print("Battery Voltage: ");
    Serial.println(bvoltage);

    battChargeCurrent = node.getResponseBuffer(0x05) / 100.0f;
    Serial.print("Battery Charge Current: ");
    Serial.println(battChargeCurrent);
  }
}

void AddressRegistry_3106()
{
  result = node.readInputRegisters(0x3106, 2);

  if (result == node.ku8MBSuccess) {
    battChargePower = (node.getResponseBuffer(0x00) | node.getResponseBuffer(0x01) << 16)  / 100.0f;
    Serial.print("Battery Charge Power: ");
    Serial.println(battChargePower);
  }
}

void AddressRegistry_310D()
{
  result = node.readInputRegisters(0x310D, 3);

  if (result == node.ku8MBSuccess) {
    lcurrent = node.getResponseBuffer(0x00) / 100.0f;
    Serial.print("Load Current: ");
    Serial.println(lcurrent);

    lpower = (node.getResponseBuffer(0x01) | node.getResponseBuffer(0x02) << 16) / 100.0f;
    Serial.print("Load Power: ");
    Serial.println(lpower);
  } else {
    rs485DataReceived = false;
    Serial.println("Read register 0x310D failed!");
  }
}

void AddressRegistry_311A() {
  result = node.readInputRegisters(0x311A, 2);

  if (result == node.ku8MBSuccess) {
    bremaining = node.getResponseBuffer(0x00) / 1.0f;
    Serial.print("Battery Remaining %: ");
    Serial.println(bremaining);

    btemp = node.getResponseBuffer(0x01) / 100.0f;
    Serial.print("Battery Temperature: ");
    Serial.println(btemp);
  } else {
    rs485DataReceived = false;
    Serial.println("Read register 0x311A failed!");
  }
}

void AddressRegistry_331B() {
  result = node.readInputRegisters(0x331B, 2);

  if (result == node.ku8MBSuccess) {
    battOverallCurrent = (node.getResponseBuffer(0x00) | node.getResponseBuffer(0x01) << 16) / 100.0f;
    Serial.print("Battery Discharge Current: ");
    Serial.println(battOverallCurrent);
  } else {
    rs485DataReceived = false;
    Serial.println("Read register 0x331B failed!");
  }
}


#define p_ledtick LED_BUILTIN
int ledState = LOW; 
unsigned long previousMillis = 0;

#define RXD2 16
#define TXD2 17

void setup()
{
  Serial.begin(defaultBaudRate);
  Serial2.begin(defaultBaudRate);
  
  node.begin(1, Serial2);

  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);


  pinMode(p_ledtick, OUTPUT);


  Serial.println("Setup OK!");
  Serial.println("----------------------------");
  Serial.println();

  Serial.print("Connecting to Wifi SSID ");
  Serial.print(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  secured_client.setCACert(TELEGRAM_CERTIFICATE_ROOT); 
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.print("\nWiFi connected. IP address: ");
  Serial.println(WiFi.localIP());

  Serial.print("Retrieving time: ");
  configTime(0, 0, "pool.ntp.org"); 
  time_t now = time(nullptr);
  while (now < 24 * 3600)
  {
    Serial.print(".");
    delay(100);
    now = time(nullptr);
  }
  Serial.println(now);

  bot_setup();
}

void loop()
{
  telegran();
  unsigned long currentMillis = millis();
  const long interval = 5000;
  if (currentMillis - previousMillis >= interval) 
  {
    previousMillis = currentMillis;

    if (ledState == LOW) 
    {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }

    digitalWrite(p_ledtick, ledState);

    executeCurrentRegistryFunction();
    nextRegistryNumber();
  }
}



void telegran()
{
  if (millis() - bot_lasttime > BOT_MTBS)
  {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

    while (numNewMessages)
    {
      Serial.println("got response");
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }

    bot_lasttime = millis();
  }
}
