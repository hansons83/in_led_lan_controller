/*
 Basic MQTT example
*/

#include <SPI.h>
//#include <Wire.h>
#include <Ethernet2.h>
#include <PubSubClient.h>
#include <MsTimer2.h>
#include <OneWire.h>
#include <EEPROM.h>

#define SDA_PORT PORTC
#define SDA_PIN 4
#define SCL_PORT PORTC
#define SCL_PIN 5

#define I2C_TIMEOUT 100
#define I2C_FASTMODE 1

#include <SoftWire.h>

SoftWire Wire = SoftWire();

#define EEPROM_VERSION_OFFSET  0
#define EEPROM_SETTINGS_OFFSET 1
#define EEPROM_VERSION         0x55

#define ETH_SHIELD_RESET_PIN   A0

static struct StoredSettings{
  uint8_t  ver;
  uint8_t  mqtt_ip[4];
  uint16_t mqtt_port;
  char     mqtt_username[32];
  char     mqtt_password[16];
  // fade in/out time from 0 to 100
  uint16_t time;
} boardSettings;

#define set_bit(var, bit_nr) ((var) |= 1 << (bit_nr))
#define clear_bit(var, bit_nr) ((var) &= ~(1 << (bit_nr)))
#define get_bit(var, bit_nr) (((var) & (1 << (bit_nr))) ? true : false)

const char hex_to_char[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
inline void toHexStr (byte b, char* target)
{
  target[0] = hex_to_char[b>>4];
  target[1] = hex_to_char[b&0x0F];
}

static const uint8_t TOPIC_ID_START_INDEX = 6;

static const uint8_t TOPIC_CMD_CHANNEL_INDEX = 27;
char outputCommandTopic[]     = { "LEDIO/\0\0\0\0\0\0\0\0/command/out/+\0" };

static const uint8_t TOPIC_IN_STATE_CHANNEL_INDEX = 24;
char inputStateTopic[]  = { "LEDIO/\0\0\0\0\0\0\0\0/state/in/ \0"  };

static const uint8_t TOPIC_OUT_STATE_CHANNEL_INDEX = 25;
char outputStateTopic[] = { "LEDIO/\0\0\0\0\0\0\0\0/state/out/ \0"  };

// Update these with values suitable for your network.
byte mac[] = { 0x6C, 0x75, 0x00, 0x00, 0x00, 0x00, 0x00 };

EthernetServer ethServer(80);
EthernetClient remoteClient;
PubSubClient   mqttClient(remoteClient);
OneWire        ds2401(A1);

static const uint8_t NUM_IOS = 8;
static const uint8_t INPUT_HIGH_STATE = 0xFF;
static const uint8_t INPUT_LOW_STATE = 0x00;
static const uint8_t INPUT_PINS_START = 2;
static const uint8_t PCA9634_ADDRESS = 0x47;

static  uint8_t inputCounters[NUM_IOS] = {0, 0, 0, 0, 0, 0, 0, 0};
static  byte    inputsState = 0;
static  byte    inputsStateToPublish = 0;
static  byte    lastinputsState = 0;


static  float   outputCurrentVal[NUM_IOS] = {0, 0, 0, 0, 0, 0, 0, 0};
static  uint8_t outputExpectedVal[NUM_IOS] = {0, 0, 0, 0, 0, 0, 0, 0};
static  int8_t  outputStepSign[NUM_IOS] = {0, 0, 0, 0, 0, 0, 0, 0};

#define PCA9634_MODE2_NMOS_INV_VAL (1<<4)
#define PCA9634_MODE2_NMOS_OUTDRV_VAL (1<<2)

#define PCA9634_LEDOUT_OFF_VAL (0)
#define PCA9634_LEDOUT_ON_VAL  (1)
#define PCA9634_LEDOUT_PWM_VAL (2)

#define PCA9634_LED0_STATE_SHIFT 0
#define PCA9634_LED1_STATE_SHIFT 2
#define PCA9634_LED2_STATE_SHIFT 4
#define PCA9634_LED3_STATE_SHIFT 6

#define PCA9634_LED4_STATE_SHIFT 0
#define PCA9634_LED5_STATE_SHIFT 2
#define PCA9634_LED6_STATE_SHIFT 4
#define PCA9634_LED7_STATE_SHIFT 6

uint8_t PCA9634_setup()
{
  uint8_t error = 0;
  // mode 1
  Wire.beginTransmission( PCA9634_ADDRESS );
  Wire.write( 0x00 );
  Wire.write( 0x00 );
  error |= Wire.endTransmission( );
  // mode 2
  Wire.beginTransmission( PCA9634_ADDRESS );
  Wire.write( 0x01 );
  Wire.write( PCA9634_MODE2_NMOS_INV_VAL | PCA9634_MODE2_NMOS_OUTDRV_VAL );
  error |= Wire.endTransmission( );

  // LEDOUT0
  Wire.beginTransmission( PCA9634_ADDRESS );
  Wire.write( 0x0C );
  Wire.write( 0xAA );
  error |= Wire.endTransmission( );
  // LEDOUT1
  Wire.beginTransmission( PCA9634_ADDRESS );
  Wire.write( 0x0D );
  Wire.write( 0xAA );
  error |= Wire.endTransmission( );

  if(error != 0)
  {
    Serial.print("PCA er: ");
    Serial.println(error);
  }
  
  return error;
}
uint8_t PCA9634_write_pwm(uint8_t led, uint8_t val)
{
  uint8_t error = 0;
  // mode 0
  Wire.beginTransmission( PCA9634_ADDRESS );
  Wire.write( 0x02 + led );
  Wire.write( val );
  error |= Wire.endTransmission( );

  if(error != 0)
  {
    Serial.print("PCA er: ");
    Serial.println(error);
  }
  
  return error;
}
/*
uint8_t PCA9634_read(uint8_t led)
{
  uint8_t retVal;
  uint8_t error = 0;
    // mode 0
  Wire.beginTransmission( PCA9634_ADDRESS );
  Wire.write( 0x02 + led );
  error |= Wire.endTransmission( );
  Wire.requestFrom( PCA9634_ADDRESS, 1, 1);
  while(!Wire.available());
  retVal = Wire.read( );

  return retVal;
}
*/
void callback(char* topic, byte* payload, unsigned int length)
{
  int8_t ledNr = 0;
  int bright = 0;
  Serial.print("Msg: ");
  Serial.println(topic);
//  Serial.print("[");
//  int i=0;
//  for (i=0;i<length;i++) {
//    Serial.print((char)payload[i]);
//  }
//  Serial.println("]");
  payload[length] = 0;

  ledNr = topic[TOPIC_CMD_CHANNEL_INDEX] - '0';
  
  if(ledNr < 1 || ledNr > NUM_IOS)
  {
    Serial.println("Wrg LED");
    return;
  }
  bright = atoi(payload);
  if(bright < 0 || bright > 100)
  {
    Serial.println("Wrg VAL");
    return;
  }
  else
  {
    Serial.print("Exp VAL: ");
    Serial.println(bright);
    //outputCurrentVal[ledNr-1] = PCA9634_read(ledNr-1);
    Serial.println(outputCurrentVal[ledNr-1]);
    noInterrupts();
    if(outputCurrentVal[ledNr-1] > bright)
    {
      outputStepSign[ledNr-1] = -1;
    }
    else
    { 
      outputStepSign[ledNr-1] = 1;
    }
    outputExpectedVal[ledNr-1] = bright;
    interrupts();
  }
  /*if(length == 2 && memcmp(payload, "ON", 2) == 0)
  {
    setOutputState(relayPin-1, true );
  }
  else if(length == 3 && memcmp(payload, "OFF", 3) == 0)
  {
    setOutputState(relayPin-1, false );
  }
  else if(length == 6 && memcmp(payload, "TOGGLE", 6) == 0)
  {
    toggleOutputState(relayPin-1);
  }
  else
  {
    Serial.println("Wrg payload");
  }*/
}

void readInputsUpdateOutputs()
{
  for(uint8_t i = 0; i < NUM_IOS; ++i)
  {
    inputCounters[i] <<= 1;
    if(digitalRead(INPUT_PINS_START + i) == LOW)
    {
      inputCounters[i] += 1;
    }
    else
    {
      inputCounters[i] += 0;
    }
    if(inputCounters[i] == INPUT_HIGH_STATE)
    {
      set_bit(inputsState, i);
    }
    else if(inputCounters[i] == INPUT_LOW_STATE)
    {
      clear_bit(inputsState, i);
    }
    if(get_bit(inputsState, i) != get_bit(lastinputsState, i))
    {
      set_bit(inputsStateToPublish, i);
    }

    if(outputCurrentVal[i] != outputExpectedVal[i])
    {
      outputCurrentVal[i] += (200.0f / (float)boardSettings.time) * outputStepSign[i];
      if(outputStepSign[i] > 0)
      {
        if(outputCurrentVal[i] > outputExpectedVal[i])
          outputCurrentVal[i] = outputExpectedVal[i];
      }
      else
      {
        if(outputCurrentVal[i] < outputExpectedVal[i])
          outputCurrentVal[i] = outputExpectedVal[i];
      }
      Serial.println(outputCurrentVal[i]);
      PCA9634_write_pwm(i, 2.55f*outputCurrentVal[i]);
    }

  }
  lastinputsState = inputsState;
}

void checkInputsAndPublish(PubSubClient& client)
{
  noInterrupts();
  byte currentInputsStateToPublish = inputsStateToPublish;
  byte currentInputsState = inputsState;
  inputsStateToPublish = 0;
  interrupts();
  
  if(currentInputsStateToPublish == 0)
    return;
  
  for(uint8_t i = 0; i < NUM_IOS; ++i)
  {
    if(get_bit(currentInputsStateToPublish, i))
    {
      inputStateTopic[TOPIC_IN_STATE_CHANNEL_INDEX] = i + '1';
      if(get_bit(currentInputsState, i))
      {
        Serial.print(inputStateTopic);
        Serial.println(": ON");
        client.publish((const char*)inputStateTopic, "ON");
      }
      else
      {
        Serial.print(inputStateTopic);
        Serial.println(": OFF");
        client.publish((const char*)inputStateTopic, "OFF");
      }
    }
  }
}

void PrintTwoDigitHex (byte b, boolean newline)
{
  Serial.print(b/16, HEX);
  Serial.print(b%16, HEX);
  if (newline) Serial.println();
}
void getMacAddress(byte* target)
{
  int8_t i;           // This is for the for loops
  //byte crc_calc;    //calculated CRC
  //byte crc_byte;    //actual CRC as sent by ds24012401
  //1-Wire bus reset, needed to start operation on the bus,
  //returns a 1/TRUE if presence pulse detected
  if (ds2401.reset() == TRUE)
  {
    ds2401.write(0x33);  //Send Read data command
    //Serial.print("FC: 0x");
    //PrintTwoDigitHex (ds2401.read(), 1);
    //Serial.print("HD: ");
    ds2401.read();
    for (i = 3; i >= 0; i--)
    {
      target[i+2] = ds2401.read();
      toHexStr(target[i+2], outputCommandTopic + (TOPIC_ID_START_INDEX + i*2));
      toHexStr(target[i+2], inputStateTopic + (TOPIC_ID_START_INDEX + i*2));
      toHexStr(target[i+2], outputStateTopic + (TOPIC_ID_START_INDEX + i*2));
    }
    Serial.print("MAC: ");
    for (i = 0; i < 6; i++)
    {
      PrintTwoDigitHex (target[i], 0);
      if(i< 5)Serial.print(":");
    }
    Serial.println();
    Serial.println(outputCommandTopic);
    Serial.println(inputStateTopic);
    Serial.println(outputStateTopic);
  }
  else //Nothing is connected in the bus
  {
    Serial.println( "No MAC");
  }
}
void searchI2cDevices()
{
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
  // The i2c_scanner uses the return value of
  // the Write.endTransmisstion to see if
  // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C at 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("!");
  
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Err at 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
  }

}
void setup()
{
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
 
  for(uint8_t i = 0; i < NUM_IOS; ++i)
  {
    pinMode(INPUT_PINS_START + i, INPUT_PULLUP);      // sets the switch sensor digital pin as input
  }
  
  pinMode(ETH_SHIELD_RESET_PIN, OUTPUT);
  
  getMacAddress(mac);

  if(EEPROM.read(EEPROM_VERSION_OFFSET) != EEPROM_VERSION)
  {
    Serial.println("Clearing!");
    memset(&boardSettings, 0, sizeof(boardSettings));
    boardSettings.time = 200;
    EEPROM.write(EEPROM_VERSION_OFFSET, EEPROM_VERSION);
    EEPROM.put(EEPROM_SETTINGS_OFFSET, boardSettings);
  }
  
  EEPROM.get(EEPROM_SETTINGS_OFFSET, boardSettings);
  
  for(uint8_t i = 0; i < 4; ++i)
  {
    Serial.print(boardSettings.mqtt_ip[i]);
    if(i < 3)Serial.print(".");
  }
  Serial.print(":");
  Serial.println(boardSettings.mqtt_port);
  
  Serial.print(boardSettings.mqtt_username);
  Serial.print(":");
  Serial.println(boardSettings.mqtt_password);
  
  Wire.begin();

  //searchI2cDevices();

  PCA9634_setup();
  for(uint8_t i = 0; i < NUM_IOS; ++i)
  {
    PCA9634_write_pwm(i, outputCurrentVal[i]);
  }

  MsTimer2::set(2, readInputsUpdateOutputs);
  MsTimer2::start();

  // Enable eth module.
  digitalWrite(A0, HIGH);

  mqttClient.setCallback(callback);
  Serial.print("Eth: ");
  //Ethernet.begin(mac, IPAddress(192, 168, 1, 6), IPAddress(255, 255, 255, 0), IPAddress(192, 168, 1, 254));
  while(!Ethernet.begin(mac))
  {
    delay(1000);
  }
  ethServer.begin();

  Serial.println(Ethernet.localIP()); 
  //Serial.println(Ethernet.subnetMask());
  //Serial.println(Ethernet.gatewayIP());
  //Serial.println(Ethernet.dnsServerIP());
}

void handleMqttClient()
{
  static unsigned long lastMillis = 0;  
  static unsigned long lastConnectMillis = 0xFFFFFFFF;
  static long sinceLastConnect;
  
  if (!mqttClient.connected())
  {
    sinceLastConnect = millis() - lastConnectMillis;
    sinceLastConnect = sinceLastConnect > 0 ? sinceLastConnect : ((~((unsigned long)0) - lastConnectMillis) + millis());

    if(sinceLastConnect >= 10000)
    {
      mqttClient.setServer(boardSettings.mqtt_ip, boardSettings.mqtt_port);
      Serial.print("MQTT con...");
      if (mqttClient.connect((const char*)(mac+2), boardSettings.mqtt_username, boardSettings.mqtt_password)) {
        Serial.print(" conn: Sub:");
        Serial.println(outputCommandTopic);
        mqttClient.subscribe(outputCommandTopic);
      }
      else
      {
        Serial.print("Err, rc=");
        Serial.println(mqttClient.state());
      }
      lastConnectMillis = millis();
    }
  }
  else
  {
    //checkOutputsAndPublish(mqttClient);
    checkInputsAndPublish(mqttClient);
    mqttClient.loop();
  }
}
void handleHttpServer()
{
//http://192.168.1.11/?ip=192.168.1.3&port=1883&user=openhabian&pwd=swiatek123&time=500&
  
  static const int PROGMEM srvBufferSize = 100;
  static char srvBuffer[101];
  static byte srvBufferPos = 0;
  static EthernetClient remoteClient;
  
  // listen for incoming clients
  remoteClient = ethServer.available();
  if (!remoteClient)
    return;
    
  while (remoteClient.connected()) {
    if (remoteClient.available()) {
      char c = remoteClient.read();

      if (c != '\n' && c != '\r') {
        //read char by char HTTP request
        if (srvBufferPos < srvBufferSize) {

          //store characters to string 
          srvBuffer[srvBufferPos] = c;
          ++srvBufferPos;
        } 
        else
        {
          srvBufferPos = 0;
        }
        continue;
      }
    
      srvBuffer[srvBufferPos] = '\0';
      ///////////////
      //Serial.println(srvBuffer);

      //now output HTML data header
      // Parse get request
      char* ipStr, *portStr, *pch, *userStr, *pwdStr, *timeStr;
      byte counter = 0;
      /////////////////////
      ipStr = strstr(srvBuffer, "ip=");
      portStr = strstr(srvBuffer, "port=");
      userStr = strstr(srvBuffer, "user=");
      pwdStr = strstr(srvBuffer, "pwd=");
      timeStr = strstr(srvBuffer, "time=");
      if(ipStr != NULL)
      {
        ipStr += 3;
        Serial.print("ip: ");
        pch = strtok (ipStr, ".");
        while (ipStr != NULL && counter < 4)
        {
          Serial.print(pch);
          if(counter<3)Serial.print(".");
          else Serial.println("");
          boardSettings.mqtt_ip[counter] = atoi(pch);
          // go to next token
          pch = strtok (NULL, ".&");
          ++counter;
        }
      }
      if(portStr != NULL)
      {
        portStr += 5;
        Serial.print("port: ");
        pch = strtok (portStr, "&");
        Serial.println(pch);
        boardSettings.mqtt_port = atoi(pch);
      }
      if(userStr != NULL)
      {
        userStr += 5;
        Serial.print("user: ");
        pch = strtok (userStr, "&");
        Serial.println(pch);
        strcpy(boardSettings.mqtt_username, pch);
      }
      if(pwdStr != NULL)
      {
        pwdStr += 4;
        Serial.print("pwd: ");
        pch = strtok (pwdStr, "&");
        Serial.println(pch);
        strcpy(boardSettings.mqtt_password, pch);
      }
      if(timeStr != NULL)
      {
        timeStr += 5;
        Serial.print("time: ");
        pch = strtok (timeStr, "&");
        Serial.print(pch);
        Serial.println("ms");
        boardSettings.time = atoi(pch);
        if(boardSettings.time < 2)
        {
          boardSettings.time = 2;
        }
        if(boardSettings.time > 10000)
        {
          boardSettings.time = 10000;
        }
      }
      if(ipStr || portStr || userStr || pwdStr || timeStr)
      {
        EEPROM.put(EEPROM_SETTINGS_OFFSET, boardSettings);
        mqttClient.disconnect();
      }
      // Respond with current configuration
      remoteClient.println("HTTP/1.1 200 OK");
      remoteClient.println("Content-Type: text/html");
      remoteClient.println();

      remoteClient.println("<HTML>");
      remoteClient.println("<HEAD>");
      remoteClient.println("<TITLE>In/Out controller setup</TITLE>");
      remoteClient.println("</HEAD>");
      remoteClient.println("<BODY>");

      remoteClient.println("<H1>Settings</H1>");

      remoteClient.print("ip: ");
      for(uint8_t i = 0; i < 4; ++i)
      {
        remoteClient.print(boardSettings.mqtt_ip[i]);
        if(i < 3)remoteClient.print(".");
      }
      remoteClient.println("<BR>");
      remoteClient.print("port: ");
      remoteClient.print(boardSettings.mqtt_port);
      remoteClient.println("<BR>");
      remoteClient.print("user: ");
      remoteClient.print((const char*)boardSettings.mqtt_username);
      remoteClient.println("<BR>");
      remoteClient.print("pwd: ");
      remoteClient.print((const char*)boardSettings.mqtt_password);
      remoteClient.println("<BR>");
      remoteClient.print("time: ");
      remoteClient.print(boardSettings.time);
      remoteClient.print("ms");
      remoteClient.println("<BR>");

      remoteClient.println("</FORM>");
      remoteClient.println("<BR>");
      remoteClient.println("</BODY>");
      remoteClient.println("</HTML>");
      
      delay(1);
      //stopping client
      remoteClient.stop();
      //clearing string for next read
      srvBufferPos = 0;
    }
  }
}
  
static unsigned long maintainLastMillis = 0xFFFFFFFF;
static long sinceLastMaintain;
void loop()
{
  static byte res;
  sinceLastMaintain = millis() - maintainLastMillis;
  sinceLastMaintain = sinceLastMaintain > 0 ? sinceLastMaintain : ((~((unsigned long)0) - maintainLastMillis) + millis());
  if(sinceLastMaintain >= 100)
  {
    maintainLastMillis = millis();
    res = Ethernet.maintain();
    if(res == 2 || res == 4)
    {
      Serial.print("Eth: ");
      Serial.println(Ethernet.localIP()); 
    }
    else if(res == 1 || res == 3)
    {
      // Need to reset lan module?
    }
  }
  handleHttpServer();
  handleMqttClient();
}

