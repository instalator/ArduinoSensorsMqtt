#include <SPI.h>           // Ethernet shield
#include <Ethernet.h>      // Ethernet shield
#include <PubSubClient.h>  // MQTT 
#include <EEPROM.h>
#include <Servo.h>
#include <dht.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <avr/wdt.h>
#include <TextFinder.h>
#include <avr/pgmspace.h>

#define ID_CONNECT   "bedroom"
#define SERVO_PIN    3 //Порт к которому подключен сервопривод
#define ONE_WIRE_BUS 8
#define DHT22_PIN    9
#define MQ7_PIN      A0 // Вход подключения MQ7
#define FOTO_PIN     A1 // Устанавливаем вход для Фоторезистора

byte mac[]    = { 0x02, 0x23, 0x40, 0x41, 0x8A, 0x13 };
byte mqtt_serv[] = {192, 168, 1, 10};  // IP MQTT сервера

const char htmlx0[] PROGMEM = "<html><title>Controller IO setup Page</title><body marginwidth=\"0\" marginheight=\"0\" ";
const char htmlx1[] PROGMEM = "leftmargin=\"0\" \"><table bgcolor=\"#999999\" border";
const char htmlx2[] PROGMEM = "=\"0\" width=\"100%\" cellpadding=\"1\" ";
const char htmlx3[] PROGMEM = "\"><tr><td>&nbsp Controller IO setup Page</td></tr></table><br>";
const char* const string_table0[] PROGMEM = {htmlx0, htmlx1, htmlx2, htmlx3};

const char htmla0[] PROGMEM = "<script>function hex2num (s_hex) {eval(\"var n_num=0X\" + s_hex);return n_num;}";
const char htmla1[] PROGMEM = "</script><table><form><input type=\"hidden\" name=\"SBM\" value=\"1\"><tr><td>MAC:&nbsp&nbsp&nbsp";
const char htmla2[] PROGMEM = "<input id=\"T1\" type=\"text\" size=\"1\" maxlength=\"2\" name=\"DT1\" value=\"";
const char htmla3[] PROGMEM = "\">.<input id=\"T3\" type=\"text\" size=\"1\" maxlength=\"2\" name=\"DT2\" value=\"";
const char htmla4[] PROGMEM = "\">.<input id=\"T5\" type=\"text\" size=\"1\" maxlength=\"2\" name=\"DT3\" value=\"";
const char htmla5[] PROGMEM = "\">.<input id=\"T7\" type=\"text\" size=\"1\" maxlength=\"2\" name=\"DT4\" value=\"";
const char htmla6[] PROGMEM = "\">.<input id=\"T9\" type=\"text\" size=\"1\" maxlength=\"2\" name=\"DT5\" value=\"";
const char htmla7[] PROGMEM = "\">.<input id=\"T11\" type=\"text\" size=\"1\" maxlength=\"2\" name=\"DT6\" value=\"";
const char* const string_table1[] PROGMEM = {htmla0, htmla1, htmla2, htmla3, htmla4, htmla5, htmla6, htmla7};

const char htmlb0[] PROGMEM = "\"><input id=\"T2\" type=\"hidden\" name=\"DT1\"><input id=\"T4\" type=\"hidden\" name=\"DT2";
const char htmlb1[] PROGMEM = "\"><input id=\"T6\" type=\"hidden\" name=\"DT3\"><input id=\"T8\" type=\"hidden\" name=\"DT4";
const char htmlb2[] PROGMEM = "\"><input id=\"T10\" type=\"hidden\" name=\"DT5\"><input id=\"T12\" type=\"hidden\" name=\"D";
const char htmlb3[] PROGMEM = "T6\"></td></tr><tr><td>MQTT: <input type=\"text\" size=\"1\" maxlength=\"3\" name=\"DT7\" value=\"";
const char htmlb4[] PROGMEM = "\">.<input type=\"text\" size=\"1\" maxlength=\"3\" name=\"DT8\" value=\"";
const char htmlb5[] PROGMEM = "\">.<input type=\"text\" size=\"1\" maxlength=\"3\" name=\"DT9\" value=\"";
const char htmlb6[] PROGMEM = "\">.<input type=\"text\" size=\"1\" maxlength=\"3\" name=\"DT10\" value=\"";
const char* const string_table2[] PROGMEM = {htmlb0, htmlb1, htmlb2, htmlb3, htmlb4, htmlb5, htmlb6};

const char htmlc0[] PROGMEM = "\"></td></tr><tr><td><br></td></tr><tr><td><input id=\"button1\"type=\"submit\" value=\"SAVE\" ";
const char htmlc1[] PROGMEM = "></td></tr></form></table></body></html>";
const char* const string_table3[] PROGMEM = {htmlc0, htmlc1};

const char htmld0[] PROGMEM = "Onclick=\"document.getElementById('T2').value ";
const char htmld1[] PROGMEM = "= hex2num(document.getElementById('T1').value);";
const char htmld2[] PROGMEM = "document.getElementById('T4').value = hex2num(document.getElementById('T3').value);";
const char htmld3[] PROGMEM = "document.getElementById('T6').value = hex2num(document.getElementById('T5').value);";
const char htmld4[] PROGMEM = "document.getElementById('T8').value = hex2num(document.getElementById('T7').value);";
const char htmld5[] PROGMEM = "document.getElementById('T10').value = hex2num(document.getElementById('T9').value);";
const char htmld6[] PROGMEM = "document.getElementById('T12').value = hex2num(document.getElementById('T11').value);\"";
const char* const string_table4[] PROGMEM = {htmld0, htmld1, htmld2, htmld3, htmld4, htmld5, htmld6};


////////////////////////////////////////////////////////////////////////////
void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  String strTopic = String(topic);
  String strPayload = String((char*)payload);
  callback_iobroker(strTopic, strPayload);
}
////////////////////////////////////////////////////////////////////////////
EthernetClient ethClient;
EthernetServer http_server(80);
PubSubClient mqtt(ethClient);
Servo servo;
dht DHT;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS_sensors(&oneWire);
DeviceAddress addr_T_radiator = { 0x28, 0xFF, 0xFD, 0x6D, 0x53, 0x15, 0x01, 0xDD }; ///////////зпменить
DeviceAddress addr_T_in = { 0x28, 0x9C, 0xE2, 0x44, 0x06, 0x00, 0x00, 0xB4 };

unsigned long prevMillis = 0; //для reconnect
unsigned long prevMillis2 = 0;
int count = 0;
int ServoAngle = 100; //Угол сервопривода при включении
bool Window = false;
bool old_Window = true;
bool Motion = false;
bool old_Motion = true;
int MQ7Value = analogRead(MQ7_PIN);
unsigned int fotoValue = analogRead(FOTO_PIN);
float T_radiator = 0; //Temp_radiator
float T_in = 0; //Temp_in
float Hout = 0;
float Tout = 0;
static char buf [100];
const byte ID = 0x91;
char buffer[100];

const int start_DI_pin [] = {2, 4, 5}; // Порты ВВОДА
int n_DI_pin = sizeof(start_DI_pin) / sizeof(start_DI_pin[0]) - 1; //Вычисляем длинну массива
const int start_DO_pin [] = {6, 7}; //Порты ВЫВОДА
int n_DO_pin = sizeof(start_DO_pin) / sizeof(start_DO_pin[0]) - 1; //Вычисляем длинну массива

////////////////////////////////////////////////////////////////////////////
void setup() {
  MCUSR = 0;
  wdt_disable();
  //Serial.begin(9600);
  for (int i = 0 ; i <= n_DI_pin; i++) {
    pinMode (start_DI_pin [i], INPUT);
  }
  digitalWrite(5, HIGH); //Окно
  digitalWrite(4, HIGH); //Объемник
  for (int i = 0 ; i <= n_DO_pin; i++) {
    pinMode (start_DO_pin [i], OUTPUT);
  }
  mqttSetup();
  httpSetup();

  if (mqtt.connect(ID_CONNECT)) {
    char s[16];
    sprintf(s, "%d.%d.%d.%d", Ethernet.localIP()[0], Ethernet.localIP()[1], Ethernet.localIP()[2], Ethernet.localIP()[3]);
    mqtt.publish("myhome/Bedroom/ip", s);
    getSensors();
    PubTopic();
    mqtt.publish("myhome/Bedroom/connection", "true");
    mqtt.subscribe("myhome/Bedroom/#");
  }
  wdt_enable(WDTO_8S);
}
/////////////////////////////////////////////////////////////////////////
void loop() {
  wdt_reset();
  checkHttp();
  mqtt.loop();
  //control();
  if (!mqtt.connected()) {
    if (millis() - prevMillis > 10000) {
      prevMillis = millis();
      reconnect();
    }
  } else {
    getSensors();
    if (millis() - prevMillis2 > 5000) {
      prevMillis2 = millis();
      PubTopic();
    }
    if (Window != old_Window) {
      old_Window = Window;
      mqtt.publish("myhome/Bedroom/Window", BoolToChar(Window));
    }
    if (Motion != old_Motion) {
      old_Motion = Motion;
      mqtt.publish("myhome/Bedroom/Motion", BoolToChar(Motion));
    }
  }
}

void PubTopic() {
  mqtt.publish("myhome/Bedroom/Temp_room", FloatToChar(Tout));
  mqtt.publish("myhome/Bedroom/Humidity_room", FloatToChar(Hout));
  mqtt.publish("myhome/Bedroom/Temp_radiator", FloatToChar(T_radiator));
  mqtt.publish("myhome/Bedroom/Temp_in", FloatToChar(T_in));
  mqtt.publish("myhome/Bedroom/Lux", IntToChar(fotoValue));
  mqtt.publish("myhome/Bedroom/Servo", IntToChar(ServoAngle));
  mqtt.publish("myhome/Bedroom/MQ7", IntToChar(MQ7Value));
}

void reconnect() {
  count++;
  if (mqtt.connect(ID_CONNECT)) {
    count = 0;
    wdt_reset();
    mqtt.publish("myhome/Bedroom/connection", "true");
    getSensors();
    PubTopic();
    mqtt.subscribe("myhome/Bedroom/#");
  }
  if (count > 10) {
    wdt_enable(WDTO_15MS);
    for (;;) {}
  }
}

void callback_iobroker(String strTopic, String strPayload) {
  if (strTopic == "myhome/Bedroom/Servo") {
    ServoAngle = strPayload.toInt();
    if (ServoAngle >= 0 && ServoAngle <= 255) {
      servo.attach(SERVO_PIN);
      servo.write(ServoAngle); //ставим вал под 0
      delay(1000);
      servo.detach();
    }
  }
}

void getSensors() {
  Window = digitalRead(5);
  Motion = digitalRead(4);
  MQ7Value = analogRead(MQ7_PIN);
  fotoValue = analogRead(FOTO_PIN);  // считываем значение с фоторезистора
  if (DHT.read22(DHT22_PIN) == DHTLIB_OK) {
    if (DHT.humidity != 0 || DHT.temperature != 0) {
      Hout = DHT.humidity;
      Tout = DHT.temperature;
    }
  }
  DS_sensors.requestTemperatures();
  T_radiator = DS_sensors.getTempC(addr_T_radiator);
  T_in = DS_sensors.getTempC(addr_T_in);
}

const char* IntToChar (int v) {
  sprintf(buf, "%d", v);
  return buf;
}
const char* FloatToChar (float f) {
  sprintf(buf, "%d.%02d", (int)f, (unsigned int)(f * 100) % 100);
  return buf;
}
const char* BoolToChar (bool r) {
  return r ? "true" : "false";
}

/////////////////////////////////////////////////////////////////////////////////
void mqttSetup() {
  int idcheck = EEPROM.read(0);
  if (idcheck == ID) {
    for (int i = 0; i < 4; i++) {
      mqtt_serv[i] = EEPROM.read(i + 7);
    }
  }
  mqtt.setServer(mqtt_serv, 1883);
  mqtt.setCallback(callback);
}

void httpSetup() {
  int idcheck = EEPROM.read(0);
  if (idcheck == ID) {
    for (int i = 0; i < 6; i++) {
      mac[i] = EEPROM.read(i + 1);
    }
  }
  if (Ethernet.begin(mac) == 0) {
  }
}

void checkHttp() {
  EthernetClient http = http_server.available();
  if (http) {
    TextFinder  finder(http );
    while (http.connected()) {
      if (http.available()) {
        if ( finder.find("GET /") ) {
          if (finder.findUntil("setup", "\n\r")) {
            if (finder.findUntil("SBM", "\n\r")) {
              byte SET = finder.getValue();
              while (finder.findUntil("DT", "\n\r")) {
                int val = finder.getValue();
                if (val >= 1 && val <= 6) {
                  mac[val - 1] = finder.getValue();
                }
                if (val >= 7 && val <= 10) {
                  mqtt_serv[val - 7] = finder.getValue();
                }
              }
              for (int i = 0 ; i < 6; i++) {
                EEPROM.write(i + 1, mac[i]);
              }
              for (int i = 0 ; i < 4; i++) {
                EEPROM.write(i + 7, mqtt_serv[i]);
              }
              EEPROM.write(0, ID);
              http.println("HTTP/1.1 200 OK");
              http.println("Content-Type: text/html");
              http.println();
              for (int i = 0; i < 4; i++) {
                strcpy_P(buffer, (char*)pgm_read_word(&(string_table0[i])));
                http.print( buffer );
              }
              http.println();
              http.print("Saved!");
              http.println();
              http.print("Restart");
              for (int i = 1; i < 10; i++) {
                http.print(".");
                delay(500);
              }
              http.println("OK");
              Reset(); // ребутим с новыми параметрами
            }
            http.println("HTTP/1.1 200 OK");
            http.println("Content-Type: text/html");
            http.println();
            for (int i = 0; i < 4; i++) {
              strcpy_P(buffer, (char*)pgm_read_word(&(string_table0[i])));
              http.print( buffer );
            }
            for (int i = 0; i < 3; i++) {
              strcpy_P(buffer, (char*)pgm_read_word(&(string_table1[i])));
              http.print( buffer );
            }
            http.print(mac[0], HEX);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table1[3])));
            http.print( buffer );
            http.print(mac[1], HEX);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table1[4])));
            http.print( buffer );
            http.print(mac[2], HEX);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table1[5])));
            http.print( buffer );
            http.print(mac[3], HEX);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table1[6])));
            http.print( buffer );
            http.print(mac[4], HEX);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table1[7])));
            http.print( buffer );
            http.print(mac[5], HEX);
            for (int i = 0; i < 4; i++) {
              strcpy_P(buffer, (char*)pgm_read_word(&(string_table2[i])));
              http.print( buffer );
            }
            http.print(mqtt_serv[0], DEC);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table2[4])));
            http.print( buffer );
            http.print(mqtt_serv[1], DEC);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table2[5])));
            http.print( buffer );
            http.print(mqtt_serv[2], DEC);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table2[6])));
            http.print( buffer );
            http.print(mqtt_serv[3], DEC);
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table3[0])));
            http.print( buffer );
            for (int i = 0; i < 7; i++) {
              strcpy_P(buffer, (char*)pgm_read_word(&(string_table4[i])));
              http.print( buffer );
            }
            strcpy_P(buffer, (char*)pgm_read_word(&(string_table3[1])));
            http.print( buffer );
            break;
          }
        }
        http.println("HTTP/1.1 200 OK");
        http.println("Content-Type: text/html");
        http.println();
        http.print("IOT controller [");
        http.print(ID_CONNECT);
        http.print("]: go to <a href=\"/setup\"> setup</a>");
        break;
      }
    }
    delay(1);
    http.stop();
  } else {
    return;
  }
}

void Reset() {
  for (;;) {}
}
