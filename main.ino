#include <SPI.h>           // Ethernet shield
#include <Ethernet.h>      // Ethernet shield
#include <PubSubClient.h>  // MQTT 
#include <Servo.h>
#include <dht.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <avr/wdt.h>
Servo servo;

byte mac[]    = { 0x01, 0x23, 0x40, 0x41, 0x8A, 0x13 };
byte server[] = { 192, 168, 1, 190 }; //IP Брокера
byte ip[]     = { 192, 168, 1, 57 };  //IP Клиента (Arduino)

////////////////////////////////////////////////////////////////////////////
void callback(char* topic, byte* payload, unsigned int length) {
    payload[length] = '\0';
    String strTopic = String(topic);
    String strPayload = String((char*)payload);
    callback_iobroker(strTopic, strPayload);
}
////////////////////////////////////////////////////////////////////////////
EthernetClient ethClient;
PubSubClient client(server, 1883, callback, ethClient);

#define ID_CONNECT   "bedroom"
#define SERVO_PIN    3 //Порт к которому подключен сервопривод
#define ONE_WIRE_BUS 8
#define DHT22_PIN    9

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
int pause = 0;
int MQ7_Pin = A0; // Куда подключили MQ7
int MQ7Value = analogRead(MQ7_Pin); 
int foto_Pin = A1;    // устанавливаем входную ногу для Фоторезистора
unsigned int fotoValue = analogRead(foto_Pin);
float T_radiator = 0; //Temp_radiator
float T_in = 0; //Temp_in
float Hout = 0;
float Tout = 0;
static char buf [100];

const int start_DI_pin []= {2, 4, 5}; // Порты ВВОДА
int n_DI_pin = sizeof(start_DI_pin) / sizeof(start_DI_pin[0])-1; //Вычисляем длинну массива
const int start_DO_pin []= {6, 7}; //Порты ВЫВОДА
int n_DO_pin = sizeof(start_DO_pin) / sizeof(start_DO_pin[0])-1; //Вычисляем длинну массива

////////////////////////////////////////////////////////////////////////////
void setup() {
  MCUSR = 0;
  wdt_disable();
  Serial.begin(115200);
  //Serial.println("start");
  for(int i=0 ;i<=n_DI_pin; i++) { pinMode (start_DI_pin [i], INPUT); }
  digitalWrite(5, HIGH); //Окно
  digitalWrite(4, HIGH); //Объемник
  for(int i=0 ;i<=n_DO_pin; i++) { pinMode (start_DO_pin [i], OUTPUT); }

  Ethernet.begin(mac, ip);
  if (client.connect(ID_CONNECT)) {
    getSensors();
    //control();
    PubTopic();
    client.subscribe("myhome/Bedroom/#");
  }
  wdt_enable(WDTO_8S);
}
/////////////////////////////////////////////////////////////////////////
void loop() {
  wdt_reset();
  client.loop();
  //control();
  getSensors();
  if (millis() - prevMillis2 > 5000){
        prevMillis2 = millis();
        PubTopic();
  }
  if(Window != old_Window){
      old_Window = Window;
      client.publish("myhome/Bedroom/Window", BoolToChar(Window));
    }
    if(Motion != old_Motion){
      old_Motion = Motion;
      client.publish("myhome/Bedroom/Motion", BoolToChar(Motion));
    }
  if (!client.connected()){
     if (millis() - prevMillis > 5000){
        prevMillis = millis();
        reconnect();
     }
  }
}

void PubTopic(){
    client.publish("myhome/Bedroom/Temp_room", FloatToChar(Tout));
    client.publish("myhome/Bedroom/Humidity_room", FloatToChar(Hout));
    client.publish("myhome/Bedroom/Temp_radiator", FloatToChar(T_radiator));
    client.publish("myhome/Bedroom/Temp_in", FloatToChar(T_in));
    client.publish("myhome/Bedroom/Lux", IntToChar(fotoValue));
    client.publish("myhome/Bedroom/Servo", IntToChar(ServoAngle));
    client.publish("myhome/Bedroom/MQ7", IntToChar(MQ7Value));
}

void reconnect() {
    count++;
    if (client.connect(ID_CONNECT)) {
      count = 0;
      wdt_reset();
      client.publish("myhome/lighting/connection", "true");
      //control();
      getSensors();
      PubTopic();
      client.subscribe("myhome/Bedroom/#");
    }
    if (count > 10){
      wdt_enable(WDTO_15MS);
        for(;;){}
    }
}

void callback_iobroker(String strTopic, String strPayload){
  if (strTopic == "myhome/Bedroom/Servo"){
        ServoAngle=strPayload.toInt();
    if (ServoAngle >= 0 && ServoAngle <= 255){
          servo.attach(SERVO_PIN);
          //delay(200);
          servo.write(ServoAngle); //ставим вал под 0
          delay(1000);
          //client.publish("myhome/Bedroom/Servo", b); 
          servo.detach();
    }
  }
}

void getSensors () {
  Window = digitalRead(5);
  Motion = digitalRead(4);
  MQ7Value = analogRead(MQ7_Pin); 
  fotoValue = analogRead(foto_Pin);  // считываем значение с фоторезистора

  if (DHT.read22(DHT22_PIN)== DHTLIB_OK) {
    if(DHT.humidity != 0 || DHT.temperature != 0){
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
  sprintf(buf, "%d.%02d", (int)f, (int)(f*100)%100);
  return buf;
}

const char* BoolToChar (bool r) {
    return r ? "true" : "false";
}
