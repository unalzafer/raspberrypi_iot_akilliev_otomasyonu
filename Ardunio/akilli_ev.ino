
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHT.h>

#define MQTT_VERSION MQTT_VERSION_3_1_1

// Wifi: Adı ve Şifresi
const char* WIFI_SSID = "ASUS";
const char* WIFI_PASSWORD = "sevgi.seli";
//const char* WIFI_SSID = "61";
//const char* WIFI_PASSWORD = "61616161";

// MQTT: ID, server IP, port, username and password
const PROGMEM char* MQTT_CLIENT_ID = "office_rgb_light"; 
const PROGMEM char* MQTT_SERVER_IP = "192.168.1.7";  //local ip degeri
const PROGMEM uint16_t MQTT_SERVER_PORT = 1883;      //baglanti port
const PROGMEM char* MQTT_USER = "username";          //mqtt yuklemesi yaparken kullanıcı adi ve sifresi
const PROGMEM char* MQTT_PASSWORD = "raspberry";

// MQTT: konular
// state
const PROGMEM char* MQTT_LIGHT_STATE_TOPIC = "office/rgb1/light/status";
const PROGMEM char* MQTT_LIGHT_COMMAND_TOPIC = "office/rgb1/light/switch";

// brightness
const PROGMEM char* MQTT_LIGHT_BRIGHTNESS_STATE_TOPIC = "office/rgb1/brightness/status";
const PROGMEM char* MQTT_LIGHT_BRIGHTNESS_COMMAND_TOPIC = "office/rgb1/brightness/set";

// colors (rgb)
const PROGMEM char* MQTT_LIGHT_RGB_STATE_TOPIC = "office/rgb1/rgb/status";
const PROGMEM char* MQTT_LIGHT_RGB_COMMAND_TOPIC = "office/rgb1/rgb/set";

// payloads by default (on/off)
const PROGMEM char* LIGHT_ON = "ON";
const PROGMEM char* LIGHT_OFF = "OFF";

// variables used to store the state, the brightness and the color of the light
boolean m_rgb_state = false;
uint8_t m_rgb_brightness = 100;
uint8_t m_rgb_red = 255;
uint8_t m_rgb_green = 255;
uint8_t m_rgb_blue = 255;

// pins used for the rgb led (PWM)
const PROGMEM uint8_t RGB_LIGHT_RED_PIN = 5;
const PROGMEM uint8_t RGB_LIGHT_GREEN_PIN = 0;
const PROGMEM uint8_t RGB_LIGHT_BLUE_PIN = 4;

// buffer used to send/receive data with MQTT
//buffer (tampon saha), verilerin I/O işlemlerinden sonra belleğe yazılmadan önce uğradıkları bir sahadır. 
const uint8_t MSG_BUFFER_SIZE = 20;
char m_msg_buffer[MSG_BUFFER_SIZE]; 


//PIR Sensor Kod Başlangıcı
const PROGMEM char* MQTT_MOTION_STATUS_TOPIC = "office/motion/status";

// default payload
const PROGMEM char* MOTION_ON = "ON";
const PROGMEM char* MOTION_OFF = "OFF";

// PIR : D4/GPIO2
const PROGMEM uint8_t PIR_PIN = 2;
uint8_t m_pir_state = LOW; // no motion detected
uint8_t m_pir_value = 0;


//Pthotocell Sensor Başlangıcı
// MQTT: topic
const PROGMEM char* MQTT_SENSOR_TOPIC = "office/sensor3";

// sleeping time
const PROGMEM uint16_t SLEEPING_TIME_IN_SECONDS = 600; // 10 minutes x 60 seconds

// Photocell: A0 
const PROGMEM uint8_t PHOTOCELL_PIN = 0;

//DHT11 Sensor başlangıcı
//DHT11 topic
const PROGMEM char* DHT_MQTT_SENSOR_TOPIC = "office/sensor1";
// DHT - D5/GPIO14
#define DHTPIN 14
#define DHTTYPE DHT11

const int buzzer = 13; 

DHT dht(DHTPIN, DHTTYPE);

WiFiClient wifiClient;
PubSubClient client(wifiClient);

// function called to adapt the brightness and the color of the led
void setColor(uint8_t p_red, uint8_t p_green, uint8_t p_blue) {
  // convert the brightness in % (0-100%) into a digital value (0-255)
  uint8_t brightness = map(m_rgb_brightness, 0, 100, 0, 255);

  analogWrite(RGB_LIGHT_RED_PIN, map(p_red, 0, 255, 0, brightness));
  analogWrite(RGB_LIGHT_GREEN_PIN, map(p_green, 0, 255, 0, brightness));
  analogWrite(RGB_LIGHT_BLUE_PIN, map(p_blue, 0, 255, 0, brightness));
}

// function called to publish the state of the led (on/off)
void publishRGBState() {
  if (m_rgb_state) {
    client.publish(MQTT_LIGHT_STATE_TOPIC, LIGHT_ON, true);
  } else {
    client.publish(MQTT_LIGHT_STATE_TOPIC, LIGHT_OFF, true);
  }
}

// function called to publish the brightness of the led (0-100)
void publishRGBBrightness() {
  snprintf(m_msg_buffer, MSG_BUFFER_SIZE, "%d", m_rgb_brightness);
  client.publish(MQTT_LIGHT_BRIGHTNESS_STATE_TOPIC, m_msg_buffer, true);
}

// function called to publish the colors of the led (xx(x),xx(x),xx(x))
void publishRGBColor() {
  snprintf(m_msg_buffer, MSG_BUFFER_SIZE, "%d,%d,%d", m_rgb_red, m_rgb_green, m_rgb_blue);
  client.publish(MQTT_LIGHT_RGB_STATE_TOPIC, m_msg_buffer, true);
}

// function called when a MQTT message arrived
void callback(char* p_topic, byte* p_payload, unsigned int p_length) {
  // concat the payload into a string
  String payload;
  for (uint8_t i = 0; i < p_length; i++) {
    payload.concat((char)p_payload[i]);
  }
  // handle message topic
  if (String(MQTT_LIGHT_COMMAND_TOPIC).equals(p_topic)) {
    // test if the payload is equal to "ON" or "OFF"
    if (payload.equals(String(LIGHT_ON))) {
      if (m_rgb_state != true) {
        m_rgb_state = true;
        setColor(m_rgb_red, m_rgb_green, m_rgb_blue);
        publishRGBState();
      }
    } else if (payload.equals(String(LIGHT_OFF))) {
      if (m_rgb_state != false) {
        m_rgb_state = false;
        setColor(0, 0, 0);
        publishRGBState();
      }
    }
  } else if (String(MQTT_LIGHT_BRIGHTNESS_COMMAND_TOPIC).equals(p_topic)) {
    uint8_t brightness = payload.toInt();
    if (brightness < 0 || brightness > 100) {
      // do nothing...
      return;
    } else {
      m_rgb_brightness = brightness;
      setColor(m_rgb_red, m_rgb_green, m_rgb_blue);
      publishRGBBrightness();
    }
  } else if (String(MQTT_LIGHT_RGB_COMMAND_TOPIC).equals(p_topic)) {
    // get the position of the first and second commas
    uint8_t firstIndex = payload.indexOf(',');
    uint8_t lastIndex = payload.lastIndexOf(',');
    
    uint8_t rgb_red = payload.substring(0, firstIndex).toInt();
    if (rgb_red < 0 || rgb_red > 255) {
      return;
    } else {
      m_rgb_red = rgb_red;
    }
    
    uint8_t rgb_green = payload.substring(firstIndex + 1, lastIndex).toInt();
    if (rgb_green < 0 || rgb_green > 255) {
      return;
    } else {
      m_rgb_green = rgb_green;
    }
    
    uint8_t rgb_blue = payload.substring(lastIndex + 1).toInt();
    if (rgb_blue < 0 || rgb_blue > 255) {
      return;
    } else {
      m_rgb_blue = rgb_blue;
    }
    
    setColor(m_rgb_red, m_rgb_green, m_rgb_blue);
    publishRGBColor();
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("BILGI: MQTT baglantisi baslatiliyor...");
    // Attempt to connect
    if (client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("BILGI: baglandi");
      
      // Once connected, publish an announcement...
      // publish the initial values
      publishRGBState();
      publishRGBBrightness();
      publishRGBColor();

      // ... and resubscribe
      client.subscribe(MQTT_LIGHT_COMMAND_TOPIC);
      client.subscribe(MQTT_LIGHT_BRIGHTNESS_COMMAND_TOPIC);
      client.subscribe(MQTT_LIGHT_RGB_COMMAND_TOPIC);
    } else {
      Serial.print("ERROR: hata, rc=");
      Serial.print(client.state());
      Serial.println("DEBUG: baglanti tekrar deneniyor 5 saniye icinde");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


// pir sensor fonsiyon başlangıcı
void publishPirSensorState() {
  if (m_pir_state) {
    client.publish(MQTT_MOTION_STATUS_TOPIC, MOTION_OFF, true);
  } else {
    client.publish(MQTT_MOTION_STATUS_TOPIC, MOTION_ON, true);
  }
}

//Photocell fonsiyon başlangıç
void publishData(int p_analogRead) {
  // convert 0-1024 into a percentage
  uint8_t brightness = map(p_analogRead, 0, 1024, 0, 100);
  
  //  Yeni JSON objesi oluştur 
  //Json, Javascript uygulamaları için oluşturulmuş bir veri formatıdır
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  root["brightness"] = (String)brightness;
  root.prettyPrintTo(Serial);
  Serial.println("");
  /*
  {
    "brightness":  "75"
  }
  */
  char data[200];
  root.printTo(data, root.measureLength() + 1);
  client.publish(MQTT_SENSOR_TOPIC, data);
}

// Sıcaklık ve nem fonsiyon çağır ve data paylaş
void publishDataDHT11(float p_temperature, float p_humidity) {
  // Yeni JSON objesi oluştur  
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  // INFO: the data must be converted into a string; a problem occurs when using floats...
  root["temperature"] = (String)p_temperature;
  root["humidity"] = (String)p_humidity;
  root.prettyPrintTo(Serial);
  Serial.println("");
  /*
     {
        "temperature": "23.20" ,
        "humidity": "43.70"
     }
  */
  char data[200];
  root.printTo(data, root.measureLength() + 1);
  client.publish(DHT_MQTT_SENSOR_TOPIC, data, true);
}


void setup() {
  // init the serial
  Serial.begin(115200);

  // init the RGB led
  pinMode(RGB_LIGHT_BLUE_PIN, OUTPUT);
  pinMode(RGB_LIGHT_RED_PIN, OUTPUT);
  pinMode(RGB_LIGHT_GREEN_PIN, OUTPUT);
  analogWriteRange(255);
  setColor(0, 0, 0);
  
  //buzzer pini
 pinMode(buzzer, OUTPUT); // Set buzzer - pin 13
 
  // init the WiFi connection
  Serial.println();
  Serial.println();
  Serial.print("BiLGi: Connecting to ");
  WiFi.mode(WIFI_STA);
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("BiLGi: WiFi connected");
  Serial.print("BiLGi: IP address: ");
  Serial.println(WiFi.localIP());

  // init the MQTT connection
  client.setServer(MQTT_SERVER_IP, MQTT_SERVER_PORT);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();


  // DEğer oku  PIR sensor
  m_pir_value = digitalRead(PIR_PIN);
  if (m_pir_value == HIGH) {
    if (m_pir_state == LOW) {
      //Hareket tespit edildiginde
      Serial.println("BiLGi: Hareket Algilandi");
      publishPirSensorState();
      m_pir_state = HIGH;
      tone(buzzer, 1000); // Send 1KHz sound signal...
      delay(1000);        // ...for 1 sec
      noTone(buzzer);
      delay(1000);
    }
  } else {
    if (m_pir_state == HIGH) {
      publishPirSensorState();
      Serial.println("BiLGi: Hareket sona erdi");
      m_pir_state = LOW;
    }
  }


  //Photocell loop başlangıç
  uint16_t photocell = analogRead(PHOTOCELL_PIN);

  if (photocell < 0 || photocell > 1024) {
    Serial.println("HATA: Hatalı veri photocell!");
    return;
  } else {

    publishData(photocell);
    delay(5000);
    
  }

// Sıcaklık veya nemi okumak yaklaşık 250 milisaniye alır!
// Sensör okumaları aynı zamanda 2 saniyeye kadar eski olabilir (çok yavaş bir sensör)

  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    tone(buzzer, 1000); // Send 1KHz sound signal...
    delay(500);        // ...for 1 sec
    noTone(buzzer);
    delay(500); // Stop sound...
    tone(buzzer, 1000); // Send 1KHz sound signal...
    delay(500);        // ...for 1 sec
    noTone(buzzer);
    delay(500); // Stop sound...
    Serial.println("HATA: DHT sensor okuma yapilmadi!");
    return;
  } else {
    //Serial.println(h);
    //Serial.println(t);
    publishDataDHT11(t, h);
  }

  //Serial.println("INFO: Closing the MQTT connection");
  //client.disconnect();

 // Serial.println("INFO: Closing the Wifi connection");
  //WiFi.disconnect();

  //ESP.deepSleep(SLEEPING_TIME_IN_SECONDS * 1000000, WAKE_RF_DEFAULT);
  
}
