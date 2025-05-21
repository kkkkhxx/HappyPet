#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

// === CONFIG ====================
const char* ssid = "ig : heang_kung";
const char* password = "hm31@12@@5";

const char* mqtt_server = "192.168.26.8";
const int mqtt_port = 1883;
const char* mqtt_topic = "esp32/motion";
const char* mqtt_servo_topic = "esp32/servo";
const char* mqtt_control_topic = "esp32/control";
// ==============================

const int PIR_PIN = 35;
const int LED_PIN = 33;
const int SERVO_PIN = 32;
const int BUTTON_PIN = 4;  // ปุ่มจริงที่ต่อ GPIO 4

WiFiClient espClient;
PubSubClient client(espClient);
Servo myServo;

bool enabled = false;
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  if (String(topic) == mqtt_control_topic) {
    if (message == "1") {
      enabled = true;
      Serial.println("✅ ระบบเปิดจาก MQTT");
    } else {
      enabled = false;
      Serial.println("❌ ระบบปิดจาก MQTT");
      myServo.write(0);
      digitalWrite(LED_PIN, LOW);
    }
  }
}

void setup_wifi() {
  delay(10);
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected. IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.subscribe(mqtt_control_topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  myServo.setPeriodHertz(50);
  myServo.attach(SERVO_PIN, 500, 2500);
  myServo.write(0);
  digitalWrite(LED_PIN, LOW);

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.subscribe(mqtt_control_topic);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // 🔘 ปุ่ม toggle เปิด/ปิดระบบ
  int reading = digitalRead(BUTTON_PIN);
  if ((reading != lastButtonState) && (millis() - lastDebounceTime > debounceDelay)) {
    lastDebounceTime = millis();
    if (reading == LOW) {
      enabled = !enabled;
      Serial.print("🔘 ปุ่มถูกกด → ");
      Serial.println(enabled ? "เปิดระบบ" : "ปิดระบบ");

      if (!enabled) {
        // ระบบปิด → ปิด LED/Servo และไม่ตอบสนอง PIR
        myServo.write(0);
        digitalWrite(LED_PIN, LOW);
        Serial.println("🛑 ระบบปิด → IGNORE PIR");
      }
    }
  }
  lastButtonState = reading;

  // ตรวจจับ PIR เสมอ แต่จะใช้ผลเฉพาะตอน enabled เท่านั้น
  int pirState = digitalRead(PIR_PIN);

  if (enabled) {
    if (pirState == HIGH) {
      Serial.println("🚶 ตรวจพบการเคลื่อนไหว → Servo = 90");
      myServo.write(90);
      digitalWrite(LED_PIN, HIGH);
      client.publish(mqtt_topic, "1");
      client.publish(mqtt_servo_topic, "90");
    } else {
      Serial.println("🛑 ไม่มีการเคลื่อนไหว → Servo = 0");
      myServo.write(0);
      digitalWrite(LED_PIN, LOW);
      client.publish(mqtt_topic, "0");
      client.publish(mqtt_servo_topic, "0");
    }
  } else {
    // ระบบปิด: ไม่ควรทำอะไรแม้ PIR จะแจ้ง HIGH
    // (ถ้ามีสิ่งใดควบคุมเพิ่มเมื่อปิดก็ใส่ตรงนี้)
  }

  delay(1000);
}
