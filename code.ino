#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <DHT.h>

// --- Pin Definitions ---
#define TEMP_SENSOR_PIN 14    // GPIO where DHT22 data pin is connected
#define ALCOHOL_SENSOR_PIN 35

#define ULTRASONIC_TRIG_PIN 25
#define ULTRASONIC_ECHO_PIN 26
#define TRIG2 32
#define ECHO2 33

#define VIBRATION_SENSOR_PIN 27

#define LED_TEMP 13
#define LED_ALCOHOL 12
#define LED_ULTRASONIC 14
#define BUZZER_PIN 15

// --- Threshold Limits ---
const int TEMP_LIMIT = 50;
const int ALCOHOL_LIMIT = 200;
const int DISTANCE_LIMIT = 20;
const int VIBRATION_LIMIT = 1;

// --- Timing constants ---
const unsigned long GPS_FIX_TIMEOUT = 10000;
const unsigned long SMS_COOLDOWN = 30000;

// --- GPS and GSM Setup ---
HardwareSerial gpsSerial(2);    // RX=16, TX=17
HardwareSerial gsmSerial(1);    // RX=4, TX=5

TinyGPSPlus gps;

// DHT Sensor Setup
#define DHTTYPE DHT22
DHT dht(TEMP_SENSOR_PIN, DHTTYPE);

// SMS Config
const char recipientNumber[] = "+94705126212";
bool smsSent = false;
unsigned long lastSmsTime = 0;

void setup() {
  Serial.begin(115200);

  // Setup pins
  pinMode(LED_TEMP, OUTPUT);
  pinMode(LED_ALCOHOL, OUTPUT);
  pinMode(LED_ULTRASONIC, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(VIBRATION_SENSOR_PIN, INPUT);
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);

  // Initialize DHT sensor
  dht.begin();

  // Start serial for GPS and GSM
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);
  gsmSerial.begin(57600, SERIAL_8N1, 4, 5);

  delay(2000);
  Serial.println("Setup complete");

  // Initialize GSM module
  sendATCommand("AT", 1000);
  sendATCommand("AT+CMGF=1", 1000); // Text mode
}

void loop() {
  float temperature = dht.readTemperature();  // Read temperature from DHT22
  int alcoholLevel = analogRead(ALCOHOL_SENSOR_PIN);
  float distance1 = readUltrasonicDistance(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN);
  float distance2 = readUltrasonicDistance(TRIG2, ECHO2);
  int vibration = digitalRead(VIBRATION_SENSOR_PIN);

  if (isnan(temperature)) {
    Serial.println("Failed to read temperature!");
    temperature = -1000; // Set an invalid low temperature to avoid false alerts
  }

  Serial.print("Temp: "); Serial.print(temperature); Serial.print(" C, ");
  Serial.print("Alcohol: "); Serial.print(alcoholLevel); Serial.print(", ");
  Serial.print("Distance1: "); Serial.print(distance1); Serial.print(" cm, ");
  Serial.print("Distance2: "); Serial.print(distance2); Serial.print(" cm, ");
  Serial.print("Vibration: "); Serial.println(vibration);

  bool alert = false;

  if (temperature > TEMP_LIMIT) {
    digitalWrite(LED_TEMP, HIGH);
    alert = true;
  } else {
    digitalWrite(LED_TEMP, LOW);
  }

  if (alcoholLevel > ALCOHOL_LIMIT) {
    digitalWrite(LED_ALCOHOL, HIGH);
    alert = true;
  } else {
    digitalWrite(LED_ALCOHOL, LOW);
  }

  if ((distance1 > 0 && distance1 < DISTANCE_LIMIT) || (distance2 > 0 && distance2 < DISTANCE_LIMIT)) {
    digitalWrite(LED_ULTRASONIC, HIGH);
    alert = true;
  } else {
    digitalWrite(LED_ULTRASONIC, LOW);
  }

  digitalWrite(BUZZER_PIN, alert ? HIGH : LOW);

  if (vibration == VIBRATION_LIMIT && !smsSent) {
    Serial.println("Vibration detected! Sending SMS with sensor values and GPS location...");

    double lat = 0.0, lng = 0.0;
    unsigned long start = millis();
    bool gpsFixed = false;

    while (millis() - start < GPS_FIX_TIMEOUT) {
      while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
      }
      if (gps.location.isValid()) {
        lat = gps.location.lat();
        lng = gps.location.lng();
        gpsFixed = true;
        break;
      }
    }

    if (gpsFixed) {
      sendSMSWithLocation(lat, lng, temperature, alcoholLevel, distance1, distance2, vibration);
    } else {
      Serial.println("Failed to get GPS fix.");
      char msg[200];
      snprintf(msg, sizeof(msg),
               "Alert! Vibration detected.\nTemp: %.1f C\nAlcohol: %d\nDist1: %.1f cm\nDist2: %.1f cm\nVibration: %d\nGPS fix failed.",
               temperature, alcoholLevel, distance1, distance2, vibration);
      sendSMS(msg);
    }

    smsSent = true;
    lastSmsTime = millis();
  }

  if (smsSent && (millis() - lastSmsTime > SMS_COOLDOWN)) {
    smsSent = false;
  }

  delay(1000);
}

// --- Helper Functions ---

float readUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return -1;
  float distance = (duration / 2.0) * 0.0343;
  return distance;
}

void sendATCommand(const char* cmd, unsigned long delayTime) {
  gsmSerial.println(cmd);
  delay(delayTime);
  while (gsmSerial.available()) {
    Serial.write(gsmSerial.read());
  }
}

void sendSMS(const char* message) {
  gsmSerial.println("AT"); // Check communication
  delay(1000);

  gsmSerial.println("AT+CMGF=1"); // Set SMS to text mode
  delay(1000);

  gsmSerial.print("AT+CMGS=\"");
  gsmSerial.print(recipientNumber);
  gsmSerial.println("\"");
  delay(1000);

  gsmSerial.print(message); // SMS content
  delay(500);

  gsmSerial.write(26); // CTRL+Z to send
  delay(5000);

  Serial.println("SMS sent (or in process).");
}

void sendSMSWithLocation(double lat, double lng, float temperature, int alcoholLevel, float distance1, float distance2, int vibration) {
  char msg[320];
  snprintf(msg, sizeof(msg),
           "Alert! Vibration detected.\nTemp: %.1f C\nAlcohol: %d\nDist1: %.1f cm\nDist2: %.1f cm\nVibration: %d\nGPS:\nhttps://www.google.com/maps/search/?api=1&query=%.6f,%.6f",
           temperature, alcoholLevel, distance1, distance2, vibration, lat, lng);
  sendSMS(msg);
}