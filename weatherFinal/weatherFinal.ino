#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <DHT.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>

// BMP280 Setup
Adafruit_BMP280 bmp; // I2C interface by default

// DHT11 Setup
#define DHTPIN 6          // DHT11 data pin connected to digital pin 6
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Rain Sensor Setup
#define RAIN_SENSOR_PIN 4 // Rain sensor connected to digital pin 4

// Wind Speed Sensor Setup ADVICE (use a proper anenometer with built in wind sensor)
#define HALL_SENSOR_PIN A0 // Analog pin connected to Hall sensor
int pulseCount = 0;         // Count of detected pulses
unsigned long lastPulseTime = 0; // Last time a pulse was counted
const float circumference = 0.3;    // Circumference of the rotation (meters) Changes this by the size of your anenometer
bool lastWindState = HIGH;          // Previous sensor state

// SIM900 Setup
SoftwareSerial SIM900(7, 8); // TX, RX

// LCD I2C Setup on default I2C pins (SDA A4 and SCL A5)
LiquidCrystal_I2C lcd(0x27, 16, 2); // Set the I2C address to 0x27 for a 16x2 LCD

// Authorized phone number
const String authorizedPhoneNumber = "+639556680399"; //this my no. change it to your own number

// Timer variables for periodic SMS
unsigned long lastSentTime = 0; // Last time SMS was sent
const unsigned long interval = 60000; // 1 minute in milliseconds

// Rain detection variables
bool previousRainState = false; // Track the previous state of rain detection

void setup() {
  // Initialize sensors, LCD, and Serial
  Serial.begin(9600);
  SIM900.begin(9600);
  dht.begin();
  pinMode(RAIN_SENSOR_PIN, INPUT);
  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);

  if (!bmp.begin(0x76)) {  // Try 0x77 if 0x76 doesn't work
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }

  lcd.begin(16, 2);        
  lcd.backlight();         

  SIM900.println("AT+CMGF=1");  
  delay(1000);
  SIM900.println("AT+CNMI=1,2,0,0,0");  
  delay(1000);

  Serial.println("System initialized. Waiting for SMS command 'SEND'...");
}

void loop() {
  // Display sensor data
  displaySensorData();

  // Check for incoming SMS
  if (SIM900.available()) {
    String smsContent = SIM900.readString();
    if (smsContent.indexOf(authorizedPhoneNumber) >= 0 && smsContent.indexOf("SEND") >= 0) {
      sendSensorDataSMS();
    } else {
      Serial.println("Unauthorized number or incorrect command. Ignoring SMS.");
    }
  }

  // Send periodic SMS every minute
  sendPeriodicSMS();

  // Monitor rain and send alert if rain is detected
  monitorRainSensor();
}

void displaySensorData() {
  // Read BMP280 data
  float pressure = bmp.readPressure() / 100.0F; // Convert Pa to hPa

  // Read DHT11 data
  float temperatureDHT = dht.readTemperature();
  float humidity = dht.readHumidity();

  // Read rain status
  bool isRaining = digitalRead(RAIN_SENSOR_PIN) == LOW;

  // Calculate wind speed based on pulses
  bool currentWindState = digitalRead(HALL_SENSOR_PIN);
  if (currentWindState == LOW && lastWindState == HIGH) { // Detect falling edge
    pulseCount++;
    lastPulseTime = millis(); // Record the time of the last pulse
  }
  lastWindState = currentWindState;

  // Calculate wind speed in real-time
  float rotations = pulseCount / 2.0; // Two pulses per rotation
  float windSpeedMps = (rotations * circumference) / ((millis() - lastPulseTime) / 1000.0); // m/s
  float windSpeedKmh = windSpeedMps * 3.6; // Convert to km/h

  // Display on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(temperatureDHT);
  lcd.print("C H:");
  lcd.print(humidity);
  lcd.print("%");

  lcd.setCursor(0, 1);
  lcd.print("P:");
  lcd.print(pressure);
  lcd.print("hPa");
  delay(1000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Rain:");
  lcd.print(isRaining ? "Yes" : "No");

  lcd.setCursor(0, 1);
  lcd.print("Wind:");
  lcd.print(windSpeedKmh, 2); // Display wind speed with 2 decimal places
  lcd.print("km/h");
  delay(1000);
}

String rainPrediction(float temperature, float humidity, float pressure) {
  if (pressure > 1015 && humidity < 40 && temperature > 25) {
    return "Never (0%)";
  } else if (pressure >= 1005 && pressure <= 1015 && humidity >= 40 && humidity < 60 && temperature >= 15 && temperature <= 25) {
    return "Slight (Up to 25%)";
  } else if (pressure >= 995 && pressure < 1005 && humidity >= 60 && humidity < 80 && temperature >= 10 && temperature <= 20) {
    return "Moderate (50%)";
  } else if (pressure < 995 && humidity > 80 && temperature < 15) {
    return "Highly Likely (75%+)";
  } else {
    return "No reliable prediction";
  }
}

void sendSensorDataSMS() {
  // Read data from all sensors
  float pressure = bmp.readPressure() / 100.0F; // Convert Pa to hPa
  float temperatureDHT = dht.readTemperature();
  float humidity = dht.readHumidity();
  bool isRaining = digitalRead(RAIN_SENSOR_PIN) == LOW; // Assume LOW means rain detected

  // Calculate wind speed for SMS
  float rotations = pulseCount / 2.0;
  float windSpeedMps = (rotations * circumference) / ((millis() - lastPulseTime) / 1000.0);
  float windSpeedKmh = windSpeedMps * 3.6;

  // Generate rain prediction based on temperature, humidity, and pressure
  String prediction = rainPrediction(temperatureDHT, humidity, pressure);

  // Prepare the message for SMS
  String message = "T: " + String(temperatureDHT) + "C, H: " + String(humidity) + "%\n";
  message += "P: " + String(pressure) + "hPa\n";
  message += String("Rain: ") + (isRaining ? "Yes" : "No") + "\n";
  message += "Wind: " + String(windSpeedKmh, 2) + "km/h\n";
  message += "Prediction: " + prediction;

  // Send the message
  sendSMS(authorizedPhoneNumber, message);
}

void sendPeriodicSMS() {
  unsigned long currentTime = millis();

  // Check if one minute has passed
  if (currentTime - lastSentTime >= interval) {
    Serial.println("One minute elapsed. Sending SMS...");
    sendSensorDataSMS();
    lastSentTime = currentTime;
  }
}

void monitorRainSensor() {
  bool isRaining = digitalRead(RAIN_SENSOR_PIN) == LOW; // Assume LOW means rain detected

  // Send SMS only if rain is detected and the previous state was no rain
  if (isRaining && !previousRainState) {
    Serial.println("Rain detected. Sending SMS...");
    sendSMS(authorizedPhoneNumber, "It is raining.");
  }

  // Update the previous rain state
  previousRainState = isRaining;
}

void sendSMS(String phoneNumber, String message) {
  // Set SMS mode to text
  SIM900.println("AT+CMGF=1");  
  delay(1000);

  // Specify the recipient phone number
  SIM900.print("AT+CMGS=\"");
  SIM900.print(phoneNumber);
  SIM900.println("\"");
  delay(1000);

  // Send the actual message content
  SIM900.println(message);
  delay(1000);

  // Send Ctrl+Z (ASCII 26) to mark the end of the message
  SIM900.write(26);  
  delay(1000);

  // Print confirmation to Serial Monitor
  Serial.println("SMS Sent to " + phoneNumber + ": " + message);
}
