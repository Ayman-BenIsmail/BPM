#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

const char *ssid = "O-TURKI";      // "REPLACE_WITH_YOUR_SSID"
const char *password = "01234567"; // "REPLACE_WITH_YOUR_PASSWORD"

StaticJsonDocument<16> doc;
String requestJSON = "";
String response = "";
String endPoint = "http://192.168.137.1/iomt_lab/";
//  Variables
unsigned long previousMillisGetHR = 0;
unsigned long previousMillisHR = 0;

const long intervalGetHR = 20;
const long intervalHR = 10000;

int PulseSensorHRWire = 32; // Pulse Sensor WIRE connected to ANALOG PIN 32
int LED = LED_BUILTIN;      //  The on-board ESP32 LED (=2)

// int Signal;           // holds the incoming raw data. Signal value can range from 0-4095
int Threshold = 3034; // Determine which Signal to "count as a beat", and which to ingore.

int cntHB = 0;
boolean ThresholdStat = true;
int BPMval = 0;

void GetHeartRate();
String httpPOSTRequest(String endPoint, String postData);

void setup()
{
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print('.');
    delay(1000);
  }

  Serial.print("\nESP32 Local IP address: ");
  Serial.println(WiFi.localIP());

  pinMode(LED, OUTPUT); // pin that will blink to your heartbeat!

  Serial.println();
  Serial.println("Please wait 10 seconds to get the BPM Value");
}

void loop()
{
  GetHeartRate();
  // int PulseSensorHRVal = analogRead(PulseSensorHRWire);
  // Serial.println(PulseSensorHRVal);
  // delay(20);
}

void GetHeartRate()
{
  // Process of reading heart rate.
  unsigned long currentMillisGetHR = millis();

  if (currentMillisGetHR - previousMillisGetHR >= intervalGetHR)
  {
    previousMillisGetHR = currentMillisGetHR;

    int PulseSensorHRVal = analogRead(PulseSensorHRWire);

    if (PulseSensorHRVal > Threshold && ThresholdStat == true)
    {
      cntHB++;
      ThresholdStat = false;
      digitalWrite(LED, HIGH);
    }

    if (PulseSensorHRVal < Threshold)
    {
      ThresholdStat = true;
      digitalWrite(LED, LOW);
    }
  }

  // The process for getting the BPM value.
  unsigned long currentMillisHR = millis();

  if (currentMillisHR - previousMillisHR >= intervalHR)
  {
    previousMillisHR = currentMillisHR;

    BPMval = cntHB * 6;
    Serial.print("BPM : ");
    Serial.println(BPMval);

    doc["BPM"] = BPMval;
    requestJSON = "";
    serializeJson(doc, requestJSON);
    response = httpPOSTRequest(endPoint, requestJSON);
    Serial.println(response);

    cntHB = 0;
  }
}

String httpPOSTRequest(String endPoint, String postData)
{
  HTTPClient http;
  http.begin(endPoint);
  http.setConnectTimeout(10000);
  http.addHeader("Content-Type", "application/json");
  int httpResponseCode = http.POST(postData);
  String payload = "{\"error\": \"CONNECTION_ERROR\"}";
  if (httpResponseCode > 0)
  {
    payload = http.getString();
  }
  http.end();
  return payload;
}