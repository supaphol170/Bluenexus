//Blynk setup
#define BLYNK_TEMPLATE_ID "TMPL69SlTUAKQ"
#define BLYNK_TEMPLATE_NAME "BLUENEXUS"
#define BLYNK_AUTH_TOKEN "-hCvApPWonTL2DkimXWCIozFkKvJSeUZ" //This token for compatible between esp32 and blynk

//Import module
#include <WiFi.h> //This module compatible between esp32 and wifi.
#include <OneWire.h> //This module is a software library used to handles the low-level communication on the shared data line.
#include <DallasTemperature.h> //This module is a software library used to read data from Dallas-family temperature sensor (like the DS18B20) from RAW to usable data.
#include "HUSKYLENS.h" //This module is a software library use with huskylens.
#include <Wire.h>
#include <BlynkSimpleEsp32.h>

//declaration variables zones
//-----------------------------------------------------------
//----- husky lens -----
HUSKYLENS huskylens;
void printResult(HUSKYLENSResult result);

//----- PH sensor -----
#define PHSENSORPIN 33
float slope = -0.0047; // m in formula "y=mx+c"
float offset = 14.07; // c in formula "y=mx+c"
long phTot;
float phAvg;
int x;
long phsensorValue = 0;

//----- TDS Sensor -----
#define TdsSensorPin 34 //Set pins for read data
#define VREF 3.3 //Setting voltage to use 
#define SCOUNT 30 //for how many number you want to keep before find median
int analogBuffer[SCOUNT];
float tdsValue = 0;
float temperature = 25;  // เริ่มต้น, ใช้ค่า DS18B20 อัปเดต

//----- Turbidity Sensor -----
#define TurbidityPin 32
#define TURB_MEDIAN_SAMPLES 15
int turbidityBuffer[TURB_MEDIAN_SAMPLES];

//----- DS18B20 Temp Sensor ----- 
#define ONE_WIRE_BUS 16
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

//----- Wi-Fi -----
char ssid[50] = "@KP-WiFi-RoboticsKoi";
char password[50] = "koi05032544";
//-----------------------------------------------------------

// -------------------- connecting wifi function --------------------
void connectWiFi() {
  Serial.print("Connecting Wi-Fi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  int retryCount = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (++retryCount > 30) { //try to connect 30 times
      Serial.println("\nเชื่อมต่อไม่สำเร็จ รีสตาร์ท...");
      ESP.restart();
    }
  }
  Serial.println("\nเชื่อมต่อสำเร็จ!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  Blynk.virtualWrite(V7, ssid);
}

// This function will be called every time data is sent to Virtual Pin V1
// -------------------- receiving data from Blynk --------------------
BLYNK_WRITE(V6) {
  String wifi = param.asStr(); //change incoming data format to String format.
  int firstCommaIndex = wifi.indexOf(','); //using comma(,) for seperate string income to ssid and password
  if (firstCommaIndex > 0) {
    String new_ssid = wifi.substring(0, firstCommaIndex); //by substring function have systax "substring(from, to)"
    String new_password = wifi.substring(firstCommaIndex + 1);
    // Copy String → char array (ESP32 WiFi.begin requires char* not String)
    static char ssid_buf[50];
    static char pass_buf[50];
    new_ssid.toCharArray(ssid_buf, sizeof(ssid_buf));
    new_password.toCharArray(pass_buf, sizeof(pass_buf));
    strcpy(ssid, ssid_buf);
    strcpy(password, pass_buf);
    // Reconnect Wi-Fi
    WiFi.disconnect();
    connectWiFi();
  }
  else {
    Serial.println("Invalid Wi-Fi format. Use: ssid,password");
  }
}

// Called once Blynk is connected
BLYNK_CONNECTED() {
  Blynk.virtualWrite(V7, ssid);
  Blynk.virtualWrite(V6, WiFi.localIP().toString());
}

// -------------------- Husky lens function--------------------
void printResult(HUSKYLENSResult result) {
    if (result.command == COMMAND_RETURN_BLOCK) {
        Serial.println(String() + F("Block:xCenter=") + result.xCenter + F(",yCenter=") + result.yCenter + F(",width=") + result.width + F(",height=") + result.height + F(",ID=") + result.ID);
        if (result.ID == 1) {
          Serial.println("1111111111111111");
        }
        if (result.ID == 2) {
          Serial.println("2222222222222222");
        }
        if (result.ID == 3) {
          Serial.println("333333333333");
        }
        
    
    } else {
        Serial.println("Object unknown!");
    }
}

// -------------------- Median filter for arrays --------------------
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (int i = 0; i < iFilterLen; i++) bTab[i] = bArray[i];

  for (int j = 0; j < iFilterLen - 1; j++) {
    for (int i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        int temp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = temp;
      }
    }
  }

  if ((iFilterLen & 1) > 0)
    return bTab[(iFilterLen - 1) / 2];
  else
    return (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
}

// -------------------- PH Sensors funstion--------------------
float PHsensors(int pin) {
  phTot = 0;
  for (x = 0; x < 10; x++) {
    phTot += analogRead(pin);
    delay(10);
  }
  phAvg = phTot / 10.0;
  float calibratedPH = (slope * phAvg) + offset;
  // Limit pH in range 0–14
  if (calibratedPH < 0) calibratedPH = 0;
  else if (calibratedPH > 14) calibratedPH = 14;
  return calibratedPH;
}

// -------------------- Turbidity Median Filter --------------------
int readTurbidityMedian(int pin) {
  for (int i = 0; i < TURB_MEDIAN_SAMPLES; i++) {
    turbidityBuffer[i] = analogRead(pin);
    delay(10);
  }
  return getMedianNum(turbidityBuffer, TURB_MEDIAN_SAMPLES);
}

// -------------------- Convert ADC to Turbidity Percent --------------------
float convertTurbidityToPercent(int adcValue) {
  const int MIN_ADC = 2900;  // ปรับตามการทดลองจริง
  const int MAX_ADC = 3200;  // ปรับตามการทดลองจริง
  float percent = (float)(adcValue - MIN_ADC) * 100.0 / (MAX_ADC - MIN_ADC);
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  return percent;
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200); //Setting BuadRate
  connectWiFi(); //Connecting Wi-fi
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password); //Authentication Blynk
  // Set 3 images at setup
  Blynk.setProperty(V5, "urls",
    "https://github.com/supaphol170/Bluenexus/raw/a22d7cd22f184f1dcf55012b4151477ce2803139/Low_level_water.jpg",
    "https://github.com/supaphol170/Bluenexus/raw/a22d7cd22f184f1dcf55012b4151477ce2803139/Middle_level_water.jpg",
    "https://github.com/supaphol170/Bluenexus/raw/a22d7cd22f184f1dcf55012b4151477ce2803139/High_level_water.jpg"
  );

  // husky lens setup
  Wire.begin(21, 22);  // กำหนด SDA=21, SCL=22 ตามการต่อจริงของ ESP32
  while (!huskylens.begin(Wire)) {
      Serial.println(F("Begin failed!"));
      delay(1000);
  }

  // TDS setup
  pinMode(TdsSensorPin, INPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // DS18B20 setup
  sensors.begin();

  //PH sensor setup
  Serial.println("Calibrated ESP32 pH sensor");
  Serial.print("Slope: "); Serial.println(slope, 5);
  Serial.print("Offset: "); Serial.println(offset, 5);

  Serial.println("System ready");
}

// -------------------- Loop --------------------
void loop() {
  static unsigned long lastSensorRead = millis();
  Blynk.run();
  if (millis() - lastSensorRead > 40U) {
    lastSensorRead = millis();
    // ---- อุณหภูมิ ----
    sensors.requestTemperatures();
    temperature = sensors.getTempCByIndex(0);

    // ---- อ่าน TDS ----
    for (int i = 0; i < SCOUNT; i++) {
      analogBuffer[i] = analogRead(TdsSensorPin);
      delay(40);
    }
    // calculate median and TDS
    int rawValue = getMedianNum(analogBuffer, SCOUNT);
    float averageVoltage = rawValue * (float)VREF / 4095.0;
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
    float compensationVoltage = averageVoltage / compensationCoefficient;
    tdsValue = (133.42 * pow(compensationVoltage, 3)
              - 255.86 * pow(compensationVoltage, 2)
              + 857.39 * compensationVoltage) * 0.5;
    // Clamp value to 0 (if voltage is too low)
    if (tdsValue < 0) tdsValue = 0;

    // ---- อ่าน Turbidity ---- 
    // You don't worry if value in Blynk more than 3000 cuz Turbidity sensor not dip into water.
    int rawTurbidityADC = readTurbidityMedian(TurbidityPin);
    float turbidityPercent = convertTurbidityToPercent(rawTurbidityADC);

    // ---- PH sensor ----
    float phsensoroutcome = PHsensors(PHSENSORPIN);

    // ---- Husky lens ----
    int lastWaterLevelID = -1;
    if (huskylens.request()) {
      while (huskylens.available()) {
        HUSKYLENSResult result = huskylens.read();
        if (result.command == COMMAND_RETURN_BLOCK && result.ID != lastWaterLevelID) {
          lastWaterLevelID = result.ID;
          if (result.ID == 1) {
            //#FBFF03 - Blynk YELLOW 
            Blynk.setProperty(V4, "color", "#FBFF03");
            Blynk.virtualWrite(V4, "Low Level");
            for (int i = 0; i < 29; i++) {
              Blynk.setProperty(V4, "isHidden", true);
              delay(1000);
              Blynk.setProperty(V4, "isHidden", false);
              delay(1000);
            }
            Blynk.virtualWrite(V5, 0);
          } else if (result.ID == 2) {
            //2CED02 - Blynk GREEN 
            Blynk.setProperty(V4, "color", "#2CED02");
            Blynk.virtualWrite(V4, "Normal");
            for (int i = 0; i < 29; i++) {
              Blynk.setProperty(V4, "isHidden", true);
              delay(1000);
              Blynk.setProperty(V4, "isHidden", false);
              delay(1000);
            }
            Blynk.virtualWrite(V5, 1);
          } else if (result.ID == 3) {
            //#D3435C - Blynk RED
            Blynk.setProperty(V4, "color", "#D3435C");
            Blynk.virtualWrite(V4, "Critical");
            for (int i = 0; i < 29; i++) {
              Blynk.setProperty(V4, "isHidden", true);
              delay(1000);
              Blynk.setProperty(V4, "isHidden", false);
              delay(1000);
            }
            Blynk.virtualWrite(V5, 2);
          }
        }
      }
    }

    // ---- แสดงผลทั้งหมด ----
    Serial.println("---------");
    Serial.print("Temperature (C): ");
    Serial.println(temperature, 2);
    Blynk.virtualWrite(V3, temperature);

    Serial.print("Turbidity ADC: ");
    Serial.print(rawTurbidityADC);
    Serial.print(" -> Turbidity: ");
    Serial.print(turbidityPercent, 1);
    Serial.println(" %");
    Blynk.virtualWrite(V0, turbidityPercent);

    Serial.print("TDS (ppm): ");
    Serial.println(tdsValue, 2);
    Blynk.virtualWrite(V2, tdsValue);

    Serial.print("PH Sensor: ");
    Serial.println(phsensoroutcome, 2);
    Blynk.virtualWrite(V1, phsensoroutcome);
  }
  delay(30000);
}
