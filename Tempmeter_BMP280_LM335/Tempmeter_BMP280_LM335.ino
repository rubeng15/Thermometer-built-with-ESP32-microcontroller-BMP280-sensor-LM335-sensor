#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <LiquidCrystal_I2C.h>

#define BMP_SDA 21  // SDA-pin for BMP280
#define BMP_SCL 22  // SCL-pin for BMP280

Adafruit_BMP280 bmp; // I2C
#define ADC_VREF_mV    3300.0 // in millivolt
#define ADC_RESOLUTION 4096.0
#define PIN_LM35       36 // ESP32 pin GPIO36 (ADC0) connected to LM35

LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16 columns, 2 rows

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(100); // wait for native USB
  Serial.println(F("Temperature Sensor Test"));

  Wire.begin(BMP_SDA, BMP_SCL);
  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x");
    Serial.println(bmp.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1)
      delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Temperature:");
}

void loop() {
  // BMP280 temperature reading
  float bmpTemp = bmp.readTemperature();
  Serial.print(F("BMP280 Temperature = "));
  Serial.print(bmpTemp);
  Serial.println(" °C");

  // LM35 temperature reading
  int adcVal = analogRead(PIN_LM35);
  float milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION);
  float tempC = milliVolt / 10;

  Serial.print(F("LM35 Temperature = "));
  Serial.print(tempC);
  Serial.println(" °C");

  // Display temperatures on LCD
  lcd.setCursor(0, 1);
  lcd.print("BMP: ");
  lcd.print(bmpTemp);
  lcd.print("C LM35: ");
  lcd.print(tempC);
  lcd.print("C    ");

  delay(2000);
}
