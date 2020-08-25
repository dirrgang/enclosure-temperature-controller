#include <Arduino.h>
#include <Wire.h>

const int kStatPin = 13; // On-Board LED used as status indication

// LCD
#include <LiquidCrystal_I2C.h>

#define COLUMS 20
#define ROWS 2
#define PAGE ((COLUMS) * (ROWS))

#define LCD_SPACE_SYMBOL 0x20 //space symbol from the LCD ROM, see p.9 of GDM2004D datasheet

const int kLcdAddress = 0x27;
LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);

// Temperature Sensor
#include <OneWire.h>
#include <DallasTemperature.h>

const int kOneWireBusPin = 2; // Data wire of the thermometer

OneWire oneWire(kOneWireBusPin);     // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature.
DeviceAddress enclosureThermometer;  // arrays to hold device address

// Fans
const int kFanPWMPin = 5;
const int kPwmFrequency = 25000; // PWM Frequency: Target frequency 25 kHz, acceptable operational range 21 kHz to 28 kHz 
const int kPwmResolution = 8; // 2^kPwmResolution = Resolution steps. 2^8 = 255

void lcdSetup()
{
  while (lcd.begin(COLUMS, ROWS) != 1) //colums - 20, rows - 4
  {
    Serial.println(F("PCF8574 is not connected or lcd pins declaration is wrong. Only pins numbers: 4,5,6,16,11,12,13,14 are legal."));
    delay(5000);
  }

  lcd.print(F("PCF8574 is OK...")); //(F()) saves string to flash & keeps dynamic memory free
  delay(2000);

  lcd.clear();
}

void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16)
      Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

void tempSensorSetup()
{
  // start serial port
  Serial.println("Dallas Temperature IC Control Library Demo");

  // locate devices on the bus
  Serial.print("Locating devices...");
  sensors.begin();
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: ");
  if (sensors.isParasitePowerMode())
    Serial.println("ON");
  else
    Serial.println("OFF");

  // Assign address manually. The addresses below will need to be changed
  // to valid device addresses on your bus. Device address can be retrieved
  // by using either oneWire.search(deviceAddress) or individually via
  // sensors.getAddress(deviceAddress, index)
  // Note that you will need to use your specific address here
  //insideThermometer = { 0x28, 0x1D, 0x39, 0x31, 0x2, 0x0, 0x0, 0xF0 };

  // Method 1:
  // Search for devices on the bus and assign based on an index. Ideally,
  // you would do this to initially discover addresses on the bus and then
  // use those addresses and manually assign them (see above) once you know
  // the devices on your bus (and assuming they don't change).
  if (!sensors.getAddress(enclosureThermometer, 0))
    Serial.println("Unable to find address for Device 0");

  // method 2: search()
  // search() looks for the next device. Returns 1 if a new address has been
  // returned. A zero might mean that the bus is shorted, there are no devices,
  // or you have already retrieved all of them. It might be a good idea to
  // check the CRC to make sure you didn't get garbage. The order is
  // deterministic. You will always get the same devices in the same order
  //
  // Must be called before search()
  //oneWire.reset_search();
  // assigns the first address found to insideThermometer
  //if (!oneWire.search(insideThermometer)) Serial.println("Unable to find address for insideThermometer");

  // show the addresses we found on the bus
  Serial.print("Device 0 Address: ");
  printAddress(enclosureThermometer);
  Serial.println();

  // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(enclosureThermometer, 9);

  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(enclosureThermometer), DEC);
  Serial.println();
}

void pwmSetup()
{
  ledcAttachPin(kFanPWMPin, 0);
  ledcSetup(0, kPwmFrequency, kPwmResolution);
}

void setup()
{
  // put your setup code here, to run once:

#ifdef RH_HAVE_SERIAL
  Serial.begin(115200); // Debugging only
#endif
  Wire.begin();

  delay(100);
  lcdSetup();
  delay(100);
  tempSensorSetup();
  delay(100);
  pwmSetup();
  delay(100);
} // setup()

void loop()
{

  ledcWrite(kFanPWMPin, 100);
  // put your main code here, to run repeatedly:
}