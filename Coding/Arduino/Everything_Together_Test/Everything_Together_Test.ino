#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include "Adafruit_MCP9600.h"

#include "LIS3DHTR.h"
LIS3DHTR<TwoWire> LIS; //IIC

//Define I2C pins
#define WIRE Wire
#define IIC_SDA 6   //SDA on GPIO6
#define IIC_SCL 7   //SCL on GPIO7

//Addresses (MCP9600's):
#define I2C_ADDRESS_1 (0x60)    //CONN1
#define I2C_ADDRESS_2 (0x62)    //CONN2
#define I2C_ADDRESS_3 (0x64)    //CONN3
#define I2C_ADDRESS_4 (0x66)    //CONN4
//#define I2C_ADDRESS_5 (0x67)    //CONN5

//Address (LIS3DH):
#define ACC_I2C_ADR 0x18

Adafruit_MCP9600 mcp1;
Adafruit_MCP9600 mcp2;
Adafruit_MCP9600 mcp3;
Adafruit_MCP9600 mcp4;
//Adafruit_MCP9600 mcp5;

/* Set and print ambient resolution */
Ambient_Resolution ambientRes = RES_ZERO_POINT_125;

//Buzzer pin set
#define BUZZER_PIN 14

//Buzzer notes
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494

int melody1[] = {
  NOTE_D4, NOTE_F4, NOTE_E4, NOTE_C4, NOTE_D4
};

int durations1[] = {
  10, 6, 6, 8, 10
};

int melody2[] = {
  NOTE_E4, NOTE_G4, NOTE_F4, NOTE_A4, NOTE_G4, NOTE_B4, NOTE_A4
};

int durations2[] = {
  10, 8, 10, 8, 10, 8, 10
};

bool note1trigger = false;
bool note2trigger = false;

void setup()
{
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  //Buzzer pin setup
  pinMode(BUZZER_PIN, OUTPUT);
  
  //Set I2C pins
  Wire.begin(IIC_SDA, IIC_SCL); // SDA, SCL

  //Begin LIS3DH communication
  LIS.begin(WIRE, ACC_I2C_ADR); //IIC init dafault :0x18
  delay(100);
  LIS.setOutputDataRate(LIS3DHTR_DATARATE_50HZ);
  LIS.setHighSolution(true); //High solution enable

  Serial.println("MCP9600 HW test");

   /* Initialise the driver with I2C_ADDRESS and the default I2C bus. */
  if (! mcp1.begin(I2C_ADDRESS_1, &Wire)) {
    Serial.println("Sensor 1 not found. Check wiring!");
    while (1);
  }
  if (! mcp2.begin(I2C_ADDRESS_2, &Wire)) {
    Serial.println("Sensor 2 not found. Check wiring!");
    while (1);
  }
  if (! mcp3.begin(I2C_ADDRESS_3, &Wire)) {
    Serial.println("Sensor 3 not found. Check wiring!");
    while (1);
  }
  if (! mcp4.begin(I2C_ADDRESS_4, &Wire)) {
    Serial.println("Sensor 4 not found. Check wiring!");
    while (1);
  }
  //if (! mcp5.begin(I2C_ADDRESS_5, &Wire)) {
  //  Serial.println("Sensor 5 not found. Check wiring!");
  //  while (1);
  //}

  Serial.println("Found MCP9600!");

  /* Set and print ambient resolution */
  mcp1.setAmbientResolution(ambientRes);
  mcp2.setAmbientResolution(ambientRes);
  mcp3.setAmbientResolution(ambientRes);
  mcp4.setAmbientResolution(ambientRes);
  //mcp5.setAmbientResolution(ambientRes);
  Serial.print("Ambient Resolution set to: ");
  switch (ambientRes) {
    case RES_ZERO_POINT_25:    Serial.println("0.25°C"); break;
    case RES_ZERO_POINT_125:   Serial.println("0.125°C"); break;
    case RES_ZERO_POINT_0625:  Serial.println("0.0625°C"); break;
    case RES_ZERO_POINT_03125: Serial.println("0.03125°C"); break;
  }

  mcp1.setADCresolution(MCP9600_ADCRESOLUTION_18);
  mcp2.setADCresolution(MCP9600_ADCRESOLUTION_18);
  mcp3.setADCresolution(MCP9600_ADCRESOLUTION_18);
  mcp4.setADCresolution(MCP9600_ADCRESOLUTION_18);
  //mcp5.setADCresolution(MCP9600_ADCRESOLUTION_18);
  Serial.print("ADC resolution set to ");
  switch (mcp1.getADCresolution()) {
    case MCP9600_ADCRESOLUTION_18:   Serial.print("18"); break;
    case MCP9600_ADCRESOLUTION_16:   Serial.print("16"); break;
    case MCP9600_ADCRESOLUTION_14:   Serial.print("14"); break;
    case MCP9600_ADCRESOLUTION_12:   Serial.print("12"); break;
  }
  Serial.println(" bits");

  mcp1.setThermocoupleType(MCP9600_TYPE_K);
  mcp2.setThermocoupleType(MCP9600_TYPE_K);
  mcp3.setThermocoupleType(MCP9600_TYPE_K);
  mcp4.setThermocoupleType(MCP9600_TYPE_K);
  //mcp5.setThermocoupleType(MCP9600_TYPE_K);
  Serial.print("Thermocouple type set to ");
  switch (mcp1.getThermocoupleType()) {
    case MCP9600_TYPE_K:  Serial.print("K"); break;
    case MCP9600_TYPE_J:  Serial.print("J"); break;
    case MCP9600_TYPE_T:  Serial.print("T"); break;
    case MCP9600_TYPE_N:  Serial.print("N"); break;
    case MCP9600_TYPE_S:  Serial.print("S"); break;
    case MCP9600_TYPE_E:  Serial.print("E"); break;
    case MCP9600_TYPE_B:  Serial.print("B"); break;
    case MCP9600_TYPE_R:  Serial.print("R"); break;
  }
  Serial.println(" type");

  mcp1.setFilterCoefficient(6);
  mcp2.setFilterCoefficient(6);
  mcp3.setFilterCoefficient(6);
  mcp4.setFilterCoefficient(6);
  //mcp5.setFilterCoefficient(3);
  Serial.print("Filter coefficient value set to: ");
  Serial.println(mcp1.getFilterCoefficient());

  //mcp.setAlertTemperature(1, 30);
  //Serial.print("Alert #1 temperature set to ");
  //Serial.println(mcp.getAlertTemperature(1));
  //mcp.configureAlert(1, true, true);  // alert 1 enabled, rising temp

  mcp1.enable(true);
  mcp2.enable(true);
  mcp3.enable(true);
  mcp4.enable(true);
  //mcp5.enable(true);

  Serial.println(F("------------------------------"));
}

void loop()
{
  int size1 = sizeof(durations1) / sizeof(int);
  int size2 = sizeof(durations2) / sizeof(int);

  if (!LIS) {
      Serial.println("LIS3DHTR didn't connect.");
      while (1);
      return;
  }

  //Buzzer trigger
  if(note1trigger == true){
    note1trigger = false;
  
    for (int note = 0; note < size1; note++) {
      //to calculate the note duration, take one second divided by the note type.
      //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
      int duration = 1000 / durations1[note];
      tone(BUZZER_PIN, melody1[note], duration);

      //to distinguish the notes, set a minimum time between them.
      //the note's duration + 30% seems to work well:
      int pauseBetweenNotes = duration * 1.30;
      delay(pauseBetweenNotes);
      
      //stop the tone playing:
      noTone(BUZZER_PIN);
    }
  }

  if(note2trigger == true){
    note2trigger = false;
  
    for (int note = 0; note < size2; note++) {
      //to calculate the note duration, take one second divided by the note type.
      //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
      int duration = 1000 / durations2[note];
      tone(BUZZER_PIN, melody2[note], duration);

      //to distinguish the notes, set a minimum time between them.
      //the note's duration + 30% seems to work well:
      int pauseBetweenNotes = duration * 1.30;
      delay(pauseBetweenNotes);
      
      //stop the tone playing:
      noTone(BUZZER_PIN);
    }
  }

  Serial.println("----------Accelerometer----------");
  Serial.print("x:"); Serial.print(LIS.getAccelerationX()); Serial.print("  ");
  Serial.print("y:"); Serial.print(LIS.getAccelerationY()); Serial.print("  ");
  Serial.print("z:"); Serial.println(LIS.getAccelerationZ());
  Serial.println(F("------------------------------------------------------------"));
  //Set buzzer trigger to set true if z value less than -0.8
  if(LIS.getAccelerationZ() <= -0.8){
    note1trigger = true;
  }

  Serial.println("----------MCP1 Readings----------");
  Serial.print("Hot Junction: "); Serial.println(mcp1.readThermocouple());
  Serial.print("Cold Junction: "); Serial.println(mcp1.readAmbient());
  Serial.print("ADC: "); Serial.print(mcp1.readADC() * 2); Serial.println(" uV");
  Serial.println(F("------------------------------------------------------------"));
  // if(mcp1.readThermocouple() >= 30.0){
    // note2trigger = true;
  // }

  Serial.println("----------MCP2 Readings----------");
  Serial.print("Hot Junction: "); Serial.println(mcp2.readThermocouple());
  Serial.print("Cold Junction: "); Serial.println(mcp2.readAmbient());
  Serial.print("ADC: "); Serial.print(mcp2.readADC() * 2); Serial.println(" uV");
  Serial.println(F("------------------------------------------------------------"));

  Serial.println("----------MCP3 Readings----------");
  Serial.print("Hot Junction: "); Serial.println(mcp3.readThermocouple());
  Serial.print("Cold Junction: "); Serial.println(mcp3.readAmbient());
  Serial.print("ADC: "); Serial.print(mcp3.readADC() * 2); Serial.println(" uV");
  Serial.println(F("------------------------------------------------------------"));

  Serial.println("----------MCP4 Readings----------");
  Serial.print("Hot Junction: "); Serial.println(mcp4.readThermocouple());
  Serial.print("Cold Junction: "); Serial.println(mcp4.readAmbient());
  Serial.print("ADC: "); Serial.print(mcp4.readADC() * 2); Serial.println(" uV");
  Serial.println(F("------------------------------------------------------------"));

  //Serial.println("----------MCP5 Readings----------");
  //Serial.print("Hot Junction: "); Serial.println(mcp5.readThermocouple());
  //Serial.print("Cold Junction: "); Serial.println(mcp5.readAmbient());
  //Serial.print("ADC: "); Serial.print(mcp5.readADC() * 2); Serial.println(" uV");
  //Serial.println(F("------------------------------------------------------------"));

  delay(1000);
}
