// This example shows the 3 axis acceleration.
#include "LIS3DHTR.h"
#include <Wire.h>
LIS3DHTR<TwoWire> LIS; //IIC
#define WIRE Wire
#define I2C_SDA 6   //SDA on GPIO6
#define I2C_SCL 7   //SCL on GPIO7
#define ACC_I2C_ADR 0x18

void setup() {
    Serial.begin(115200);
    while (!Serial) {};
    // Initialize I2C with custom pins for ESP32        [ChatGPT: For the following code, where do I set the SCL and SDA wires my ESP32 uses for the I2C communication:]
    Wire.begin(I2C_SDA, I2C_SCL); // SDA, SCL

    LIS.begin(WIRE, ACC_I2C_ADR); //IIC init dafault :0x18
    //LIS.begin(WIRE, 0x19); //IIC init
    delay(100);
    //  LIS.setFullScaleRange(LIS3DHTR_RANGE_2G);
    //  LIS.setFullScaleRange(LIS3DHTR_RANGE_4G);
    //  LIS.setFullScaleRange(LIS3DHTR_RANGE_8G);
    //  LIS.setFullScaleRange(LIS3DHTR_RANGE_16G);
    //  LIS.setOutputDataRate(LIS3DHTR_DATARATE_1HZ);
    //  LIS.setOutputDataRate(LIS3DHTR_DATARATE_10HZ);
    //  LIS.setOutputDataRate(LIS3DHTR_DATARATE_25HZ);
    LIS.setOutputDataRate(LIS3DHTR_DATARATE_50HZ);
    //  LIS.setOutputDataRate(LIS3DHTR_DATARATE_100HZ);
    //  LIS.setOutputDataRate(LIS3DHTR_DATARATE_200HZ);
    //  LIS.setOutputDataRate(LIS3DHTR_DATARATE_1_6KHZ);
    //  LIS.setOutputDataRate(LIS3DHTR_DATARATE_5KHZ);
    LIS.setHighSolution(true); //High solution enable
}
void loop() {
    if (!LIS) {
        Serial.println("LIS3DHTR didn't connect.");
        while (1);
        return;
    }
    //3 axis
    Serial.print("x:"); Serial.print(LIS.getAccelerationX()); Serial.print("  ");
    Serial.print("y:"); Serial.print(LIS.getAccelerationY()); Serial.print("  ");
    Serial.print("z:"); Serial.println(LIS.getAccelerationZ());

    delay(500);
}
