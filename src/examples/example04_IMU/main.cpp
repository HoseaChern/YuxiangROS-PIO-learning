#include <Arduino.h>
#include <MPU6050_light.h>
#include <Wire.h>

namespace {

// ---- 串口与 I2C 引脚 ----
constexpr uint32_t SERIAL_BAUD = 115200; // 串口波特率
constexpr uint8_t I2C_SDA_PIN = 2;       // I2C SDA 引脚
constexpr uint8_t I2C_SCL_PIN = 1;       // I2C SCL 引脚

// ---- 时序参数 ----
constexpr uint32_t BOOT_DELAY_MS = 1000;   // 启动延时, 用于姿态校准前稳定
constexpr uint32_t PRINT_INTERVAL_MS = 10; // 角度输出间隔, 单位: ms

// ---- 可变全局状态 (跨 setup/loop 共享) ----
MPU6050 mpu(Wire);       // MPU6050 对象, 使用 Wire 作为 I2C 总线
unsigned long timer = 0; // 上一次打印时刻, 单位: ms

} // namespace

void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial) {
        ;
    }

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while (status != 0) {
        // 若无法连接, 则停止一切
    }

    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(BOOT_DELAY_MS);
    mpu.calcOffsets();
    Serial.println("Done!\n");
}

void loop() {
    mpu.update();

    if ((millis() - timer) > PRINT_INTERVAL_MS) {
        Serial.print("X : ");
        Serial.print(mpu.getAngleX());
        Serial.print("\tY : ");
        Serial.print(mpu.getAngleY());
        Serial.print("\tZ : ");
        Serial.println(mpu.getAngleZ());
        timer = millis();
    }
}
