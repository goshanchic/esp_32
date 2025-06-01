#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Структура данных для передачи через BLE
struct SensorData {
  float yaw;          // Углы в градусах
  float pitch;
  float roll;
  float accelX;       // Ускорения в м/с²
  float accelY;
  float accelZ;
  float posX;         // Позиция в мм
  float posY;
  float posZ;
} __attribute__((packed));

// Объявление объекта MPU6050
MPU6050 mpu;

// Переменные для работы с DMP
bool DMPReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint8_t FIFOBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3]; // Углы Yaw, Pitch, Roll

// Переменные для линейного ускорения
VectorInt16 aa;       // Сырые данные акселерометра
VectorInt16 aaReal;   // Ускорение без гравитации
float accel[3];       // Ускорение в м/с² по осям X, Y, Z

// Переменные для расчета перемещения
float velocity[3] = {0, 0, 0};    // Скорость в м/с по осям X, Y, Z
float position[3] = {0, 0, 0};    // Позиция в мм по осям X, Y, Z
unsigned long lastTime = 0;       // Время последнего измерения
float deltaTime = 0;              // Интервал времени между измерениями в секундах

// BLE настройки
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;

// Переменные для автокалибровки
unsigned long lastMotionTime = 0;
const unsigned long stillThreshold = 5000; // 5 секунд стабильности для калибровки
const float motionThreshold = 0.2; // Порог движения (±0.2 м/с²)
bool isCalibrating = false;
float offsetYPR[3] = {0, 0, 0}; // Смещения для коррекции углов
float offsetAccel[3] = {0, 0, 0}; // Смещения для коррекции ускорений

// Константы для фильтрации шумов
const float accelerationNoiseThreshold = 0.1; // м/с² - игнорировать ускорения меньше этого значения
const float velocityDecay = 0.95; // Фактор затухания скорости для уменьшения дрейфа

// Экземпляр структуры данных
SensorData sensorData;

// Функция проверки движения
bool isDeviceStill(float* currentYPR, float* currentAccel) {
  // Проверка ускорений в пределах погрешности
  for (int i = 0; i < 3; i++) {
    if (abs(currentAccel[i]) > motionThreshold) {
      return false;
    }
  }
  
  return true;
}

// Функция калибровки - сохраняет текущие значения как смещения
void calibrateSensors(float* currentYPR, float* currentAccel) {
  Serial.println(F("Выполняется калибровка..."));
  
  // Сохраняем текущие значения как смещения
  for (int i = 0; i < 3; i++) {
    offsetYPR[i] = currentYPR[i];
    offsetAccel[i] = currentAccel[i];
  }
  
  // Сбросить скорость и положение при калибровке
  for (int i = 0; i < 3; i++) {
    velocity[i] = 0;
    position[i] = 0;
  }
  
  Serial.println(F("Калибровка завершена!"));
  Serial.print(F("Offset YPR: "));
  Serial.print(offsetYPR[0] * 180 / M_PI);
  Serial.print(F(", "));
  Serial.print(offsetYPR[1] * 180 / M_PI);
  Serial.print(F(", "));
  Serial.println(offsetYPR[2] * 180 / M_PI);
  
  Serial.print(F("Offset Accel: "));
  Serial.print(offsetAccel[0]);
  Serial.print(F(", "));
  Serial.print(offsetAccel[1]);
  Serial.print(F(", "));
  Serial.println(offsetAccel[2]);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("Подключение не удалось!");
    while (1);
  } else {
    Serial.println("Подключение успешно!");
  }

  Serial.println(F("Инициализация DMP..."));
  devStatus = mpu.dmpInitialize();

  // Начальные смещения - могут быть скорректированы после калибровки вручную
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println(F("Базовая калибровка завершена!"));

    mpu.setDMPEnabled(true);
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println(F("DMP готов к использованию!"));
  } else {
    Serial.print(F("Инициализация DMP не удалась (код "));
    Serial.print(devStatus);
    Serial.println(F(")"));
    while (1);
  }

  BLEDevice::init("MPU6050_BLE");
  pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  BLEDevice::startAdvertising();
  Serial.println("BLE готов!");
  
  // Установка начального времени
  lastMotionTime = millis();
  lastTime = millis();
}

void loop() {
  if (!DMPReady) return;

  while (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    // Расчет времени между измерениями
    unsigned long currentTime = millis();
    deltaTime = (currentTime - lastTime) / 1000.0; // в секундах
    lastTime = currentTime;
    
    // Получение кватерниона
    mpu.dmpGetQuaternion(&q, FIFOBuffer);

    // Вычисление вектора гравитации
    mpu.dmpGetGravity(&gravity, &q);

    // Вычисление углов Yaw, Pitch, Roll
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Получение сырых данных акселерометра
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

    // Преобразование ускорения в м/с² (1g = 9.81 м/с², масштаб зависит от настроек MPU)
    accel[0] = aaReal.x * 9.81 / 16384.0; // X (влево-вправо)
    accel[1] = aaReal.y * 9.81 / 16384.0; // Y (вперед-назад)
    accel[2] = aaReal.z * 9.81 / 16384.0; // Z (вверх-вниз)

    // Проверка на стабильность положения
    if (isDeviceStill(ypr, accel)) {
      // Устройство неподвижно
      if (millis() - lastMotionTime > stillThreshold && !isCalibrating) {
        // Если устройство неподвижно более 5 секунд - калибруем
        isCalibrating = true;
        calibrateSensors(ypr, accel);
      }
      
      // Если устройство неподвижно, постепенно сбрасываем скорость к нулю
      for (int i = 0; i < 3; i++) {
        velocity[i] *= 0.5; // Более быстрое затухание в состоянии покоя
        if (abs(velocity[i]) < 0.01) velocity[i] = 0;
      }
    } else {
      // Устройство в движении - сбрасываем таймер
      lastMotionTime = millis();
      isCalibrating = false;
    }

    // Применение смещений для коррекции значений
    float correctedYPR[3];
    float correctedAccel[3];
    
    for (int i = 0; i < 3; i++) {
      correctedYPR[i] = ypr[i] - offsetYPR[i];
      correctedAccel[i] = accel[i] - offsetAccel[i];
      
      // Применение порогового фильтра шумов акселерометра
      if (abs(correctedAccel[i]) < accelerationNoiseThreshold) {
        correctedAccel[i] = 0;
      }
    }

    // Интегрирование ускорения для получения скорости
    for (int i = 0; i < 3; i++) {
      velocity[i] = velocity[i] * velocityDecay + correctedAccel[i] * deltaTime;
    }
    
    // Интегрирование скорости для получения перемещения (в метрах)
    for (int i = 0; i < 3; i++) {
      position[i] += velocity[i] * deltaTime;
    }
    
    // Заполнение структуры данных
    sensorData.yaw = correctedYPR[0] * 180 / M_PI;      // Конвертация в градусы
    sensorData.pitch = correctedYPR[1] * 180 / M_PI;
    sensorData.roll = correctedYPR[2] * 180 / M_PI;
    sensorData.accelX = correctedAccel[0];               // Ускорения в м/с²
    sensorData.accelY = correctedAccel[1];
    sensorData.accelZ = correctedAccel[2];
    sensorData.posX = position[0] * 1000.0;              // Позиция в мм
    sensorData.posY = position[1] * 1000.0;
    sensorData.posZ = position[2] * 1000.0;

    // Отправка структуры через BLE
    pCharacteristic->setValue((uint8_t*)&sensorData, sizeof(SensorData));
    pCharacteristic->notify();

    // Вывод в Serial для отладки
    Serial.print("Yaw: ");
    Serial.print(sensorData.yaw);
    Serial.print("\tPitch: ");
    Serial.print(sensorData.pitch);
    Serial.print("\tRoll: ");
    Serial.print(sensorData.roll);
    Serial.print("\tAccel X: ");
    Serial.print(sensorData.accelX);
    Serial.print("\tAccel Y: ");
    Serial.print(sensorData.accelY);
    Serial.print("\tAccel Z: ");
    Serial.print(sensorData.accelZ);
    Serial.print("\tPos X (mm): ");
    Serial.print(sensorData.posX);
    Serial.print("\tPos Y (mm): ");
    Serial.print(sensorData.posY);
    Serial.print("\tPos Z (mm): ");
    Serial.println(sensorData.posZ);

    delay(10); // Уменьшенная задержка для более точного интегрирования
  }
}