

/*********************头文件*********************/
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

HardwareSerial mySerial(1);

#define Step1_BlueTooth 1
/*****************蓝牙发送接收数据*****************/
#if Step1_BlueTooth
bool MotorStatus;  // 控制电机开关
int16_t kp, kd, ksp, ksi;
// 以上为手机发送来的数据
bool BLEsendflag = false;  // 定时发送标志
uint8_t BLEBUF[13];        // 数据包的顺序为BOOL(1)/BYTE(1)/SHORT(2)/INT(4)/FLOAT4)
short PwmOut = -12;
int Encode_L = -34;
float Angle = 5.6;
#endif
/************************************************/
/********************基本设置********************/
hw_timer_t *timer = NULL;
void BClock_Init(int duty_ms);
static void IRAM_ATTR Timer0_CallBack(void);  // 以上为定时器
void Short2Byte(short i, uint8_t *byte);
void Int2Byte(int i, uint8_t *byte);
void Float2Byte(float f, uint8_t *byte);  // 以上为数据类型转BYTE
/************************************************/

#if Step1_BlueTooth
void getBlueData(uint8_t *Value) {
  // MotorStatus = Value[1];
  // kp = (Value[3] << 8) + Value[2];
  // kp = (Value[3] << 8) + Value[2];
  // kd = (Value[5] << 8) + Value[4];
  // ksp = (Value[7] << 8) + Value[6];
  // ksi = (Value[9] << 8) + Value[8];
  // mySerial.printf("MotorStatus=%d,kp=%d,kd=%d,ksp=%d,ksi=%d\n", MotorStatus, kp, kd, ksp, ksi);
  mySerial.printf("recved data: %x, %x, %x, %x, %x\n", Value[0], Value[1], Value[2], Value[3], Value[4]);
}
#endif

/*********************蓝牙BLE********************/
#if Step1_BlueTooth
BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"  // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    pServer->getAdvertising()->start();
  }
};

void getBlueData(uint8_t *Value);

uint8_t modes[8];

class MyCallbacks : public BLECharacteristicCallbacks {
  // void onWrite(BLECharacteristic *pCharacteristic) {
  //   std::string rxValue = pCharacteristic->getValue();
  //   if (rxValue.length() > 0) {
  //     for (int i = 0; i < rxValue.length(); i++) {
  //       modes[i] = rxValue[i];
  //     }
  //     getBlueData(modes);
  //   }
  // }
};

void BLEinit() {
  BLEDevice::init("controller");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());
  // BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  // pRxCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();
  pServer->getAdvertising()->start();
  mySerial.println("BLE is OK.");
}
#endif
/************************************************/

void setup() {
  mySerial.begin(115200, SERIAL_8N1, 20, 21);

#if Step1_BlueTooth
  BLEinit();All
#endif
  BClock_Init(1000);  // 定时器单位为ms，发送数据至手机的频率
}

void loop() {
  if (deviceConnected & BLEsendflag) {
    BLEsendflag = false;
    BLEBUF[0] = 0xA5;  // 包头
    BLEBUF[1] = 0x01;  //
    BLEBUF[2] = 0x02;  //
    BLEBUF[3] = 0x03;  //
    BLEBUF[4] = 0x04;  //
    // Short2Byte(PwmOut, &BLEBUF[1]);
    // Int2Byte(PwmOut, &BLEBUF[3]);
    // Float2Byte(Angle, &BLEBUF[7]);
    // BLEBUF[11] = (uint8_t)((BLEBUF[1] + BLEBUF[2] + BLEBUF[3] + BLEBUF[4] + BLEBUF[5] + BLEBUF[6] + BLEBUF[7] + BLEBUF[8] + BLEBUF[9] + BLEBUF[10]) & 0xFF);
    BLEBUF[5] = 0x5A;  // 包尾
    pTxCharacteristic->setValue(BLEBUF, 6);
    pTxCharacteristic->notify();
    mySerial.printf("sent data: %x, %x, %x, %x, %x, %x\n", BLEBUF[0], BLEBUF[1], BLEBUF[2], BLEBUF[3], BLEBUF[4], BLEBUF[5]);
  }
}

static void IRAM_ATTR Timer0_CallBack(void) {
  BLEsendflag = true;
}

void BClock_Init(int duty_ms) {
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, Timer0_CallBack, true);
  timerAlarmWrite(timer, 1000 * duty_ms, true);  // 单位us,定时模式,10ms
  timerAlarmEnable(timer);                       // 启动定时器
}

void Short2Byte(short i, uint8_t *byte) {
  unsigned long longdata = 0;
  longdata = *(unsigned long *)&i;
  byte[1] = (longdata & 0xFF00) >> 8;
  byte[0] = (longdata & 0x00FF);
}

void Int2Byte(int i, uint8_t *byte) {
  unsigned long longdata = 0;
  longdata = *(unsigned long *)&i;
  byte[3] = (longdata & 0xFF000000) >> 24;
  byte[2] = (longdata & 0x00FF0000) >> 16;
  byte[1] = (longdata & 0x0000FF00) >> 8;
  byte[0] = (longdata & 0x000000FF);
}

void Float2Byte(float f, uint8_t *byte) {
  unsigned long longdata = 0;
  longdata = *(unsigned long *)&f;
  byte[3] = (longdata & 0xFF000000) >> 24;
  byte[2] = (longdata & 0x00FF0000) >> 16;
  byte[1] = (longdata & 0x0000FF00) >> 8;
  byte[0] = (longdata & 0x000000FF);
}
