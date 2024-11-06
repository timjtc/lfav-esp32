#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <QTRSensors.h>
#include <SparkFun_TB6612.h>

// Function prototypes
void parseCommand(String command);
void setLedMode(String mode);
void robotControl();
void updatePID(int error);
void motorDrive(int left, int right);

// Pin definitions
const uint8_t QTR_PINS[] = {23, 22, 13, 15, 32, 33, 25, 26};
const uint8_t QTR_EMITTER = 27;
const uint8_t LED_MODE_IDLE = 14;
const uint8_t LED_MODE_ACTIVE = 12;
const uint8_t DRIVER_AIN1 = 17;
const uint8_t DRIVER_BIN1 = 18;
const uint8_t DRIVER_AIN2 = 16;
const uint8_t DRIVER_BIN2 = 19;
const uint8_t DRIVER_PWMA = 4;
const uint8_t DRIVER_PWMB = 21;
const uint8_t DRIVER_STBY = 5;

// BLE setup
const char* SERVICE_UUID = "a16587d4-584a-4668-b279-6ccb940cdfd0";
const char* CHARACTERISTIC_UUID_TX = "a16587d4-584a-4668-b279-6ccb940cdfd1";
const char* CHARACTERISTIC_UUID_RX = "a16587d4-584a-4668-b279-6ccb940cdfd2";
BLEServer* Server = NULL;
BLECharacteristic *CharTX, *CharRX;
bool dev_connected = false;
bool last_dev_connected = false;
String tx_idle = "";
String tx_lnfollow = "";
String tx_qtrcall = "";

// QTR sensor setup
const uint8_t SENSOR_COUNT = 8;
QTRSensors QTR;
uint16_t sensor_values[SENSOR_COUNT];
uint16_t qtrcall_min_values[SENSOR_COUNT];
uint16_t qtrcall_max_values[SENSOR_COUNT];

// Motor setup
const int offset_motorA = 1;
const int offset_motorB = 1;
Motor MotorA_Left = Motor(DRIVER_AIN1, DRIVER_AIN2, DRIVER_PWMA, offset_motorA, DRIVER_STBY);
Motor MotorB_Right = Motor(DRIVER_BIN1, DRIVER_BIN2, DRIVER_PWMA, offset_motorB, DRIVER_STBY);

// Vehicle controller setup
String mode = "IDLE";
float Kp = 0.00;
float Ki = 0.00;
float Kd = 0.00;

uint8_t Kp_final;
uint8_t Ki_final;
uint8_t Kd_final;
float Pvalue;
float Ivalue;
float Dvalue;

uint16_t position;
int P, I, D, prev_error, pid_value, pid_error;
int lsp, rsp;
int lfspeed = 230;

int left_motor = 0;
int right_motor = 0;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    dev_connected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    dev_connected = false;
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rx_value = pCharacteristic->getValue();
    if (rx_value.length() > 0) {
      parseCommand(String(rx_value.c_str()));
    }
  }
};

void setup() {

  // Initialize serial and pin modes
  Serial.begin(115200);
  // pinMode(LED_BLE, OUTPUT);
  pinMode(LED_MODE_IDLE, OUTPUT);
  pinMode(LED_MODE_ACTIVE, OUTPUT);

  BLEDevice::init("LFAV-1");

  // Create the BLE Server
  Server = BLEDevice::createServer();
  Server->setCallbacks(new ServerCallbacks());
  BLEService *Service = Server->createService(SERVICE_UUID);

  // BLEChar for telemetry data
  CharTX = Service->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY
  );

  // BLEChar for receiving commands
  CharRX = Service->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_READ | 
    BLECharacteristic::PROPERTY_WRITE
  );

  CharRX->setCallbacks(new CommandCallbacks());
  CharTX->addDescriptor(new BLE2902());
  CharRX->addDescriptor(new BLE2902());
  Service->start();
  Server->getAdvertising()->start();
  Serial.println("Waiting for a client connection...");

  // Initialize QTR sensor
  QTR.setTypeRC();
  QTR.setSensorPins(QTR_PINS, SENSOR_COUNT);
  QTR.setEmitterPin(QTR_EMITTER);

}

void loop() {

  // On connect
  if (dev_connected) {

    setLedMode(mode);

    if (mode == "QTRCALL") {
      for (uint16_t i = 0; i < 400; i++)
      {
        QTR.calibrate();
        tx_qtrcall = mode + "," +
                    String(i) + "," +
                    "-" + "," +
                    "-" + "," +
                    "-" + "," +
                    "-" + "," +
                    "-" + "," +
                    "-" + "," +
                    "-" + "," +
                    "-" + "," +
                    "-" + "," +
                    "-" + "," +
                    "-" + "," +
                    "-" + "," +
                    "-" + "," +
                    "-" + "," +
                    "-" + "," +
                    "-";
        CharTX->setValue(tx_qtrcall.c_str());
        CharTX->notify();
      }
      for (uint8_t i = 0; i < SENSOR_COUNT; i++)
      {
        qtrcall_min_values[i] = QTR.calibrationOn.minimum[i];
        qtrcall_max_values[i] = QTR.calibrationOn.maximum[i];
      }
      tx_qtrcall = mode + "," +
                  "400" + "," +
                  qtrcall_min_values[0] + "," +
                  qtrcall_min_values[1] + "," +
                  qtrcall_min_values[2] + "," +
                  qtrcall_min_values[3] + "," +
                  qtrcall_min_values[4] + "," +
                  qtrcall_min_values[5] + "," +
                  qtrcall_min_values[6] + "," +
                  qtrcall_min_values[7] + "," +
                  qtrcall_max_values[0] + "," +
                  qtrcall_max_values[1] + "," +
                  qtrcall_max_values[2] + "," +
                  qtrcall_max_values[3] + "," +
                  qtrcall_max_values[4] + "," +
                  qtrcall_max_values[5] + "," +
                  qtrcall_max_values[6] + "," +
                  qtrcall_max_values[7];
      CharTX->setValue(tx_qtrcall.c_str());
      CharTX->notify();

      mode = "IDLE";

      delay(500);
    }

    if (mode == "LNFOLLOW") {
      position = QTR.readLineBlack(sensor_values);
      Serial.println(position);

      motorDrive(230, 230);

      // robotControl();

      // send telemetry data
      tx_lnfollow = mode + "," +
                    String(left_motor) + "," + 
                    String(right_motor) + "," +
                    String(sensor_values[0]) + "," +
                    String(sensor_values[1]) + "," +
                    String(sensor_values[2]) + "," +
                    String(sensor_values[3]) + "," +
                    String(sensor_values[4]) + "," +
                    String(sensor_values[5]) + "," +
                    String(sensor_values[6]) + "," +
                    String(sensor_values[7]);
      CharTX->setValue(tx_lnfollow.c_str());
      CharTX->notify();
    }

    if (mode == "IDLE") {
      // Safe all motors
      left_motor = 0;
      right_motor = 0;
      MotorA_Left.brake();
      MotorB_Right.brake();

      // send telemetry data
      tx_idle = mode + "," + 
                String(Kp) + "," + 
                String(Ki) + "," + 
                String(Kd);
      CharTX->setValue(tx_idle.c_str());
      CharTX->notify();
      delay(500);
    }

  }

  // On disconnect
  if (!dev_connected && last_dev_connected) {

    // Reset mode and stop motors
    mode = "IDLE";

    // Give the BLE stack the chance to get things ready and advertise again
    delay(500);
    Server->startAdvertising(); 
    Serial.println("Advertising again...");
    last_dev_connected = dev_connected;

    // digitalWrite(LED_BLE, HIGH);

  }

  // While connecting
  if (dev_connected && !last_dev_connected) {

    last_dev_connected = dev_connected;
    // digitalWrite(LED_BLE, HIGH);

  }

}

void parseCommand(String command) {
  command.trim();
  if (command.startsWith("mode ")) {
    String mode_value = command.substring(5);
    if (mode_value == "lf") {
      mode = "LNFOLLOW";
    } else if (mode_value == "qc") {
      mode = "QTRCALL";
    } else if (mode_value == "id") {
      mode = "IDLE";
    } else {
      Serial.println("Unknown mode: " + mode_value + ". Available modes: id, qc, lf");
    }
  } else if (command.startsWith("set kp ")) {
    String kpValue = command.substring(7);
    Kp = kpValue.toFloat();
  } else if (command.startsWith("set ki ")) {
    String kiValue = command.substring(7);
    Ki = kiValue.toFloat();
  } else if (command.startsWith("set kd ")) {
    String kdValue = command.substring(7);
    Kd = kdValue.toFloat();
  } else {
    Serial.println("Unknown command: " + command);
  }
}

void setLedMode(String mode) {

  if (mode == "IDLE") {
    digitalWrite(LED_MODE_IDLE, HIGH);
    digitalWrite(LED_MODE_ACTIVE, LOW);
  } else if (mode == "QTRCALL") {
    digitalWrite(LED_MODE_IDLE, LOW);
    digitalWrite(LED_MODE_ACTIVE, HIGH);
  } else if (mode == "LNFOLLOW") {
    digitalWrite(LED_MODE_IDLE, LOW);
  }

}

void robotControl() {
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 4000 (for a white line, use readLineWhite() instead)

  position = QTR.readLineBlack(sensor_values);
  pid_error = 2000 - position;
  while (sensor_values[0] >= 980 &&
         sensor_values[1] >= 980 && 
         sensor_values[2] >= 980 && 
         sensor_values[3] >= 980 && 
         sensor_values[4] >= 980 && 
         sensor_values[5] >= 980 && 
         sensor_values[6] >= 980 && 
         sensor_values[7] >= 980) 
  {                             // A case when the line follower leaves the line
    if (prev_error > 0) {       //Turn left if the line was to the left before
      motorDrive(-230, 230);
    }
    else{
      motorDrive(230, -230);    // Else turn right
    }
    position = QTR.readLineBlack(sensor_values);
  }
  
  updatePID(pid_error);
}

void updatePID(int error){
    P = pid_error;
    I = I + pid_error;
    D = pid_error - prev_error;
    
    Pvalue = Kp * P;
    Ivalue = Ki * I;
    Dvalue = Kd * D; 

    float PIDvalue = Pvalue + Ivalue + Dvalue;
    prev_error = error;

    lsp = lfspeed - PIDvalue;
    rsp = lfspeed + PIDvalue;

    if (lsp > 255) {
      lsp = 255;
    }
    if (lsp < -255) {
      lsp = -255;
    }
    if (rsp > 255) {
      rsp = 255;
    }
    if (rsp < -255) {
      rsp = -255;
    }
    motorDrive(lsp, rsp);
}

void motorDrive(int left, int right) {
  
  left_motor = left;
  right_motor = right;
  MotorB_Right.drive(right);
  MotorA_Left.drive(left);

}