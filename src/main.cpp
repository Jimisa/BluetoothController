#include <Arduino.h>
#include <BluetoothSerial.h>

const int xAxisGPIO = 12;
const int yAxisGPIO = 14;
const int zButtonGPIO = 23;

String device_name = "BTController_1";

byte xyOutput;
byte xOutput, yOutput, last_xOutput, last_yOutput = 0;
int xInput, yInput = 0;
int xCenter_min = 4095, yCenter_min = 4095;
int xCenter_max, yCenter_max = 0;

BluetoothSerial SerialBT;
uint8_t carAdress[6] = {0xC4,0x4F,0x33,0x75,0x98,0xBF};
//Controller address {0xAC,0x67,0xB2,0x2B,0x27,0xF6};


void setup() {
  pinMode(xAxisGPIO,INPUT);
  pinMode(yAxisGPIO,INPUT);
  pinMode(2,OUTPUT);

  digitalWrite(2,HIGH);

  Serial.begin(115200);

  int xCal_tmp = 2048;
  int yCal_tmp = 2048;
  for(int i = 0;i<10;i++) {
    xCal_tmp = analogRead(xAxisGPIO);
    yCal_tmp = analogRead(yAxisGPIO);
    if (xCenter_max < xCal_tmp) {
      xCenter_max = xCal_tmp;
    }
    if (xCenter_min > xCal_tmp) {
      xCenter_min = xCal_tmp;
    }
    if (yCenter_max < yCal_tmp) {
      yCenter_max = yCal_tmp;
    }
    if (yCenter_min > yCal_tmp) {
      yCenter_min = yCal_tmp;
    }
    delay(50);
  }

  log_i("X Center : [%d , %d ] ; Y Center : [%d , %d ]",xCenter_min,xCenter_max,yCenter_min,yCenter_max);

  SerialBT.begin(device_name,true);
  SerialBT.setPin("1234");

  bool connected = SerialBT.connect(carAdress);

  if (connected) {
    digitalWrite(2,LOW);
    log_i("Connected succesfully");
  } else {
      while(!SerialBT.connected(5000)) {
        digitalWrite(2,HIGH);
        log_d("Failed to connect. Make sure remote device is available and in range, then restart app.");
      }
  }
}

void loop() {
  xInput = analogRead(xAxisGPIO);
  yInput = analogRead(yAxisGPIO);
  log_i("raw x-Axis : %d , raw y-Axis : %d",xInput,yInput);

  if (xInput < xCenter_min - 50) {
    xOutput = byte(map(xInput,0,xCenter_min-50,0,6));
  } else if (xInput > xCenter_max + 50) {
    xOutput = byte(map(xInput,xCenter_max+50,4095,8,15));
  } else {
    xOutput = 7;
  }
  
  if (yInput < yCenter_min - 50) {
    yOutput = byte(map(yInput,0,yCenter_min-50,0,6));
  } else if (yInput > yCenter_max + 50) {
    yOutput = byte(map(yInput,yCenter_max+50,4095,8,15));
  } else {
    yOutput = 7;
  }

  //yOutput = byte(map((yInput+575),0,4095,0,15)); //575
  log_i("xInput (byte) : %d , yInput (byte) : %d",xOutput,yOutput);
  
  if ((yOutput != last_yOutput) || (xOutput != last_xOutput)) {
    xyOutput = xOutput | (yOutput << 4); // (yOutput << 4) | xOutput
    SerialBT.write(xyOutput);

    last_xOutput = xOutput;
    last_yOutput = yOutput;
  }
  delay(500);
  if (!SerialBT.connected()) {
    while(true) {
      digitalWrite(2,HIGH);
    }
  }
}