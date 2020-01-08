#include <GyverButton.h>
#include <Wire.h>

#define MPU6050_ADRESS 0x68

const int ACCEL_OFFSET   = 200;
const int GYRO_OFFSET    = 151;  // 151
const int GYRO_SENSITITY = 131;  // 131 is sensivity of gyro from data sheet
const float GYRO_SCALE   = 0.2; //  0.02 by default - tweak as required
const float LOOP_TIME    = 0.15; // 0.1 = 100ms

int accValue[3], accAngle[3], gyroValue[3], temperature, accCorr;
float gyroAngle[3], gyroCorr;

#define N 60

int coordX[N];
int coordY[N];
int coordZ[N];

int gravX = 0;
int gravY = 1;
int gravZ = 0;

#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2

#define POS_X 0
#define NEG_X 1
#define POS_Z 2
#define NEG_Z 3
#define POS_Y 4
#define NEG_Y 5

#define BUTTON1 A2
#define BUTTON2 A3
#define GREEN_LED A0
#define RED_LED A1

#define ALL 0
#define RAIN 1
#define SIN 2
#define WATER 3


#define RAIN_TIMER 1
#define SIN_TIMER 5
#define WATER_TIMER 1

#include <SPI.h>

GButton butt1(BUTTON1);
GButton butt2(BUTTON2);


uint8_t cubeWater[8][8][8];
uint8_t cube[8][8];
int8_t currentEffect;
uint16_t timer;
uint16_t modeTimer;
bool loading;
int8_t pos;
int8_t vector[3];
int16_t coord[3];
int sinus[8];


void setup() {
  Serial.begin(9600);

  loading = true;
  currentEffect = -1;

  butt1.setStepTimeout(100);    // настрйока интервала инкремента (по умолчанию 800 мс)
  butt2.setStepTimeout(100);

  SPI.begin();
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, LOW);

  for (int i = 0; i < 8; i++) {
    sinus[i] = round((sin(PI * ((float) i - 4) / 8) + 1) * 3.5) - 4;
    Serial.println(sinus[i]);
  }


  for (int i = 0; i < 8; i++) {
    for (int j = 0; j < 8; j++) {
      for (int k = 0; k < 8; k++) {
        cubeWater[i][j][k] = 0;
      }
    }
  }

  for (int i = 0; i < N; i++) {
    coordX[i] = random(0, 7);
    coordY[i] = random(0, 7);
    coordZ[i] = random(0, 7);
    cubeWater[coordX[i]][coordY[i]][coordZ[i]] = 1;
  }

  
  randomSeed(analogRead(0));

  changeMode();

  Wire.begin();
  Wire.beginTransmission(MPU6050_ADRESS); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  Serial.flush();
  }

void loop() { 
  butt1.tick();
  if (butt1.isClick()) {
    digitalWrite(GREEN_LED, HIGH);
    Serial.write("1\n");
    changeMode();
    delay(500);
    digitalWrite(GREEN_LED, LOW);

  }
  butt2.tick();
  if (butt2.isClick()) {
    digitalWrite(RED_LED, HIGH);
    delay(500);
    digitalWrite(RED_LED, LOW);

  }
  switch (currentEffect) {
    case RAIN:
      rain();
      modeTimer = RAIN_TIMER;
      break;
    case ALL:
      all();
      break;
    case SIN:
      mySinus();
      modeTimer = SIN_TIMER;
      break;
    case WATER:
      moveWater();
      modeTimer = WATER_TIMER;
      break;
  }
  renderCube();
}

void changeMode() {
  clearCube();
  loading = true;
  timer = 0;
  randomSeed(millis());
  currentEffect = (currentEffect + 1) % 4;
}

void renderCube() {
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(SS, LOW);
    SPI.transfer(0x01 << i);
    for (uint8_t j = 0; j < 8; j++) {
      SPI.transfer(cube[i][j]);
    }
    digitalWrite(SS, HIGH);
    delay(1);
  }
}

void mySinus() {
  timer++;
  if (timer > modeTimer) {
    clearCube();
    timer = 0;
    int x = (-1) * (sinus[0] + 1);
    for (int i = 1; i < 8; i++) {
      sinus[i - 1] = sinus[i];
    }
    sinus[7] = x;
    for (uint8_t i = 0; i < 8; i++) {
      for (uint8_t j = 0; j < 8; j++) {
        setVoxel(i, sinus[i] + 4, j);
      }
    }
  }
}

void rain() {
  timer++;
  if (timer > modeTimer) {

    timer = 0;
    shift(NEG_Y);
    uint8_t numDrops = random(0, 5);
    for (uint8_t i = 0; i < numDrops; i++) {
      setVoxel(random(0, 8), 7, random(0, 8));
    }
  }
  delay(20);
}

void all() {
  lightCube();
}

void setVoxel(uint8_t x, uint8_t y, uint8_t z) {
  cube[7 - y][7 - z] |= (0x01 << x);
}

void clearVoxel(uint8_t x, uint8_t y, uint8_t z) {
  cube[7 - y][7 - z] ^= (0x01 << x);
}

bool getVoxel(uint8_t x, uint8_t y, uint8_t z) {
  return (cube[7 - y][7 - z] & (0x01 << x)) == (0x01 << x);
}


void shift(uint8_t dir) {
  if (dir == POS_X) {
    for (uint8_t y = 0; y < 8; y++) {
      for (uint8_t z = 0; z < 8; z++) {
        cube[y][z] = cube[y][z] << 1;
      }
    }
  } else if (dir == NEG_X) {
    for (uint8_t y = 0; y < 8; y++) {
      for (uint8_t z = 0; z < 8; z++) {
        cube[y][z] = cube[y][z] >> 1;
      }
    }
  } else if (dir == POS_Y) {
    for (uint8_t y = 1; y < 8; y++) {
      for (uint8_t z = 0; z < 8; z++) {
        cube[y - 1][z] = cube[y][z];
      }
    }
    for (uint8_t i = 0; i < 8; i++) {
      cube[7][i] = 0;
    }
  } else if (dir == NEG_Y) {
    for (uint8_t y = 7; y > 0; y--) {
      for (uint8_t z = 0; z < 8; z++) {
        cube[y][z] = cube[y - 1][z];
      }
    }
    for (uint8_t i = 0; i < 8; i++) {
      cube[0][i] = 0;
    }
  } else if (dir == POS_Z) {
    for (uint8_t y = 0; y < 8; y++) {
      for (uint8_t z = 1; z < 8; z++) {
        cube[y][z - 1] = cube[y][z];
      }
    }
    for (uint8_t i = 0; i < 8; i++) {
      cube[i][7] = 0;
    }
  } else if (dir == NEG_Z) {
    for (uint8_t y = 0; y < 8; y++) {
      for (uint8_t z = 7; z > 0; z--) {
        cube[y][z] = cube[y][z - 1];
      }
    }
    for (uint8_t i = 0; i < 8; i++) {
      cube[i][0] = 0;
    }
  }
}

void lightCube() {
  for (uint8_t i = 0; i < 8; i++) {
    for (uint8_t j = 0; j < 8; j++) {
      cube[i][j] = 0xFF;
    }
  }
}

void clearCube() {
  for (uint8_t i = 0; i < 8; i++) {
    for (uint8_t j = 0; j < 8; j++) {
      cube[i][j] = 0;
    }
  }
}

void moveWater() {
  timer++;
  updateAngles();
  if (timer > modeTimer) {
    clearCube();
   
    
    for (int i = 0; i < N; i++) {
      int coordXNew = (coordX[i] + gravX);
      int coordYNew = (coordY[i] + gravY);
      int coordZNew = (coordZ[i] + gravZ);
      
      coordXNew = constrain(coordXNew, 0, 7);
      coordYNew = constrain(coordYNew, 0, 7);
      coordZNew = constrain(coordZNew, 0, 7);
     
      
      if (!waterBusy(coordXNew, coordYNew, coordZNew)) {
        updateWaterBusy(i, coordXNew, coordYNew, coordZNew);
      }
    }
    
    for (uint8_t i = 0; i < N; i++) {
        setVoxel(coordX[i], coordY[i], coordZ[i]);  
    }
  }
  delay(30);
}


boolean waterBusy(int i, int j, int k) {
  if (cubeWater[i][j][k] == 1) return true;
  return false; 
}

void updateWaterBusy(int i, int newX, int newY, int newZ) {
  cubeWater[coordX[i]][coordY[i]][coordZ[i]] = 0;
  cubeWater[newX][newY][newZ] = 1;
  coordX[i] = newX;
  coordY[i] = newY; 
  coordZ[i] = newZ;
}

void updateAngles() {
  Wire.beginTransmission(MPU6050_ADRESS);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU6050_ADRESS, 7*2, true); // request a total of 7*2=14 registers

  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  for(byte i=0; i<3; i++) {
    accValue[i] = Wire.read()<<8 | Wire.read(); // reading registers: ACCEL_XOUT, ACCEL_YOUT, ACCEL_ZOUT
  }
  temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  for(byte i=0; i<3; i++) {
    gyroValue[i] = Wire.read()<<8 | Wire.read(); // reading registers: GYRO_XOUT, GYRO_>OUT, GYRO_ZOUT
  }

  Serial.print("Accel: ");
  for (byte i = 0; i < 3; i++) {
    accCorr = accValue[i] - ACCEL_OFFSET;
    accCorr = map(accCorr, -16800, 16800, -90, 90);
    accCorr = constrain(accCorr, -90, 90);
    Serial.print(accCorr);
    Serial.print("\t");
    accAngle[i] = round((double) accCorr / 90);
    Serial.print(accAngle[i]);
    Serial.print("\t");
  }
  Serial.println();
  
  gravX = - accAngle[0];;
  gravZ = accAngle[1];
  gravY = - accAngle[2];
}
