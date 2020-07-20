// ================================================================
// =                         中断函数                              =
// ================================================================

volatile bool mpuInterrupt = false;     // 中断发生过的flag
void dmpDataReady() {
    mpuInterrupt = true;
}





// 其他函数

void getWorldAccel() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
}

void getYpr() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  for (int i=0; i<sizeof(ypr)-1; i++){
    ypr_input[i] = double(ypr[i]* 180/M_PI);
  }
}

bool fallDetect(int threshold) {
  getWorldAccel();
  if (aaWorld.z <= threshold) {
    return true;
    }
  else {
    return false;
    }
}


/////////////////////////////////////////calibration mpu6050////////////////////////////////////////
void useCalibOffset() {
  mpu.setXGyroOffset(EEPROM.read(gxOffsetAddress));
  mpu.setYGyroOffset(EEPROM.read(gyOffsetAddress));
  mpu.setZGyroOffset(EEPROM.read(gzOffsetAddress));
  mpu.setXAccelOffset(EEPROM.read(axOffsetAddress));
  mpu.setYAccelOffset(EEPROM.read(ayOffsetAddress));
  mpu.setZAccelOffset(EEPROM.read(azOffsetAddress));
}

void useFactoryOffset() {
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
}

/////////////////////////////////////// Serial commands /////////////////////////////////////////////
String serialReadWord() {
  char currentChar;
  String output = "";
  while(1) {
    currentChar = Serial.read();
    if (currentChar == ' ' || !Serial.available()) {
      break;
    }
    else {
      output += currentChar;
    }
  }
  return output;
}

int serialReadWordInt() {
  String command = serialReadWord();
  if (command == "set") { return 1; }
  else if (command == "get") { return 2;}
  else if (command == "rp") { return 3; }
  else if (command == "ri") { return 4; }
  else if (command == "rd") { return 5; }
  else if (command == "pp") { return 6; }
  else if (command == "pi") { return 7; }
  else if (command == "pd") { return 8; }
  else if (command == "pid") { return 9;}
  else { return 0;}
}

void doSerialCommands() {
  switch (serialReadWordInt()) {
    default:
      break;
    case 1: // "set"
      switch (serialReadWordInt()){
        default:
          break;
        case 3: // "rp"
          rollKp = serialReadWord().toDouble();
          Serial.println("rollKp is set to: " + String(rollKp, DEC));
          break;
        case 4: // "ri"
          rollKi = serialReadWord().toDouble();
          Serial.println("rollKi is set to: " + String(rollKi, DEC));
          break;
        case 5: // "rd"
          rollKd = serialReadWord().toDouble();
          Serial.println("rollKd is set to: " + String(rollKd, DEC));
          break;
        case 6: // "pp"
          pitchKp = serialReadWord().toDouble();
          Serial.println("pitchKp is set to: " + String(pitchKp, DEC));
          break;
        case 7: // "pi"
          pitchKi = serialReadWord().toDouble();
          Serial.println("pitchKi is set to: " + String(pitchKi, DEC));
          break;
        case 8: // "pd"
          pitchKd = serialReadWord().toDouble();
          Serial.println("pitchKd is set to: " + String(pitchKd, DEC));
          break;
      }
      break;

    case 2: // "get"
      switch (serialReadWordInt()){
        default:
          break;
        case 3: // "rp"
          Serial.println("rollKp is: " + String(rollKp, DEC));
          break;
        case 4: // "ri"
          Serial.println("rollKi is: " + String(rollKi, DEC));
          break;
        case 5: // "rd"
          Serial.println("rollKd is: " + String(rollKd, DEC));
          break;
        case 6: // "pp"
          Serial.println("pitchKp is: " + String(pitchKp, DEC));
          break;
        case 7: // "pi"
          Serial.println("pitchKi is: " + String(pitchKi, DEC));
          break;
        case 8: // "pd"
          Serial.println("pitchKd is: " + String(pitchKd, DEC));
          break;
        case 9: // "pid"
          Serial.println("\t|\tP\t|\tI\t|\tD\t|\nroll:\t|\t"+String(rollKp, 2)+"\t|\t"+String(rollKi, 2)+"\t|\t"+String(rollKd, 2)+"\t|\t\napitch:\t|\t"+String(pitchKp, 2)+"\t|\t"+String(pitchKi, 2)+"\t|\t"+String(pitchKd, 2)+"\t|\t");
      }
      break;
  }
}
