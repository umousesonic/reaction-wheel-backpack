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

  yaw   = double(ypr[0]* 180/M_PI);
  pitch = double(ypr[1]* 180/M_PI);
  roll  = double(ypr[2]* 180/M_PI);
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

void usePretestedOffset() {
  mpu.setXGyroOffset(-227);
  mpu.setYGyroOffset(7);
  mpu.setZGyroOffset(-8);
  mpu.setXAccelOffset(-4422);
  mpu.setYAccelOffset(150);
  mpu.setZAccelOffset(518);
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
  else if (command == "debugmode") { return 10; }
  else if (command == "off") { return 11; }
  else if (command == "on") { return 12; }
  else if (command == "throttle") { return 13; }
  else if (command == "pid_accel") { return 14; }
  else if (command == "z_accel") { return 15; }
  else if (command == "onTimeMax") { return 16; }
  else if (command == "ypr") { return 17; }
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
          rollPID.SetTunings(rollKp, rollKi, rollKd);
          Serial.println("rollKp is set to: " + String(rollPID.GetKp(), DEC));
          break;
        case 4: // "ri"
          rollKi = serialReadWord().toDouble();
          rollPID.SetTunings(rollKp, rollKi, rollKd);
          Serial.println("rollKi is set to: " + String(rollPID.GetKi(), DEC));
          break;
        case 5: // "rd"
          rollKd = serialReadWord().toDouble();
          rollPID.SetTunings(rollKp, rollKi, rollKd);
          Serial.println("rollKd is set to: " + String(rollPID.GetKd(), DEC));
          break;
        case 6: // "pp"
          pitchKp = serialReadWord().toDouble();
          pitchPID.SetTunings(pitchKp, pitchKi, pitchKd);
          Serial.println("pitchKp is set to: " + String(pitchPID.GetKp(), DEC));
          break;
        case 7: // "pi"
          pitchKi = serialReadWord().toDouble();
          pitchPID.SetTunings(pitchKp, pitchKi, pitchKd);
          Serial.println("pitchKi is set to: " + String(pitchPID.GetKi(), DEC));
          break;
        case 8: // "pd"
          pitchKd = serialReadWord().toDouble();
          pitchPID.SetTunings(pitchKp, pitchKi, pitchKd);
          Serial.println("pitchKd is set to: " + String(pitchPID.GetKd(), DEC));
          break;
        case 16:  // "onTimeMax"
          onTimeMax = strtoul(serialReadWord().c_str(), NULL, 10);
          Serial.println("onTimeMax is set to: " + String(onTimeMax));
          break;
      }
      break;

    case 2: // "get"
      switch (serialReadWordInt()){
        default:
          break;
        case 3: // "rp"
          Serial.println("rollKp is: " + String(rollPID.GetKp(), DEC));
          break;
        case 4: // "ri"
          Serial.println("rollKi is: " + String(rollPID.GetKi(), DEC));
          break;
        case 5: // "rd"
          Serial.println("rollKd is: " + String(rollPID.GetKd(), DEC));
          break;
        case 6: // "pp"
          Serial.println("pitchKp is: " + String(pitchPID.GetKp(), DEC));
          break;
        case 7: // "pi"
          Serial.println("pitchKi is: " + String(pitchPID.GetKi(), DEC));
          break;
        case 8: // "pd"
          Serial.println("pitchKd is: " + String(pitchPID.GetKd(), DEC));
          break;
        case 9: // "pid"
          Serial.println("\t|\tP\t|\tI\t|\tD\t|\nroll:\t|\t"+String(rollPID.GetKp(), 2)+"\t|\t"+String(rollPID.GetKi(), 2)+"\t|\t"+String(rollPID.GetKd(), 2)+"\t|\t\napitch:\t|\t"+String(pitchPID.GetKp(), 2)+"\t|\t"+String(pitchPID.GetKi(), 2)+"\t|\t"+String(pitchPID.GetKd(), 2)+"\t|\t");
      }
       break;
      
     case 10:  // "debugmode"
       switch(serialReadWordInt()) {
         default:
           break;
          case 11:  // "off"
            debugMode = 0;
            break;
          case 9: // "pid"
            switch (serialReadWordInt()) {
              default: break;
              case 11: bitWrite(debugMode, 0, B0); break;
              case 12: bitWrite(debugMode, 0, B1); break;
            }
            break;
          case 13:  // "throttle"
            switch (serialReadWordInt()) {
              default: break;
              case 11: bitWrite(debugMode, 1, B0); break;
              case 12: bitWrite(debugMode, 1, B1); break;
            }
            break;
          case 14:  // "pid_accel"
            switch (serialReadWordInt()) {
              default: break;
              case 11: bitWrite(debugMode, 2, B0); break;
              case 12: bitWrite(debugMode, 2, B1); break;
            }
            break;
          case 15:  // "z_accel"
            switch (serialReadWordInt()) {
              default: break;
              case 11: bitWrite(debugMode, 3, B0); break;
              case 12: bitWrite(debugMode, 3, B1); break;
            }
            break;
          case 17:  // "ypr"
            switch (serialReadWordInt()) {
              default: break;
              case 11: bitWrite(debugMode, 4, B0); break;
              case 12: bitWrite(debugMode, 4, B1); break;
            }
            break;
       }
       break;
  }
}

void debugFunction() {
  for (int i = 31; i >=0; i--) {
    if (bitRead(debugMode, i)) {
      switch (i){
        default: break;
        case 0: Serial.println("\t|\tP\t|\tI\t|\tD\t|\nroll:\t|\t"+String(rollKp, 2)+"\t|\t"+String(rollKi, 2)+"\t|\t"+String(rollKd, 2)+"\t|\t\napitch:\t|\t"+String(pitchKp, 2)+"\t|\t"+String(pitchKi, 2)+"\t|\t"+String(pitchKd, 2)+"\t|\t");
                break;
        case 1: Serial.println("Throttle:\tr:" + String(throttle[2]) + "\tp:" + String(throttle[1])); break;
        case 2: Serial.println("pid_accel:\tr:" + String(roll_PID_Accel) + "\tp:" + String(pitch_PID_Accel)); break;
        case 3: Serial.println("z_accel:" + String(aaWorld.z)); break;
        case 4: Serial.println("yaw:" + String(yaw) + "\tpitch:" + String(pitch) + "\troll:" + String(roll)); break;
      }
    }
  }
}
