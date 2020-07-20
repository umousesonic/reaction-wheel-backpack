// ================================================================
// =                           主循环                              =
// ================================================================

void loop() {
    // failsafe
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
      
    }

    // 重置中断flag和刷新中断状态
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // 检查堆栈溢出 (执行效率太烂才会发生)
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // 没发生的话去处理模块传回来的数据
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        // 状态灯
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);

        
        if (fallDetect(myThreshold) || isTriggered) {
          if (!isTriggered) {
            isTriggered = true;
            triggerTime = millis();
            for (int i = 0; i <= 2; i++) {
              throttle[i] = 1000;
            }
          }
          if (millis() >= triggerTime + onTimeMax) {
            isTriggered = false;
            rollMotor.speed(1000);
            pitchMotor.speed(1000);
            delay(10);
          } else {
            // calculate ypr
            getYpr();
            
            // calculate PID
            if (rollPID.Compute()) {
              throttle[2] += roll_PID_Accel;
            }
            if (pitchPID.Compute()) {
              throttle[1] += pitch_PID_Accel;
            }

            for (int i = 0; i <= 2; i++) {
              throttle[i] = throttle[i] > 2000 ? 2000 : throttle[i];
              throttle[i] = throttle[i] < 1000 ? 1000 : throttle[i];
            }

            /*
            Serial.print("Triggered!\tpitch:");
            Serial.print(throttle[1]);
            Serial.print("\troll:");
            Serial.println(throttle[2]);
            */

            

            // output speed
            rollMotor.speed(throttle[2]);
            pitchMotor.speed(throttle[1]);
          }        
      }
    }
    doSerialCommands();
    Serial.flush();
    //Serial.println(aaWorld.z);
    
    
}
