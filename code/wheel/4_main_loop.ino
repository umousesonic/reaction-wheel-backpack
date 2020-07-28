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

        // calculate ypr
        getYpr();

        // 状态灯
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);

        
        if ((fallDetect(myThreshold) || isTriggered) && millis() > startMillis + 3000) {
          if (!isTriggered) {
            isTriggered = true;
            triggerTime = millis();
            for (int i = 0; i <= 2; i++) {
              throttle[i] = 1500;
            }
          }
          if (millis() >= triggerTime + onTimeMax) {
            isTriggered = false;
            rollMotor.speed(1500);
            pitchMotor.speed(1500);
            delay(10);
          } else {
            
            
            // calculate PID
            if (rollPID.Compute()) {
              throttle[2] += roll_PID_Accel * ROLL_MIXER;
            }
            if (pitchPID.Compute()) {
              throttle[1] += pitch_PID_Accel * PITCH_MIXER;
            }

            for (int i = 0; i <= 2; i++) {
              throttle[i] = throttle[i] > 2000 ? 2000 : throttle[i];
              throttle[i] = throttle[i] < 1000 ? 1000 : throttle[i];
            }
            

            // output speed
            rollMotor.speed(throttle[2]);
            pitchMotor.speed(throttle[1]);

            //debug
            Serial.println("Triggered!");
          }        
      }
    }
    doSerialCommands();
    Serial.flush();
    debugFunction();
    //Serial.println(aaWorld.z);
    
    
}
