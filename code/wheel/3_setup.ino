
void setup() {
    //arm ESCs
    rollMotor.arm();
    pitchMotor.arm();
    delay(5000);
    
    
    // 加入i2c总线 (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // 初始化串口
    // NOTE: 8MHz以下的设备跑不动115200串口，需要38400等才能稳定
    Serial.begin(115200);
    //while (!Serial); // 等待Leonardo的串口枚举，其他的直接通过
    

    // 初始化mpu
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // 等待开始指令
    Serial.println(F("\nSend any character to begin: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again



    // 加载及配置DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    
    // factory offset
    //useFactoryOffset();
    //useCalibOffset();
    usePretestedOffset();

    // 开启DMP及确认DMP正常 (returns 0 if so)
    if (devStatus == 0) {
        // 开启DMP
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // 设置arduino中断
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // 设置DMP Ready flag for main loop() 
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // 获取DMP数据包
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);


    // Setup PID controller
    rollPID.SetSampleTime(100);
    rollPID.SetOutputLimits(-500.0,500.0);
    rollPID.SetMode(AUTOMATIC);
    
    pitchPID.SetSampleTime(100);
    pitchPID.SetOutputLimits(-500.0,500.0);
    pitchPID.SetMode(AUTOMATIC);

    startMillis = millis();
    

    
}
