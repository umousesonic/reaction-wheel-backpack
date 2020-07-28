//////////////////////////////////////MPU defines///////////////////////////////////////
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high
//需要int引脚连接到外部中断0

#define LED_PIN 13 // Arduino LED is 13
bool blinkState = false;


//////////////////////////////////////ESCs///////////////////////////////////////////////
#define ROLLMOTORPIN 7
#define PITCHMOTORPIN 8
#define SPEED_MIN 1000
#define SPEED_MAX 2000
#define DEFAULT_SPEED 1500

#define ROLL_MIXER 1
#define PITCH_MIXER -1

#define myThreshold -1000

ESC rollMotor (ROLLMOTORPIN,  SPEED_MIN, SPEED_MAX, DEFAULT_SPEED);
ESC pitchMotor(PITCHMOTORPIN, SPEED_MIN, SPEED_MAX, DEFAULT_SPEED);

int throttle[] = {1500, 1500, 1500};

//////////////////////////////////// MPU control/status vars//////////////////////////////
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
double yaw, pitch, roll;

///////////////////////////////////// MPU Calibration///////////////////////////////////////

#define axOffsetAddress 1
#define ayOffsetAddress 2
#define azOffsetAddress 3
#define gxOffsetAddress 4
#define gyOffsetAddress 5
#define gzOffsetAddress 6

//////////////////////////////////////PID Controller defines///////////////////////////////////
double roll_PID_Accel, pitch_PID_Accel;

double rollKp = 1;
double rollKi = 0;
double rollKd = 0;

double pitchKp = 1;
double pitchKi = 0;
double pitchKd = 0;

PID rollPID (&roll, &roll_PID_Accel,  0, rollKp,  rollKi,  rollKd,  DIRECT);
PID pitchPID(&pitch, &pitch_PID_Accel, 0, pitchKp, pitchKi, pitchKd, DIRECT);




////////////////////////////////// Program Flags & Variables //////////////////////////////////////////
bool isTriggered = false;
double triggerTime = 0;
int startMillis = 0;
unsigned long debugMode = 0;
unsigned long int onTimeMax = 500;
