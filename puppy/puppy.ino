#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>

/////////////////////////////////////////////
//
//
// 以下是接线的参数，按这些进行接线
// PWM控制板的接线参见：16路PWM控制板接线.jpg
//
//

//Constant variables with "-1" as tail means that it controls walking, no need to change; 
const int rightFront1 = 0; 
const int leftFront1 = 1; 
const int leftBack1 = 2; 
const int rightBack1 = 3; 

//Constant variables with "-2" as tail means that it controls standing and sitting, no need to change; 
const int rightFront2 = 4; 
const int leftFront2 = 5; 
const int leftBack2 = 6; 
const int rightBack2 = 7; 

//摇杆接线：Vx接A0，Vy接A1，SW接2端口
const int VxReal = A0, VyReal = A1; 
const int Vx = 0, Vy = 1; 
const int sw = 2; 

//蓝牙模块接线
//注：这里rx和tx指的是uno的串口，是反的，
//蓝牙的RX应该连下面的tx，TX应该连下面的rx！！！
const int rx = 8, tx = 9; 


/////////////////////////////////////////////
//
//
// 以下是调试参数，需要在调试过程中修改
//
//

/////////////////////////////////////////////
//
// 推荐调试过程：
// 1. 先站立，调节SERVO_BEG_n宏的大小使得狗正常站立（该宏不要小于STEP_HEIGHT、MOVE_DISTANCE和SIT_DEPTH，也不要大于4096 - SIT_DEPTH、4096 - STEP_HEIGHT与4096 - MOVE_DISTANCE）
// 2. 再尝试下蹲，观察控制上下的舵机是否与默认的方向是反的，如果反了，调节XX_XX_VERT_REVERSE这4个宏
//    如果蹲的幅度不够或过多，调节STEP_HEIGHT宏的值
// 3. 再尝试转弯，如果有那条腿的控制水平运动的舵机方向反了，如果反了，调节XX_XX_LEVL_REVERSE这4个宏
// 4. 再尝试前进和后退，如果步幅过大或过小，调节MOVE_DISTANCE宏
//    如果一条腿移动之后停顿过长，调节ONE_LEG_MOVE_TIME和STEP_INTERVAL宏（优先调节STEP_INTERVAL）
// 5. 如果一切顺利，把__DEBUG__改成0，完工
//
//


/////////////////////////////////////////////
//
//控制方式：
//蓝牙模块：u是站立，d是下蹲，f是前进，b是后退，l是左转，r是右转
//摇杆模块：按下切换站立与坐下，前后左右调整前进、后腿、左转、右转
//
//

//这个宏在调试的时候定义成1，可以在串口监视器显示信息，调试完毕后改成0
#define __DEBUG__  1 


//下面这8个宏代表是否有哪个舵机的安装方向和我写代码默认的方向是反的，如果反了，把对应的宏改成1
//下面这4个表示控制狗腿水平方向摆动的舵机，默认方向是正方向移动是两条前腿向前，两条后腿向后
#define RIGHT_FRONT_LEVL_REVERSE 0        //右前腿控制水平移动的，后同
#define LEFT_FRONT_LEVL_REVERSE 0
#define LEFT_BACK_LEVL_REVERSE 0
#define RIGHT_BACK_LEVL_REVERSE 0
//下面这4个宏表示控制狗腿竖直方向移动的舵机，默认方向是正方向移动狗腿抬起
#define RIGHT_FRONT_VERT_REVERSE 0        //右前腿控制竖直移动的
#define LEFT_FRONT_VERT_REVERSE 0
#define LEFT_BACK_VERT_REVERSE 0
#define RIGHT_BACK_VERT_REVERSE 0

//SERVO_BEG_n表示当狗正常站立时n引脚的参数（数值越大舵机摆动角度越大），须大于零，需要手动调整
#define SERVO_BEG_0 375
#define SERVO_BEG_1 375
#define SERVO_BEG_2 375
#define SERVO_BEG_3 375
#define SERVO_BEG_4 400
#define SERVO_BEG_5 400
#define SERVO_BEG_6 400
#define SERVO_BEG_7 400

//一条腿移动的时间
#define ONE_LEG_MOVE_TIME 150

//走路时腿抬起的高度
#define STEP_HEIGHT 70

//下蹲的幅度
#define SIT_DEPTH 100

//走路时一步向前迈的距离
#define MOVE_DISTANCE 80

//走路时每迈一步的间隔时间
#define STEP_INTERVAL 200



///////////////////////////////////////
//
//
// 以下是常量和全局变量，不能修改
//
//


// Use default i2c address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
SoftwareSerial sws(rx, tx); 

//LEG代号，不需要更改
typedef enum
{
  RIGHT_FRONT = 0, 
  LEFT_FRONT = 1, 
  LEFT_BACK = 2, 
  RIGHT_BACK = 3
} LegType; 

int vertLeg = 0, levlLeg = 0, servoBegLevl = 0, servoBegVert = 0; 
int dogState;   //狗的状态
unsigned char msg;  //蓝牙模块接收到的消息

//狗的状态
#define DOG_STAND 0
#define DOG_SIT 1

//行进方向，不需要调整
#define FORWARD 1
#define BACKWARD -1

//舵机控制板PWM频率，不需要调整
#define PWM_FREQUENCY 60


#if RIGHT_FRONT_LEVL_REVERSE
#define DEAL_RIGHT_FRONT_LEVL(x) ((x) = -(x))
#else
#define DEAL_RIGHT_FRONT_LEVL(x) (x)
#endif

#if LEFT_FRONT_LEVL_REVERSE
#define DEAL_LEFT_FRONT_LEVL(x) ((x) = -(x))
#else
#define DEAL_LEFT_FRONT_LEVL(x) (x)
#endif

#if LEFT_BACK_LEVL_REVERSE
#define DEAL_LEFT_BACK_LEVL(x) ((x) = -(x))
#else
#define DEAL_LEFT_BACK_LEVL(x) (x)
#endif

#if RIGHT_BACK_LEVL_REVERSE
#define DEAL_RIGHT_BACK_LEVL(x) ((x) = -(x))
#else
#define DEAL_RIGHT_BACK_LEVL(x) (x)
#endif

#if RIGHT_FRONT_VERT_REVERSE
#define DEAL_RIGHT_FRONT_VERT(x) ((x) = -(x))
#else
#define DEAL_RIGHT_FRONT_VERT(x) (x)
#endif

#if LEFT_FRONT_VERT_REVERSE
#define DEAL_LEFT_FRONT_VERT(x) ((x) = -(x))
#else
#define DEAL_LEFT_FRONT_VERT(x) (x)
#endif

#if LEFT_BACK_VERT_REVERSE
#define DEAL_LEFT_BACK_VERT(x) ((x) = -(x))
#else
#define DEAL_LEFT_BACK_VERT(x) (x)
#endif

#if RIGHT_BACK_VERT_REVERSE
#define DEAL_RIGHT_BACK_VERT(x) ((x) = -(x))
#else
#define DEAL_RIGHT_BACK_VERT(x) (x)
#endif



////////////////////////////////////
//
//
// 以下是函数
//
//



//运动某只腿
void MoveLeg(LegType leg, int vertArg, int levlArg, int direct); 
//站立/复位
void stand(); 
//下蹲
void sit(); 
//向前走
void moveForward(); 
//向后走
void moveBackward(); 
//向左转
void TurnLeft(); 
//向右转
void TurnRight(); 

void setup()
{
  Serial.begin(9600); 
  pwm.begin(); 
  pwm.setPWMFreq(PWM_FREQUENCY); 

  //初始化四条腿
  stand(); 
  dogState = DOG_STAND; 

  //初始化摇杆
  pinMode(sw, INPUT_PULLUP); 

  //初始化蓝牙模块
  sws.begin(9600); 
}

void loop()
{
  //先看蓝牙
  if (sws.available())
  {
    msg = sws.read(); 

#if __DEBUG__

    Serial.print("Get message: "); 
    Serial.println(msg); 

#endif  //__DEBUG__

    switch(msg)
    {
    case 'd':   //坐下
      if (dogState != DOG_SIT)
      {
        dogState = DOG_SIT; 
        sit(); 
        delay(1000); 
      }
    return; 
    case 'u':    //站起来
      if (dogState != DOG_STAND)
      {
        dogState = DOG_STAND; 
        stand(); 
        delay(1000); 
      }
    return; 
    case 'l':   //左转
      TurnLeft(); 
    return; 
    case 'r':   //右转
      TurnRight(); 
    return; 
    case 'f':   //前进
      moveForward(); 
    return; 
    case 'b':   //后退
      moveBackward(); 
    return; 
    }

  }
  //检查摇杆按钮
  int swState = digitalRead(sw); 
  if (swState == LOW)
  {
    delay(20); 
    if (swState == LOW)
    {
      if (dogState == DOG_STAND)
      {
        dogState = DOG_SIT; 
        sit(); 
      }
      else if (dogState == DOG_SIT)
      {
        dogState = DOG_STAND; 
        stand(); 
      }
      delay(1000);      //不能连续使他站起坐下
      return; 
    }
  }

  //如果狗在做着，不能行走
  if (dogState == DOG_SIT) return; 
  //检查摇杆
  int xVal = analogRead(Vx); 
  int yVal = analogRead(Vy); 
  bool xPos = (xVal > 1000), xNeg = (xVal < 20); 
  bool yPos = (yVal > 1000), yNeg = (yVal < 20); 
  if (xPos && !yPos && !yNeg)
  {
    moveForward(); 
  }
  else if (xNeg && !yPos && !yNeg)
  {
    moveBackward(); 
  }
  else if (yNeg && !xPos && !xNeg)
  {
    TurnLeft(); 
  }
  else if (yPos && !xPos && !xNeg)
  {
    TurnRight(); 
  }
}

void stand()
{
  pwm.setPWM(rightFront2, 0, SERVO_BEG_4); 
  pwm.setPWM(leftFront2, 0, SERVO_BEG_5); 
  pwm.setPWM(leftBack2, 0, SERVO_BEG_6); 
  pwm.setPWM(rightBack2, 0, SERVO_BEG_7); 
  pwm.setPWM(rightFront1, 0, SERVO_BEG_0); 
  pwm.setPWM(leftFront1, 0, SERVO_BEG_1); 
  pwm.setPWM(leftBack1, 0, SERVO_BEG_2); 
  pwm.setPWM(rightBack1, 0, SERVO_BEG_3); 
}

//下蹲
void sit()
{
  //四条腿调到正常位置
  pwm.setPWM(rightFront1, 0, SERVO_BEG_0); 
  pwm.setPWM(leftFront1, 0, SERVO_BEG_1); 
  pwm.setPWM(leftBack1, 0, SERVO_BEG_2); 
  pwm.setPWM(rightBack1, 0, SERVO_BEG_3); 

  //蹲下
  pwm.setPWM(rightFront2, 0, SERVO_BEG_4 + SIT_DEPTH); 
  pwm.setPWM(leftFront2, 0, SERVO_BEG_5 + SIT_DEPTH); 
  pwm.setPWM(leftBack2, 0, SERVO_BEG_6 + SIT_DEPTH); 
  pwm.setPWM(rightBack2, 0, SERVO_BEG_7 + SIT_DEPTH); 
}

//运动某只腿
void MoveLeg(LegType leg, int vertArg, int levlArg, int direct)
{
  switch(leg)
  {
  case LegType::RIGHT_FRONT: 
    levlLeg = rightFront1; vertLeg = rightFront2; 
    servoBegLevl = SERVO_BEG_0; servoBegVert = SERVO_BEG_4; 
    DEAL_RIGHT_FRONT_LEVL(levlArg); 
    DEAL_RIGHT_FRONT_VERT(vertArg); 
  break; 
  case LegType::LEFT_FRONT: 
    levlLeg = leftFront1; vertLeg = leftFront2; 
    servoBegLevl = SERVO_BEG_1; servoBegVert = SERVO_BEG_5; 
    DEAL_LEFT_FRONT_LEVL(levlArg); 
    DEAL_LEFT_FRONT_VERT(vertArg); 
  break; 
  case LegType::LEFT_BACK: 
    levlLeg = leftBack1; vertLeg = leftBack2; 
    servoBegLevl = SERVO_BEG_2; servoBegVert = SERVO_BEG_6; 
    levlArg = -levlArg; 
    DEAL_LEFT_BACK_LEVL(levlArg); 
    DEAL_LEFT_BACK_VERT(vertArg); 
  break; 
  case LegType::RIGHT_BACK: 
    levlLeg = rightBack1; vertLeg = rightBack2; 
    servoBegLevl = SERVO_BEG_3; servoBegVert = SERVO_BEG_7; 
    levlArg = -levlArg; 
    DEAL_RIGHT_BACK_LEVL(levlArg); 
    DEAL_RIGHT_BACK_VERT(vertArg); 
  break; 
  }
  //抬腿
  pwm.setPWM(vertLeg, 0, servoBegVert + vertArg); 
  //前行
  pwm.setPWM(levlLeg, 0, servoBegLevl + levlArg * direct); 
  delay(ONE_LEG_MOVE_TIME);    //等待移动
  //落腿
  pwm.setPWM(vertLeg, 0, servoBegVert); 
}

//向前走
void moveForward()
{
  //左前腿归位
  MoveLeg(LegType::LEFT_FRONT, STEP_HEIGHT, 0, FORWARD); 
  delay(STEP_INTERVAL / 8); 
  //右后腿向前迈
  MoveLeg(LegType::RIGHT_BACK, STEP_HEIGHT, MOVE_DISTANCE, FORWARD); 
  delay(STEP_INTERVAL / 4); 
  //左前腿向后抓地爬行
  MoveLeg(LegType::LEFT_FRONT, 0, MOVE_DISTANCE, BACKWARD); 
  delay(STEP_INTERVAL / 8); 
  //右后腿向后抓地爬行
  MoveLeg(LegType::RIGHT_BACK, 0, 0, BACKWARD); 
  delay(STEP_INTERVAL / 8); 
  
  //右前腿归位
  MoveLeg(LegType::RIGHT_FRONT, STEP_HEIGHT, 0, FORWARD); 
  delay(STEP_INTERVAL / 8); 
  //左后腿向前迈
  MoveLeg(LegType::LEFT_BACK, STEP_HEIGHT, MOVE_DISTANCE, FORWARD); 
  delay(STEP_INTERVAL / 4); 
  //右前腿向后抓地爬行
  MoveLeg(LegType::RIGHT_FRONT, 0, MOVE_DISTANCE, BACKWARD); 
  delay(STEP_INTERVAL / 8); 
  //左后腿向后抓地爬行
  MoveLeg(LegType::LEFT_BACK, 0, 0, BACKWARD); 
  delay(STEP_INTERVAL / 8); 
}

//向后走
void moveBackward()
{
  //右后腿归位
  MoveLeg(LegType::RIGHT_BACK, STEP_HEIGHT, 0, BACKWARD); 
  delay(STEP_INTERVAL / 8); 
  //左前腿向后迈
  MoveLeg(LegType::LEFT_FRONT, STEP_HEIGHT, MOVE_DISTANCE, BACKWARD); 
  delay(STEP_INTERVAL / 4); 
  //右后腿向前抓地爬行
  MoveLeg(LegType::RIGHT_BACK, 0, MOVE_DISTANCE, FORWARD); 
  delay(STEP_INTERVAL / 8); 
  //左前腿向后抓地爬行
  MoveLeg(LegType::LEFT_FRONT, 0, 0, FORWARD); 
  delay(STEP_INTERVAL / 8); 
  
  //左后腿归位
  MoveLeg(LegType::LEFT_BACK, STEP_HEIGHT, 0, BACKWARD); 
  delay(STEP_INTERVAL / 8); 
  //右前腿向后迈
  MoveLeg(LegType::RIGHT_FRONT, STEP_HEIGHT, MOVE_DISTANCE, BACKWARD); 
  delay(STEP_INTERVAL / 4); 
  //左后腿向前抓地爬行
  MoveLeg(LegType::LEFT_BACK, 0, MOVE_DISTANCE, FORWARD); 
  delay(STEP_INTERVAL / 8); 
  //右前腿向前抓地爬行
  MoveLeg(LegType::RIGHT_FRONT, 0, 0, FORWARD); 
  delay(STEP_INTERVAL / 8); 
}

//向左转
void TurnLeft()
{
  MoveLeg(LegType::LEFT_FRONT, STEP_HEIGHT, MOVE_DISTANCE, BACKWARD);   //左前爪
  delay(STEP_INTERVAL / 2); 
  MoveLeg(LegType::RIGHT_BACK, STEP_HEIGHT, MOVE_DISTANCE, FORWARD);   //右后爪
  delay(STEP_INTERVAL / 2); 
  MoveLeg(LegType::LEFT_BACK, STEP_HEIGHT, MOVE_DISTANCE, BACKWARD);   //左后爪
  delay(STEP_INTERVAL / 2); 
  MoveLeg(LegType::RIGHT_FRONT, STEP_HEIGHT, MOVE_DISTANCE, FORWARD);   //右前爪
  delay(STEP_INTERVAL / 2); 
  //归正
  pwm.setPWM(rightFront1, 0, SERVO_BEG_0); 
  pwm.setPWM(leftFront1, 0, SERVO_BEG_1); 
  pwm.setPWM(leftBack1, 0, SERVO_BEG_2); 
  pwm.setPWM(rightBack1, 0, SERVO_BEG_3); 
  delay(STEP_INTERVAL / 2); 
}

//向右转
void TurnRight()
{
  MoveLeg(LegType::LEFT_FRONT, STEP_HEIGHT, MOVE_DISTANCE, FORWARD);   //左前爪
  delay(STEP_INTERVAL / 2); 
  MoveLeg(LegType::RIGHT_BACK, STEP_HEIGHT, MOVE_DISTANCE, BACKWARD);   //右后爪
  delay(STEP_INTERVAL / 2); 
  MoveLeg(LegType::LEFT_BACK, STEP_HEIGHT, MOVE_DISTANCE, FORWARD);   //左后爪
  delay(STEP_INTERVAL / 2); 
  MoveLeg(LegType::RIGHT_FRONT, STEP_HEIGHT, MOVE_DISTANCE, BACKWARD);   //右前爪
  delay(STEP_INTERVAL / 2); 
  //归正
  pwm.setPWM(rightFront1, 0, SERVO_BEG_0); 
  pwm.setPWM(leftFront1, 0, SERVO_BEG_1); 
  pwm.setPWM(leftBack1, 0, SERVO_BEG_2); 
  pwm.setPWM(rightBack1, 0, SERVO_BEG_3); 
  delay(STEP_INTERVAL / 2); 
}
