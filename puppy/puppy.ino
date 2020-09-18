#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>
#include <DHT.h>
#include "U8glib.h"
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE|U8G_I2C_OPT_DEV_0);

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
//const int rx = 8, tx = 9; 
const int rx = 10,tx = 11;
//
//
//烟雾传感器
const int mq2 = A2;
int smoke_density = 0;
int threshold = 500;
//
//
//蜂鸣器
const int buzzer = 3;
//
//存储温度、湿度的临时变量
int temper = 0, humidi = 0;
char temp_char[10], humi_char[10];
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
#define RIGHT_FRONT_LEVL_REVERSE 1        //右前腿控制水平移动的，后同
#define LEFT_FRONT_LEVL_REVERSE 0
#define LEFT_BACK_LEVL_REVERSE 0
#define RIGHT_BACK_LEVL_REVERSE 1
//下面这4个宏表示控制狗腿竖直方向移动的舵机，默认方向是正方向移动狗腿抬起
#define RIGHT_FRONT_VERT_REVERSE 1        //右前腿控制竖直移动的
#define LEFT_FRONT_VERT_REVERSE 0
#define LEFT_BACK_VERT_REVERSE 0
#define RIGHT_BACK_VERT_REVERSE 1

//SERVO_BEG_n表示当狗正常站立时n引脚的参数（数值越大舵机摆动角度越大），须大于零，需要手动调整
#define SERVO_BEG_0 300
#define SERVO_BEG_1 300
#define SERVO_BEG_2 300
#define SERVO_BEG_3 300
#define SERVO_BEG_4 362     //减小抬高
#define SERVO_BEG_5 350     //增大抬高
#define SERVO_BEG_6 320     //减小抬高
#define SERVO_BEG_7 285     //增大抬高

//舵机角度最大值
#define MAX_VAL 600

//一条腿移动的时间
#define ONE_LEG_MOVE_TIME 100

//走路时腿抬起的高度
#define STEP_HEIGHT 80

//下蹲的幅度
#define SIT_DEPTH 100

//走路时一步向前迈的距离
#define MOVE_DISTANCE 60

//走路时每迈一步的间隔时间
#define STEP_INTERVAL 500   //origin:200



///////////////////////////////////////
//
//
// 以下是常量和全局变量，不能修改
//
//
//dht模块的引脚与型号
#define DHT_PIN 12
#define DHT_TYPE DHT11

DHT mydht(DHT_PIN,DHT_TYPE);

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



//运动某只腿，返回运动的水平角度
int MoveLeg(LegType leg, int vertArg, int levlArg, int direct); 
//站立/复位
void stand(); 
//站起一半
void standHalf(); 
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
  pinMode(mq2, INPUT);
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer,LOW);    //高电平触发buzzer
  mydht.begin();
  pwm.begin(); 
  pwm.setPWMFreq(PWM_FREQUENCY); 

  //初始化四条腿
  stand(); 
  dogState = DOG_STAND; 

  //初始化摇杆
  pinMode(sw, INPUT_PULLUP); 
  u8g.setFont(u8g_font_unifont);
  //初始化蓝牙模块
  sws.begin(9600); 
}

void loop()
{
     u8g.firstPage();
    do{
        u8g.drawStr(0,12,"Hum");
        u8g.drawStr(50,12,"Tem");
        u8g.drawStr(0,40,humi_char);  //湿度
        u8g.drawStr(50,40,temp_char);     //温度
    }
    while(u8g.nextPage());
    smoke_density = analogRead(mq2);
    temper = mydht.readTemperature();
    humidi = mydht.readHumidity();
    itoa(temper,temp_char,10);
    itoa(humidi,humi_char,10);
    if(smoke_density > threshold)
        digitalWrite(buzzer,HIGH);
    else digitalWrite(buzzer,LOW);
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
        if (dogState == DOG_SIT) standHalf(); 
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
    case 't':
        sws.print("temperature: ");
        sws.print(temper);
        sws.println(" Celsius degree");
     return;
    case 'h':
        sws.print("humidity: ");
        sws.print(humidi);
        sws.println(" %");
     return;
     case 's':
        sws.print("smoke density: ");
        sws.println(smoke_density);
    return;
    case 'y':
        sws.print("threshold of smoke: ");
        sws.println(threshold);
    return;
    case 'i':
        threshold += 5;
        threshold = threshold < 800 ? threshold : 800;
        sws.print("threshold has been changed as ");
        sws.println(threshold);
    return;
    case 'e':
        threshold -= 5;
        threshold = threshold > 100? threshold : 100;
        sws.print("threshold has been changed as ");
        sws.println(threshold);
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
        dogState = DOG_STAND; 
        //sit();
        stand();  
      }
      else if (dogState == DOG_SIT)
      {
        dogState = DOG_STAND; 
        standHalf(); 
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
  bool xPos = (xVal > 800), xNeg = (xVal < 200); 
  bool yPos = (yVal > 800), yNeg = (yVal < 200); 


  
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
  delay(50); 
}

void stand()
{
  pwm.setPWM(rightFront2, 0, SERVO_BEG_4); 
  pwm.setPWM(leftBack2, 0, SERVO_BEG_6); 
  pwm.setPWM(leftFront2, 0, SERVO_BEG_5); 
  pwm.setPWM(rightBack2, 0, SERVO_BEG_7); 
  pwm.setPWM(rightFront1, 0, SERVO_BEG_0); 
  pwm.setPWM(leftBack1, 0, SERVO_BEG_2); 
  pwm.setPWM(leftFront1, 0, SERVO_BEG_1); 
  pwm.setPWM(rightBack1, 0, SERVO_BEG_3); 
}

void standHalf()
{
  for (int i = 15; i >= 1; --i)
  {
    pwm.setPWM(rightFront2, 0, SERVO_BEG_4 - (SIT_DEPTH >> 4) * i); 
    pwm.setPWM(rightBack2, 0, SERVO_BEG_7 + (SIT_DEPTH >> 4) * i);
    pwm.setPWM(leftBack2, 0, SERVO_BEG_6 - (SIT_DEPTH >> 4) * i); 
    pwm.setPWM(leftFront2, 0, SERVO_BEG_5 + (SIT_DEPTH >> 4) * i); 
    delay(60); 
  }
  /*pwm.setPWM(rightFront2, 0, SERVO_BEG_4 - (SIT_DEPTH >> 2)); 
  pwm.setPWM(leftBack2, 0, SERVO_BEG_6 - (SIT_DEPTH >> 2)); 
  pwm.setPWM(leftFront2, 0, SERVO_BEG_5 + (SIT_DEPTH >> 2)); 
  pwm.setPWM(rightBack2, 0, SERVO_BEG_7 + (SIT_DEPTH >> 2));*/
  
  pwm.setPWM(rightFront1, 0, SERVO_BEG_0); 
  pwm.setPWM(leftBack1, 0, SERVO_BEG_2); 
  pwm.setPWM(leftFront1, 0, SERVO_BEG_1); 
  pwm.setPWM(rightBack1, 0, SERVO_BEG_3); 
}

//下蹲
void sit()
{
  //四条腿调到正常位置
  pwm.setPWM(rightFront1, 0, SERVO_BEG_0); 
  pwm.setPWM(leftBack1, 0, SERVO_BEG_2); 
  pwm.setPWM(leftFront1, 0, SERVO_BEG_1); 
  pwm.setPWM(rightBack1, 0, SERVO_BEG_3); 

  //蹲下
  pwm.setPWM(rightFront2, 0, SERVO_BEG_4 - SIT_DEPTH); 
  pwm.setPWM(leftBack2, 0, SERVO_BEG_6 - SIT_DEPTH); 
  pwm.setPWM(leftFront2, 0, SERVO_BEG_5 + SIT_DEPTH); 
  pwm.setPWM(rightBack2, 0, SERVO_BEG_7 + SIT_DEPTH); 
}

//运动某只腿
int MoveLeg(LegType leg, int vertArg, int levlArg, int direct)
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
    //levlArg = -levlArg;   更改说明：后方的腿在移动时，向前与抬腿的pulse变化是相反的，默认向前是正，通过宏修改。下同。
    vertArg = -vertArg;
    DEAL_LEFT_BACK_LEVL(levlArg); 
    DEAL_LEFT_BACK_VERT(vertArg); 
  break; 
  case LegType::RIGHT_BACK: 
    levlLeg = rightBack1; vertLeg = rightBack2; 
    servoBegLevl = SERVO_BEG_3; servoBegVert = SERVO_BEG_7; 
    //levlArg = -levlArg; 
    vertArg = -vertArg;
    DEAL_RIGHT_BACK_LEVL(levlArg); 
    DEAL_RIGHT_BACK_VERT(vertArg); 
  break; 
  }
  //抬腿
  pwm.setPWM(vertLeg, 0, servoBegVert + vertArg); 
  //前行
  pwm.setPWM(levlLeg, 0, servoBegLevl + levlArg * direct); 
  delay(ONE_LEG_MOVE_TIME << 1);    //等待移动
  //落腿
  pwm.setPWM(vertLeg, 0, servoBegVert); 
  return servoBegLevl + levlArg * direct; 
}

int MoveLeg_NoItv(LegType leg, int vertArg, int levlArg, int direct, bool tai)
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
    //levlArg = -levlArg;   更改说明：后方的腿在移动时，向前与抬腿的pulse变化是相反的，默认向前是正，通过宏修改。下同。
    vertArg = -vertArg;
    DEAL_LEFT_BACK_LEVL(levlArg); 
    DEAL_LEFT_BACK_VERT(vertArg); 
  break; 
  case LegType::RIGHT_BACK: 
    levlLeg = rightBack1; vertLeg = rightBack2; 
    servoBegLevl = SERVO_BEG_3; servoBegVert = SERVO_BEG_7; 
    //levlArg = -levlArg; 
    vertArg = -vertArg;
    DEAL_RIGHT_BACK_LEVL(levlArg); 
    DEAL_RIGHT_BACK_VERT(vertArg); 
  break; 
  }
  //抬腿
  pwm.setPWM(vertLeg, 0, servoBegVert + vertArg); 
  //前行
  pwm.setPWM(levlLeg, 0, servoBegLevl + levlArg * direct); 
  //delay(ONE_LEG_MOVE_TIME << 1);    //等待移动
  //落腿
  if (!tai) pwm.setPWM(vertLeg, 0, servoBegVert); 
  return servoBegLevl + levlArg * direct; 
}

//向前走
void moveForward()
{
#if __DEBUG__
  Serial.println("Forward called"); 
#endif
  
  //左前腿归位
  MoveLeg(LegType::LEFT_FRONT, STEP_HEIGHT, 0, FORWARD); 
  delay(STEP_INTERVAL / 8); 
  //右后腿向前迈
  MoveLeg(LegType::RIGHT_BACK, STEP_HEIGHT, MOVE_DISTANCE, FORWARD); 
  delay(STEP_INTERVAL / 4); 
  MoveLeg_NoItv(LegType::RIGHT_FRONT, STEP_HEIGHT, 0, BACKWARD, true); 
  MoveLeg_NoItv(LegType::LEFT_BACK, STEP_HEIGHT, 0, BACKWARD, true); 
  delay(STEP_INTERVAL / 8); 
  //左前腿向后抓地爬行
  MoveLeg_NoItv(LegType::LEFT_FRONT, 0, MOVE_DISTANCE, BACKWARD, false); 
  //delay(STEP_INTERVAL / 8); 
  //右后腿向后抓地爬行
  MoveLeg_NoItv(LegType::RIGHT_BACK, 0, 0, BACKWARD, false); 
  delay(200); 
  
  //右前腿归位
  MoveLeg(LegType::RIGHT_FRONT, STEP_HEIGHT, 0, FORWARD); 
  delay(STEP_INTERVAL / 8); 
  //左后腿向前迈
  MoveLeg(LegType::LEFT_BACK, STEP_HEIGHT, MOVE_DISTANCE, FORWARD); 
  delay(STEP_INTERVAL / 4); 
  MoveLeg_NoItv(LegType::LEFT_FRONT, STEP_HEIGHT, 0, BACKWARD, true); 
  MoveLeg_NoItv(LegType::RIGHT_BACK, STEP_HEIGHT, 0, BACKWARD, true); 
  delay(STEP_INTERVAL / 8); 
  //右前腿向后抓地爬行
  MoveLeg_NoItv(LegType::RIGHT_FRONT, 0, MOVE_DISTANCE, BACKWARD, false); 
  //delay(STEP_INTERVAL / 8); 
  //左后腿向后抓地爬行
  MoveLeg_NoItv(LegType::LEFT_BACK, 0, 0, BACKWARD, false); 
  delay(STEP_INTERVAL / 8); 
  delay(200); 
  MoveLeg(LegType::LEFT_FRONT, STEP_HEIGHT, 0, BACKWARD); 
  MoveLeg(LegType::RIGHT_BACK, 0, 0, BACKWARD); 
  MoveLeg(LegType::RIGHT_FRONT, STEP_HEIGHT, 0, BACKWARD); 
  delay(200); 
  stand(); 
  delay(200); 
  stand(); 

#if __DEBUG__
  Serial.println("Forward ended"); 
#endif
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
  
  //对正
  stand(); 
  
  int lastPosRightBack = MoveLeg(LegType::RIGHT_BACK, STEP_HEIGHT, MOVE_DISTANCE, FORWARD);   //右后爪
  delay(STEP_INTERVAL / 2); 
  int lastPosLeftBack = MoveLeg(LegType::LEFT_BACK, STEP_HEIGHT, MOVE_DISTANCE, BACKWARD);   //左后爪
  delay(STEP_INTERVAL / 2); 
  int lastPosRightFront = MoveLeg(LegType::RIGHT_FRONT, STEP_HEIGHT, MOVE_DISTANCE, FORWARD);   //右前爪
  delay(STEP_INTERVAL / 2); 
  int lastPosLeftFront = MoveLeg(LegType::LEFT_FRONT, STEP_HEIGHT, MOVE_DISTANCE, BACKWARD);   //左前爪
  delay(STEP_INTERVAL / 2); 

  

  int leftFrontStep = (lastPosLeftFront > SERVO_BEG_1) ? -1 : 1; 
  int rightBackStep = (lastPosRightBack > SERVO_BEG_3) ? -1 : 1; 
  int leftBackStep = (lastPosLeftBack > SERVO_BEG_2) ? -1 : 1; 
  int rightFrontStep = (lastPosRightFront > SERVO_BEG_0) ? -1 : 1; 

  bool finish = false; 

  //逐渐归正
  while (!finish)
  {
    finish = true; 
    if (lastPosLeftFront != SERVO_BEG_1)
    {
      lastPosLeftFront += leftFrontStep; 
      if (lastPosLeftFront < 0) lastPosLeftFront = 0; 
      else if (lastPosLeftFront > MAX_VAL) lastPosLeftFront = MAX_VAL; 
      else 
      {
        finish = false; 
        pwm.setPWM(leftFront1, 0, lastPosLeftFront); 
      }
    }
    if (lastPosRightBack != SERVO_BEG_3)
    {
      lastPosRightBack += rightBackStep; 
      if (lastPosRightBack < 0) lastPosRightBack = 0; 
      else if (lastPosRightBack > MAX_VAL) lastPosRightBack = MAX_VAL; 
      else 
      {
        finish = false; 
        pwm.setPWM(rightBack1, 0, lastPosRightBack); 
      }
    }
    if (lastPosLeftBack != SERVO_BEG_2)
    {
      lastPosLeftBack += leftBackStep; 
      if (lastPosLeftBack < 0) lastPosLeftBack = 0; 
      else if (lastPosLeftBack > MAX_VAL) lastPosLeftBack = MAX_VAL; 
      else 
      {
        finish = false; 
        pwm.setPWM(leftBack1, 0, lastPosLeftBack); 
      }
    }
    if (lastPosRightFront != SERVO_BEG_0)
    {
      lastPosRightFront += rightFrontStep; 
      if (lastPosRightFront < 0) lastPosRightFront = 0; 
      else if (lastPosRightFront > MAX_VAL) lastPosRightFront = MAX_VAL; 
      else 
      {
        finish = false; 
        pwm.setPWM(rightFront1, 0, lastPosRightFront); 
      }
    }
  }
 
  delay(STEP_INTERVAL / 2); 
  stand(); 
}

//向右转
void TurnRight()
{
  //对正
  stand(); 
  
  int lastPosRightBack = MoveLeg(LegType::RIGHT_BACK, STEP_HEIGHT, MOVE_DISTANCE, BACKWARD);   //右后爪
  delay(STEP_INTERVAL / 2); 
  int lastPosLeftBack = MoveLeg(LegType::LEFT_BACK, STEP_HEIGHT, MOVE_DISTANCE, FORWARD);   //左后爪
  delay(STEP_INTERVAL / 2); 
  int lastPosRightFront = MoveLeg(LegType::RIGHT_FRONT, STEP_HEIGHT, MOVE_DISTANCE, BACKWARD);   //右前爪
  delay(STEP_INTERVAL / 2); 
  int lastPosLeftFront = MoveLeg(LegType::LEFT_FRONT, STEP_HEIGHT, MOVE_DISTANCE, FORWARD);   //左前爪
  delay(STEP_INTERVAL / 2); 

  

  int leftFrontStep = (lastPosLeftFront > SERVO_BEG_1) ? -1 : 1; 
  int rightBackStep = (lastPosRightBack > SERVO_BEG_3) ? -1 : 1; 
  int leftBackStep = (lastPosLeftBack > SERVO_BEG_2) ? -1 : 1; 
  int rightFrontStep = (lastPosRightFront > SERVO_BEG_0) ? -1 : 1; 

  bool finish = false; 

  //逐渐归正
  while (!finish)
  {
    finish = true; 
    if (lastPosLeftFront != SERVO_BEG_1)
    {
      lastPosLeftFront += leftFrontStep; 
      if (lastPosLeftFront < 0) lastPosLeftFront = 0; 
      else if (lastPosLeftFront > MAX_VAL) lastPosLeftFront = MAX_VAL; 
      else 
      {
        finish = false; 
        pwm.setPWM(leftFront1, 0, lastPosLeftFront); 
      }
    }
    if (lastPosRightBack != SERVO_BEG_3)
    {
      lastPosRightBack += rightBackStep; 
      if (lastPosRightBack < 0) lastPosRightBack = 0; 
      else if (lastPosRightBack > MAX_VAL) lastPosRightBack = MAX_VAL; 
      else 
      {
        finish = false; 
        pwm.setPWM(rightBack1, 0, lastPosRightBack); 
      }
    }
    if (lastPosLeftBack != SERVO_BEG_2)
    {
      lastPosLeftBack += leftBackStep; 
      if (lastPosLeftBack < 0) lastPosLeftBack = 0; 
      else if (lastPosLeftBack > MAX_VAL) lastPosLeftBack = MAX_VAL; 
      else 
      {
        finish = false; 
        pwm.setPWM(leftBack1, 0, lastPosLeftBack); 
      }
    }
    if (lastPosRightFront != SERVO_BEG_0)
    {
      lastPosRightFront += rightFrontStep; 
      if (lastPosRightFront < 0) lastPosRightFront = 0; 
      else if (lastPosRightFront > MAX_VAL) lastPosRightFront = MAX_VAL; 
      else 
      {
        finish = false; 
        pwm.setPWM(rightFront1, 0, lastPosRightFront); 
      }
    }
  }
 
  delay(STEP_INTERVAL / 2); 
  stand(); 
}
