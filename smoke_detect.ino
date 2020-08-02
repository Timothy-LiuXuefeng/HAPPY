#include"SoftwareSerial.h"
#include"U8glib.h"
#include"DHT.h"
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE|U8G_I2C_OPT_DEV_0);
SoftwareSerial myserial(4,5);
#define DHTPIN 12
#define DHTTYPE DHT11
DHT mydht(DHTPIN,DHTTYPE);      //dht模块
int thres = 50;     //设置阈值
int smoke_val = 0;
char hum[7] = "";
char tem[7] = "";
float hum_flt =0 ,tem_flt = 0;
int buzzer = 6; //蜂鸣器
void setup()
{
    myserial.begin(9600);
    mydht.begin();
    pinMode(buzzer,OUTPUT);
    u8g.setFont(u8g_font_unifont);
}
void loop()
{
    u8g.firstPage();
    do{
        u8g.drawStr(0,12,"Hum");
        u8g.drawStr(50,12,"Tem");
        u8g.drawStr(0,40,hum);  //湿度
        u8g.drawStr(50,40,tem);     //温度
    }
    while(u8g.nextPage());
    hum_flt = mydht.readHumidity();
    tem_flt = mydht.readTemperature();
    if(myserial.available())    //执行读取相关的设置。
    {
        char temp = myserial.read();
        if(temp == 't')
        {
            myserial.print("temperature is ");
            myserial.print(tem_flt);
        }
        else if(temp == 'h')
        {
            myserial.print("humidity is ");
            myserial.print(hum_flt);
        }
        else if(temp == 's')
        {
            while(!myserial.available());
            int value = myserial.read();
            if(value)
            {
                myserial.println("threshold has been changed as ");
                myserial.println(value);
            }
        }
    }
    sprintf(hum,"%f",hum_flt);
    sprintf(tem,"%f",tem_flt);
    smoke_val = analogRead(0);
    if(smoke_val > thres)   //烟雾大于阈值,报警
    {
        digitalWrite(buzzer,HIGH);
        delay(50);
        digitalWrite(buzzer,LOW);
        delay(50);
    }
}