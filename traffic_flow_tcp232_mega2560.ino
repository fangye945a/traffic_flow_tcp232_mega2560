#include "COMMON.h"
#include "GSM_MQTT.h"

unsigned int cur_state = 0;   //地感当前状态
unsigned int car_flow_counts = 0; //车流量计数 
unsigned long PrevMillis_connect = 0;
unsigned long CurMillis_connect = 0;
void Param_init()
{     
    EEPROM.get(0,car_flow_counts);  //获取EEPROM中开门时长   默认为10  单位100ms
    if(car_flow_counts == 0xFFFF)  //若为初始值
    {
        car_flow_counts = 0;
        EEPROM.put(0,car_flow_counts);
    }
}

void Parse_Json_message_Task(char *json)
{
    StaticJsonBuffer<MESSAGE_BUFFER_LENGTH> jsonBuffer;
    JsonObject& root = jsonBuffer.parseObject(json);
    if (!root.success()) 
    {
#ifdef DE_BUG
        mySerial.println("parseObject() failed"); //解析对象失败
#endif
        return;
    }
    int optcode = root["optcode"];
#ifdef DE_BUG
        mySerial.print("optcode="); //对应操作命令
        mySerial.println(optcode);
#endif
    switch(optcode)
    {
        case 110:                       //查询设备状态,推送上线消息
        {
            memset( tmp,0,sizeof(tmp) );
            sprintf(tmp,"{\"devId\":\"%s\",\"state\":1,\"signal\":%s}",DEV_ID,csq);
            MQTT.publish(0, 1, 0, MQTT._generateMessageID(), "zx/door/firstword",tmp);
        }
        break;
        case 119:                       //强制重启设备
        {
            delay(50);
            asm volatile ("jmp 0");
        }
        break;
        default:
#ifdef DE_BUG
        mySerial.println("error optcode!!");
#endif
        break;
    }
}

void IO_init()
{
    unsigned char i = 0;
    pinMode(led, OUTPUT);          //设置IO口为输出模式  白色led
    pinMode(sw, INPUT);            //设置IO口为输入模式  车辆检测
    pinMode(led_red, OUTPUT);      //设置IO口为输出模式  红色led
    pinMode(led_blue, OUTPUT);     //设置IO口为输出模式  蓝色led
    //pinMode(locked ,OUTPUT);       //继电器控制引脚
    pinMode(gsm_reset ,OUTPUT);    //GSM模块复位引脚
   
    digitalWrite(led,LOW);           //白色led灯点熄灭
    digitalWrite(led_red,HIGH);      //红色led灯点亮
    digitalWrite(led_blue,LOW);      //蓝色led灯点熄灭
	
    digitalWrite(gsm_reset,LOW);  //GSM模块复位初始化
    delay(500);					  //置低500ms然后拉高
    digitalWrite(gsm_reset,HIGH); //GSM模块复位初始化
    digitalWrite(led,HIGH);           //白色led灯点
}

void URAT_init()
{
#ifdef DE_BUG
    mySerial.begin(115200);
#endif
    GSM_UART.begin(9600);   //GSM串口初始化
}

void Car_Event()
{
    unsigned int sw_state = digitalRead(sw);//检测地感状态
    delay(30);  //消抖
    if(sw_state != digitalRead(sw))
    {
      return;
    }
    else 
    {
        if(sw_state != cur_state)     //是否与当前记录状态不同  
        {
            if(cur_state == 1)      //下降沿       //地感有车低电平，无车高电平
            {
                if(MQTT.MQTT_Flag == true)  //如果在线
                {
                    sprintf(tmp,"{\"devId\":\"%s\",\"number\":1}",DEV_ID);
#ifdef DE_BUG
                    mySerial.println(tmp);
#endif
                    MQTT.publish(0, 1, 0, MQTT._generateMessageID(), "zx/park/car",tmp);
                }else   //否则记录到EEPROM中
                {
                    car_flow_counts++;
                    EEPROM.put(0,car_flow_counts);
                }
            }
#ifdef DE_BUG
            mySerial.print("Change state:");
            mySerial.print("From ");
            mySerial.print(cur_state);
            mySerial.print(" to ");
            mySerial.println(sw_state);
#endif
            cur_state = sw_state;
        }
    }
}

void setup()
{
    URAT_init();   //串口初始化
    IO_init();     //IO口初始化
    Param_init();   //参数初始化
    wdt_enable(WDTO_8S); //开启看门狗，并设置溢出时间为8秒
#ifdef DE_BUG
    mySerial.println("setup finish!!");
#endif
}

void loop()
{
    MQTT.processing(); // MQTT任务
    Serial1Event();    // GSM 任务
    Car_Event();       // 车流量检测任务
    wdt_reset(); //喂狗操作
}
