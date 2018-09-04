#include "COMMON.h"
#include "GSM_MQTT.h"
#include "Arduino.h"
#include <avr/pgmspace.h>

String MQTT_HOST = "106.14.26.130";           //MQTT服务器IP
String MQTT_PORT = "1995";                    //MQTT服务器端口号

char tmp[128]={0};         //消息缓存数组
char csq[4]={"0"};          //信号强度



uint8_t GSM_Response = 0;
unsigned long previousMillis = 0;
//char inputString[UART_BUFFER_LENGTH];         // a string to hold incoming data
boolean stringComplete = false;                 // whether the string is complete

GSM_MQTT MQTT(40);  /* 20 is the keepalive duration in seconds */


void GSM_MQTT::AutoConnect(void)                  //mqtt连接请求
{
  memset( tmp,0,sizeof(tmp) );
  sprintf(tmp,"{\"devId\":\"%s\",\"state\":0}",DEV_ID);
  connect(DEV_ID, 1, 1, "zx_admin", "ZxNumber1", 1, 1, 1, 1, "zx/door/lastword",tmp);  //mqtt连接指定用户名、密码、遗言内容和遗言发送的频道
}

void GSM_MQTT::OnConnect(void)                     //连接请求成功后
{
    memset( tmp,0,sizeof(tmp) );
    sprintf(tmp,"zx/door/opt/%s",DEV_ID);
    subscribe(0, _generateMessageID(), tmp, 1);
    
    memset( tmp,0,sizeof(tmp) );
    sprintf(tmp,"{\"devId\":\"%s\",\"state\":1,\"signal\":%s}",DEV_ID,csq);
    publish(0, 1, 0, _generateMessageID(), "zx/door/firstword",tmp);

    if(car_flow_counts > 0) //如果离线时有车经过  则上报历史数据
    {
        sprintf(tmp,"{\"devId\":\"%s\",\"number\":%d}",DEV_ID,car_flow_counts);
#ifdef DE_BUG
        mySerial.println(tmp);
#endif
        MQTT.publish(0, 1, 0, MQTT._generateMessageID(), "zx/park/car",tmp);
        car_flow_counts = 0;
        EEPROM.put(0,car_flow_counts);
    }
    
}

void GSM_MQTT::OnMessage(char *Topic, int TopicLength, char *Message, int MessageLength)  //当收到消息时被调用
{
   Parse_Json_message_Task(Message); //解析消息并进行对应的任务
}

GSM_MQTT::GSM_MQTT(unsigned long KeepAlive)
{
  _KeepAliveTimeOut = KeepAlive;
}

void GSM_MQTT::begin(void)
{
  delay(1000);
}

void GSM_MQTT::_ping(void)
{
  if (pingFlag == true)
  {
    unsigned long currentMillis = millis();
    if (_PingPrevMillis == 0 ||(currentMillis - _PingPrevMillis ) >= _KeepAliveTimeOut * 1000)//如果时间大于心跳周期则发送ping包
    {
      _PingPrevMillis = currentMillis;//by fangye 20170911 用于解决ping包无回复不会认为掉线的BUG
      if(_ping_return_flag > 1)//如果发送2次心跳后还没有收到心跳回复则认为设备已离线
      {
        MQTT_Flag = false;
        pingFlag =  false;
        TCP_Flag = false;
        digitalWrite(led_red,HIGH);      //红色led灯点亮
        digitalWrite(led_blue,LOW);   //上线指示灯熄灭  
        _ping_return_flag = 0;  //心跳超时计数清零
        _PingPrevMillis = 0;
#ifdef DE_BUG
          mySerial.println("TCP Close...");
#endif
        return;
      }
#ifdef DE_BUG
      mySerial.println("Keep-Alive...");
#endif     
      GSM_UART.print(char(PINGREQ * 16));
      _sendLength(0);
  	  //by fangye 20170911 用于解决ping包无回复不会认为掉线的BUG
  	  _ping_return_flag++;	//收到心跳回复包时该标志会清零
     }
  }
}
void GSM_MQTT::_sendUTFString(char *string)
{
  int localLength = strlen(string);
  GSM_UART.print(char(localLength / 256));
  GSM_UART.print(char(localLength % 256));
  GSM_UART.print(string);
}
void GSM_MQTT::_sendLength(int len)
{
  bool  length_flag = false;
  while (length_flag == false)
  {
    if ((len / 128) > 0)
    {
      GSM_UART.print(char(len % 128 + 128));
      len /= 128;
    }
    else
    {
      length_flag = true;
      GSM_UART.print(char(len));
    }
  }
}
void GSM_MQTT::connect(char *ClientIdentifier, char UserNameFlag, char PasswordFlag, char *UserName, char *Password, char CleanSession, char WillFlag, char WillQoS, char WillRetain, char *WillTopic, char *WillMessage)
{
  ConnectionAcknowledgement = NO_ACKNOWLEDGEMENT ;
  GSM_UART.print(char(CONNECT * 16 ));
  char ProtocolName[7] = "MQIsdp";
  int localLength = (2 + strlen(ProtocolName)) + 1 + 3 + (2 + strlen(ClientIdentifier));
  if (WillFlag != 0)
  {
    localLength = localLength + 2 + strlen(WillTopic) + 2 + strlen(WillMessage);
  }
  if (UserNameFlag != 0)
  {
    localLength = localLength + 2 + strlen(UserName);

    if (PasswordFlag != 0)
    {
      localLength = localLength + 2 + strlen(Password);
    }
  }
  _sendLength(localLength);
  _sendUTFString(ProtocolName);
  GSM_UART.print(char(_ProtocolVersion));
  GSM_UART.print(char(UserNameFlag * User_Name_Flag_Mask + PasswordFlag * Password_Flag_Mask + WillRetain * Will_Retain_Mask + WillQoS * Will_QoS_Scale + WillFlag * Will_Flag_Mask + CleanSession * Clean_Session_Mask));
  GSM_UART.print(char(_KeepAliveTimeOut / 256));
  GSM_UART.print(char(_KeepAliveTimeOut % 256));
  _sendUTFString(ClientIdentifier);
  if (WillFlag != 0)
  {
    _sendUTFString(WillTopic);
    _sendUTFString(WillMessage);
  }
  if (UserNameFlag != 0)
  {
    _sendUTFString(UserName);
    if (PasswordFlag != 0)
    {
      _sendUTFString(Password);
    }
  }
}
void GSM_MQTT::publish(char DUP, char Qos, char RETAIN, unsigned int MessageID, char *Topic, char *Message)
{
  GSM_UART.print(char(PUBLISH * 16 + DUP * DUP_Mask + Qos * QoS_Scale + RETAIN));
  int localLength = (2 + strlen(Topic));
  if (Qos > 0)
  {
    localLength += 2;
  }
  localLength += strlen(Message);
  _sendLength(localLength);
  _sendUTFString(Topic);
  if (Qos > 0)
  {
    GSM_UART.print(char(MessageID / 256));
    GSM_UART.print(char(MessageID % 256));
  }
  GSM_UART.print(Message);
}
void GSM_MQTT::publishACK(unsigned int MessageID)
{
  GSM_UART.print(char(PUBACK * 16));
  _sendLength(2);
  GSM_UART.print(char(MessageID / 256));
  GSM_UART.print(char(MessageID % 256));
}
void GSM_MQTT::publishREC(unsigned int MessageID)
{
  GSM_UART.print(char(PUBREC * 16));
  _sendLength(2);
  GSM_UART.print(char(MessageID / 256));
  GSM_UART.print(char(MessageID % 256));
}
void GSM_MQTT::publishREL(char DUP, unsigned int MessageID)
{
  GSM_UART.print(char(PUBREL * 16 + DUP * DUP_Mask + 1 * QoS_Scale));
  _sendLength(2);
  GSM_UART.print(char(MessageID / 256));
  GSM_UART.print(char(MessageID % 256));
}

void GSM_MQTT::publishCOMP(unsigned int MessageID)
{
  GSM_UART.print(char(PUBCOMP * 16));
  _sendLength(2);
  GSM_UART.print(char(MessageID / 256));
  GSM_UART.print(char(MessageID % 256));
}
void GSM_MQTT::subscribe(char DUP, unsigned int MessageID, char *SubTopic, char SubQoS)
{
  GSM_UART.print(char(SUBSCRIBE * 16 + DUP * DUP_Mask + 1 * QoS_Scale));
  int localLength = 2 + (2 + strlen(SubTopic)) + 1;
  _sendLength(localLength);
  GSM_UART.print(char(MessageID / 256));
  GSM_UART.print(char(MessageID % 256));
  _sendUTFString(SubTopic);
  GSM_UART.print(SubQoS);

}
void GSM_MQTT::unsubscribe(char DUP, unsigned int MessageID, char *SubTopic)
{
  GSM_UART.print(char(UNSUBSCRIBE * 16 + DUP * DUP_Mask + 1 * QoS_Scale));
  int localLength = (2 + strlen(SubTopic)) + 2;
  _sendLength(localLength);

  GSM_UART.print(char(MessageID / 256));
  GSM_UART.print(char(MessageID % 256));

  _sendUTFString(SubTopic);
}
void GSM_MQTT::disconnect(void)
{
  GSM_UART.print(char(DISCONNECT * 16));
  _sendLength(0);
  pingFlag = false;
}
//Messages
const char CONNECTMessage[] PROGMEM  = {"Client request to connect to Server\r\n"};
const char CONNACKMessage[] PROGMEM  = {"Connect Acknowledgment\r\n"};
const char PUBLISHMessage[] PROGMEM  = {"Publish message\r\n"};
const char PUBACKMessage[] PROGMEM  = {"Publish Acknowledgment\r\n"};
const char PUBRECMessage[] PROGMEM  = {"Publish Received (assured delivery part 1)\r\n"};
const char PUBRELMessage[] PROGMEM  = {"Publish Release (assured delivery part 2)\r\n"};
const char PUBCOMPMessage[] PROGMEM  = {"Publish Complete (assured delivery part 3)\r\n"};
const char SUBSCRIBEMessage[] PROGMEM  = {"Client Subscribe request\r\n"};
const char SUBACKMessage[] PROGMEM  = {"Subscribe Acknowledgment\r\n"};
const char UNSUBSCRIBEMessage[] PROGMEM  = {"Client Unsubscribe request\r\n"};
const char UNSUBACKMessage[] PROGMEM  = {"Unsubscribe Acknowledgment\r\n"};
const char PINGREQMessage[] PROGMEM  = {"PING Request\r\n"};
const char PINGRESPMessage[] PROGMEM  = {"PING Response\r\n"};
const char DISCONNECTMessage[] PROGMEM  = {"Client is Disconnecting\r\n"};

void GSM_MQTT::printMessageType(uint8_t Message)
{
  switch (Message)
  {
    case CONNECT:
      {
        int k, len = strlen_P(CONNECTMessage);
        char myChar;
        for (k = 0; k < len; k++)
        {
          myChar =  pgm_read_byte_near(CONNECTMessage + k);
#ifdef DE_BUG
          mySerial.print(myChar);
#endif
        }
        break;
      }
    case CONNACK:
      {
        int k, len = strlen_P(CONNACKMessage);
        char myChar;
        for (k = 0; k < len; k++)
        {
          myChar =  pgm_read_byte_near(CONNACKMessage + k);
#ifdef DE_BUG
          mySerial.print(myChar);
#endif
        }
        break;
      }
    case PUBLISH:
      {
        int k, len = strlen_P(PUBLISHMessage);
        char myChar;
        for (k = 0; k < len; k++)
        {
          myChar =  pgm_read_byte_near(PUBLISHMessage + k);
#ifdef DE_BUG
          mySerial.print(myChar);
#endif
        }
        break;
      }
    case PUBACK:
      {
        int k, len = strlen_P(PUBACKMessage);
        char myChar;
        for (k = 0; k < len; k++)
        {
          myChar =  pgm_read_byte_near(PUBACKMessage + k);
#ifdef DE_BUG
          mySerial.print(myChar);
#endif
        }
        break;
      }
    case  PUBREC:
      {
        int k, len = strlen_P(PUBRECMessage);
        char myChar;
        for (k = 0; k < len; k++)
        {
          myChar =  pgm_read_byte_near(PUBRECMessage + k);
#ifdef DE_BUG
          mySerial.print(myChar);
#endif
        }
        break;
      }
    case PUBREL:
      {
        int k, len = strlen_P(PUBRELMessage);
        char myChar;
        for (k = 0; k < len; k++)
        {
          myChar =  pgm_read_byte_near(PUBRELMessage + k);
#ifdef DE_BUG
          mySerial.print(myChar);
#endif
        }
        break;
      }
    case PUBCOMP:
      {
        int k, len = strlen_P(PUBCOMPMessage );
        char myChar;
        for (k = 0; k < len; k++)
        {
          myChar =  pgm_read_byte_near(PUBCOMPMessage  + k);
#ifdef DE_BUG
          mySerial.print(myChar);
#endif
        }
        break;
      }
    case SUBSCRIBE:
      {
        int k, len = strlen_P(SUBSCRIBEMessage );
        char myChar;
        for (k = 0; k < len; k++)
        {
          myChar =  pgm_read_byte_near(SUBSCRIBEMessage  + k);
#ifdef DE_BUG
          mySerial.print(myChar);
#endif
        }
        break;
      }
    case SUBACK:
      {
        int k, len = strlen_P(SUBACKMessage );
        char myChar;
        for (k = 0; k < len; k++)
        {
          myChar =  pgm_read_byte_near(SUBACKMessage  + k);
#ifdef DE_BUG
          mySerial.print(myChar);
#endif
        }
        break;
      }
    case UNSUBSCRIBE:
      {
        int k, len = strlen_P(UNSUBSCRIBEMessage );
        char myChar;
        for (k = 0; k < len; k++)
        {
          myChar =  pgm_read_byte_near(UNSUBSCRIBEMessage  + k);
#ifdef DE_BUG
          mySerial.print(myChar);
#endif
        }
        break;
      }
    case UNSUBACK:
      {
        int k, len = strlen_P(UNSUBACKMessage );
        char myChar;
        for (k = 0; k < len; k++)
        {
          myChar =  pgm_read_byte_near(UNSUBACKMessage  + k);
#ifdef DE_BUG
          mySerial.print(myChar);
#endif
        }
        break;
      }
    case PINGREQ:
      {
        int k, len = strlen_P(PINGREQMessage);
        char myChar;
        for (k = 0; k < len; k++)
        {
          myChar =  pgm_read_byte_near(PINGREQMessage + k);
#ifdef DE_BUG
          mySerial.print(myChar);
#endif
        }
        break;
      }
    case PINGRESP:
      {
        int k, len = strlen_P(PINGRESPMessage);
        char myChar;
        for (k = 0; k < len; k++)
        {
          myChar =  pgm_read_byte_near(PINGRESPMessage + k);
#ifdef DE_BUG
          mySerial.print(myChar);
#endif
        }
        break;
      }
    case DISCONNECT:
      {
        int k, len = strlen_P(DISCONNECTMessage);
        char myChar;
        for (k = 0; k < len; k++)
        {
          myChar =  pgm_read_byte_near(DISCONNECTMessage + k);
#ifdef DE_BUG
          mySerial.print(myChar);
#endif
        }
        break;
      }
  }
}

//Connect Ack
const char ConnectAck0[] PROGMEM  = {"Connection Accepted\r\n"};
const char ConnectAck1[] PROGMEM  = {"Connection Refused: unacceptable protocol version\r\n"};
const char ConnectAck2[] PROGMEM  = {"Connection Refused: identifier rejected\r\n"};
const char ConnectAck3[] PROGMEM  = {"Connection Refused: server unavailable\r\n"};
const char ConnectAck4[] PROGMEM  = {"Connection Refused: bad user name or password\r\n"};
const char ConnectAck5[] PROGMEM  = {"Connection Refused: not authorized\r\n"};

void GSM_MQTT::printConnectAck(uint8_t Ack)
{
  switch (Ack)
  {
    case 0:
      {
        int k, len = strlen_P(ConnectAck0);
        char myChar;
        for (k = 0; k < len; k++)
        {
          myChar =  pgm_read_byte_near(ConnectAck0 + k);
#ifdef DE_BUG
          mySerial.print(myChar);
#endif
        }
        break;
      }
    case 1:
      {
        int k, len = strlen_P(ConnectAck1);
        char myChar;
        for (k = 0; k < len; k++)
        {
          myChar =  pgm_read_byte_near(ConnectAck1 + k);
#ifdef DE_BUG
          mySerial.print(myChar);
#endif
        }
        break;
      }
    case 2:
      {
        int k, len = strlen_P(ConnectAck2);
        char myChar;
        for (k = 0; k < len; k++)
        {
          myChar =  pgm_read_byte_near(ConnectAck2 + k);
#ifdef DE_BUG
          mySerial.print(myChar);
#endif
        }
        break;
      }
    case 3:
      {
        int k, len = strlen_P(ConnectAck3);
        char myChar;
        for (k = 0; k < len; k++)
        {
          myChar =  pgm_read_byte_near(ConnectAck3 + k);
#ifdef DE_BUG
          mySerial.print(myChar);
#endif
        }
        break;
      }
    case 4:
      {
        int k, len = strlen_P(ConnectAck4);
        char myChar;
        for (k = 0; k < len; k++)
        {
          myChar =  pgm_read_byte_near(ConnectAck4 + k);
#ifdef DE_BUG
          mySerial.print(myChar);
#endif
        }
        break;
      }
    case 5:
      {
        int k, len = strlen_P(ConnectAck5);
        char myChar;
        for (k = 0; k < len; k++)
        {
          myChar =  pgm_read_byte_near(ConnectAck5 + k);
#ifdef DE_BUG
          mySerial.print(myChar);
#endif
        }
        break;
      }
  }
}

unsigned int GSM_MQTT::_generateMessageID(void)
{
  if (_LastMessaseID < 65535)
  {
    return ++_LastMessaseID;
  }
  else
  {
    _LastMessaseID = 0;
    return _LastMessaseID;
  }
}

void GSM_MQTT::processing(void)
{
  if (TCP_Flag == false)
  {
     CurMillis_connect = millis();
     if(CurMillis_connect - PrevMillis_connect > 10000)//每10秒发送一次mqtt连接申请
     {
        PrevMillis_connect = CurMillis_connect;
#ifdef DE_BUG
        mySerial.print("TCP_Flag = ");
        mySerial.println(TCP_Flag);
        mySerial.println("Connecting mqtt...");
#endif  
        AutoConnect(); //请求建立MQTT连接
        Hard_Reset_count++;   //硬件复位计数
        if(Hard_Reset_count >= 50)  //如果超过预期次数（连接不上则重启程序 复位TCP232模块）
        {
            asm volatile ("jmp 0");
        }
    }
  }
  else
  {
      if(Hard_Reset_count)
        Hard_Reset_count = 0; //硬件复位计数清零
      _ping();
  }
  wdt_reset(); //喂狗操作
}

bool GSM_MQTT::available(void)
{
  return MQTT_Flag;
}

void Serial1Event()
{
  while (GSM_UART.available())
  {
    char inChar = (char)GSM_UART.read();
    uint8_t ReceivedMessageType = (inChar / 16) & 0x0F;
    uint8_t DUP = (inChar & DUP_Mask) / DUP_Mask;
    uint8_t QoS = (inChar & QoS_Mask) / QoS_Scale;
    uint8_t RETAIN = (inChar & RETAIN_Mask);
      
    if ((ReceivedMessageType >= CONNECT) && (ReceivedMessageType <= DISCONNECT))
    {
      bool NextLengthByte = true;
      MQTT.length = 0;
      MQTT.lengthLocal = 0;
      uint32_t multiplier=1;
      if(GSM_UART.available() < 5)
        delay(2);
      char Cchar = inChar;
      while( NextLengthByte == true ) //&& (MQTT.TCP_Flag == true))
      {
        if (GSM_UART.available())
        {
          inChar = (char)GSM_UART.read();
          if(GSM_UART.available() < 5)
              delay(2);
#ifdef DE_BUG     
          mySerial.println(inChar, DEC);
#endif
          
          if ((inChar & 128) == 128)
          {
            MQTT.length += (inChar & 127) *  multiplier;
            multiplier *= 128;
#ifdef DE_BUG
           mySerial.println("More");
#endif
          }
          else
          {
            NextLengthByte = false;
            MQTT.length += (inChar & 127) *  multiplier;
            multiplier *= 128;
          }
        }
        
        MQTT.lengthLocal = MQTT.length;

#ifdef DE_BUG
        MQTT.printMessageType(ReceivedMessageType);
#endif
        MQTT.index = 0L;
        uint32_t a = 0;
        while ((MQTT.length-- > 0) && (GSM_UART.available()))
        {
          MQTT.inputString[uint32_t(MQTT.index++)] = (char)GSM_UART.read();
          delay(2);
        }
          
        if (ReceivedMessageType == CONNACK)
        {
#ifdef DE_BUG
            MQTT.ConnectionAcknowledgement = MQTT.inputString[0] * 256 + MQTT.inputString[1];
            if (MQTT.ConnectionAcknowledgement == 0)
            {
              mySerial.println("-- connect ACK!");
            }
            else
            {
              mySerial.println("-- Reconnection!");
            }
#endif 
            MQTT.TCP_Flag = true;
            MQTT.OnConnect();  //发布上线消息
          }
          else if (ReceivedMessageType == PUBLISH)
          {
            uint32_t TopicLength = (MQTT.inputString[0]) * 256 + (MQTT.inputString[1]);
#ifdef DE_BUG
            mySerial.print("Topic : '");
#endif
            MQTT.PublishIndex = 0;
            for (uint32_t iter = 2; iter < TopicLength + 2; iter++)
            {
#ifdef DE_BUG
              mySerial.print(MQTT.inputString[iter]);
#endif
              MQTT.Topic[MQTT.PublishIndex++] = MQTT.inputString[iter];
            }
            MQTT.Topic[MQTT.PublishIndex] = 0;
#ifdef DE_BUG
            mySerial.print("' Message :'");
#endif
            MQTT.TopicLength = MQTT.PublishIndex;

            MQTT.PublishIndex = 0;
            uint32_t MessageSTART = TopicLength + 2UL;
            int MessageID = 0;
            if (QoS != 0)
            {
              MessageSTART += 2;
              MessageID = MQTT.inputString[TopicLength + 2UL] * 256 + MQTT.inputString[TopicLength + 3UL];
            }
            for (uint32_t iter = (MessageSTART); iter < (MQTT.lengthLocal); iter++)
            {
#ifdef DE_BUG
              mySerial.print(MQTT.inputString[iter]);
#endif
              MQTT.Message[MQTT.PublishIndex++] = MQTT.inputString[iter];
            }
            MQTT.Message[MQTT.PublishIndex] = 0;
#ifdef DE_BUG  
            mySerial.println("'");
#endif
            MQTT.MessageLength = MQTT.PublishIndex;
            if (QoS == 1)
            {
              MQTT.publishACK(MessageID);
            }
            else if (QoS == 2)
            {
              MQTT.publishREC(MessageID);
            }
            MQTT.OnMessage(MQTT.Topic, MQTT.TopicLength, MQTT.Message, MQTT.MessageLength);
            MQTT.MessageFlag = true;
          }
          else if (ReceivedMessageType == PUBREC)
          {
#ifdef DE_BUG
            mySerial.print("Message ID :");
#endif           
            MQTT.publishREL(0, MQTT.inputString[0] * 256 + MQTT.inputString[1]) ;
#ifdef DE_BUG           
            mySerial.println(MQTT.inputString[0] * 256 + MQTT.inputString[1]) ;
#endif
          }
          else if (ReceivedMessageType == PUBREL)
          {
#ifdef DE_BUG  
            mySerial.print("Message ID :");
#endif
            MQTT.publishCOMP(MQTT.inputString[0] * 256 + MQTT.inputString[1]) ;
#ifdef DE_BUG
            mySerial.println(MQTT.inputString[0] * 256 + MQTT.inputString[1]) ;
#endif
          }
          else if ((ReceivedMessageType == PUBACK) || (ReceivedMessageType == PUBCOMP) || (ReceivedMessageType == SUBACK) || (ReceivedMessageType == UNSUBACK))
          {
#ifdef DE_BUG
            mySerial.print("Message ID :");
            mySerial.println(MQTT.inputString[0] * 256 + MQTT.inputString[1]);
#endif
            if(MQTT.pingFlag == false)
            {
              MQTT.pingFlag = true;
#ifdef DE_BUG  
              mySerial.println("---- PUBACK or SUBACK Set pingFlag = true");
#endif
            }
          }
          else if (ReceivedMessageType == PINGREQ)
          {
#ifdef DE_BUG  
              mySerial.println("Recieve PINGREQ...");
#endif
          }
    		  else if(ReceivedMessageType == PINGRESP)  //by fangye 20170911 用于解决ping包无回复不会认为掉线的BUG
    		  {
#ifdef DE_BUG  
              mySerial.println("---- Ping ACK..");
#endif
              if(MQTT.MQTT_Flag == false)
              {
                digitalWrite(led_blue,HIGH);    //蓝色led灯点亮
                digitalWrite(led_red,LOW);      //红色led灯熄灭
                MQTT.MQTT_Flag = true;
              }
              MQTT._ping_return_flag = 0;       //收到ping包的回复则将_ping_return_flag标志清零
    		  }
        }
      }//else if ((inChar = 13) || (inChar == 10))
      else
      {
#ifdef DE_BUG  
        mySerial.print("Received :Unknown Message Type :");
        mySerial.println(inChar);
#endif
      }
  }
  wdt_reset(); //喂狗操作
}

void signal_strength(unsigned long waitms)    //查询信号强度
{
   char p = 0;
   unsigned char i = 0;
   while(GSM_UART.read()>=0); //清空串口缓存
   unsigned long PrevMillis = millis();
   GSM_UART.print("AT+CSQ\r\n");
   memset(csq,0,sizeof(csq));
   unsigned long currentMillis = millis();
   while ( (i < 2) && ((currentMillis - PrevMillis) < waitms) )
   {
      while( GSM_UART.available()> 0 )  
      {
          p = GSM_UART.read();
          if( p == ',')
          {
            i = 2;
            break;
          }  
          else if(p >= '0' &&  p<= '9')
          {
              csq[i++] = p;
              break;
          }
      }
      currentMillis = millis();
   }
   if(i == 0)  //如果查询信号强度超时 则信号为0
   {
	   csq[0]='0';
	   csq[1]='\0';
   }
   while(GSM_UART.read()>=0); //清空串口缓存
   return;
}
