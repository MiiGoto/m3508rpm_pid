#include <MsTimer2.h>
#include <FlexCAN.h>
#include "PID.h"

typedef struct
{
  int16_t rotation;
  int16_t denryu;
} wheelEscDataSt;

FlexCAN CANTransmitter(1000000);
static CAN_message_t rxmsg;//can受信用buf
CAN_message_t msg;//can送信用buf
wheelEscDataSt wEscData[4];//can受信用

Pid pid0;

void setup(void)
{
  CANTransmitter.begin();
  Serial.begin(115200);
  delay(1000);

  msg.len = 8;
  msg.id = 0x200;
  for ( int idx = 0; idx < msg.len; ++idx ) {
    msg.buf[idx] = 0;
  }

  pid0.init(1.0, 0.0, 0.0); //p,i,dの順に指定できる

  MsTimer2::set(2, timerInt);
  MsTimer2::start();

}

void loop(void)
{
  int u[4] = {0};
  u[0] = 500; //ここの数字はrpm指定、-5000~5000くらい
  
  Serial.print(u[0]);//目標速度
  Serial.print(",");
  
  u[0] = pid0.pid_out(u[0]);

  for (int i = 0; i < 4; i++) {
    msg.buf[i * 2] = u[i] >> 8;
    msg.buf[i * 2 + 1] = u[i] & 0xFF;
  }
  Serial.print(pid0.debug());//現在速度
  Serial.println("");
  delay(10);
}

void timerInt() {
  while ( CANTransmitter.read(rxmsg) ) {
    if (rxmsg.id == 0x201) {
      pid0.now_value(rxmsg.buf[2] * 256 + rxmsg.buf[3]);
    }
  }
  CANTransmitter.write(msg);
}
