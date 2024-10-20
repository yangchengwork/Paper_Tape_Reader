// 用于学习TimerOne定时器

#include <TimerOne.h>

unsigned char readOffset;
unsigned char writeOffset;
unsigned char buf[256];

void callback() {
  Serial.print("read:");
  Serial.print(readOffset);
  Serial.print(" write:");
  Serial.println(writeOffset);
  if(readOffset != writeOffset) {
    readOffset++;
  } else {
    Timer1.stop();
  }
}

void setup() {
  // put your setup code here, to run once:
  Timer1.initialize(500000);
  Timer1.attachInterrupt(callback);
  readOffset = 0;
  writeOffset = 2;
  // Timer1.start();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (readOffset == writeOffset) {
    delay(1000);
    writeOffset++;   
    Timer1.restart(); 
  }
}
