#include <Keyboard.h>

void setup() {
  // put your setup code here, to run once:
  delay(1000);
  Keyboard.press(KEY_LEFT_GUI);
  Keyboard.press('r');
  delay(100);
  Keyboard.releaseAll();
  Keyboard.print("cmd");
  delay(1000);
  Keyboard.press(KEY_RETURN);
  delay(100);
  Keyboard.release(KEY_RETURN);
}

void loop() {
  // put your main code here, to run repeatedly:

}
