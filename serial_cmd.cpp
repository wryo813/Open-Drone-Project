#include <Arduino.h>
#include "serial_cmd.h"


void serial_cmd(String cmd_start, String msg_send) {
  boolean x = 0;
  if ( Serial.available() > 0 ) {
    String cmd_arrive = Serial.readStringUntil('\n');
    if (cmd_arrive == cmd_start ) {
      x = 1;
    }
    else if (cmd_arrive == "stop" ) {
      x = 0;
    }
  }
  if (x) {
    Serial.println(msg_send);
  }
}
