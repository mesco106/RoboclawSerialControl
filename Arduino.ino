#include "SerialPort.h"
#include "Gpio.h"
#include "RoboClaw.h"

void run(const String &message);

SerialPort port(115200, run);
Gpio led(LED_BUILTIN);
RoboClaw roboclaw(&Serial2, 10000);

#define address 0x80
#define Kp 1.0
#define Ki 0.5
#define Kd 0.25
#define qpps 44000

void setup() {
  roboclaw.begin(38400);
  port.setup();
  led.setup();

  //Set PID Coefficients
  roboclaw.SetM1VelocityPID(address,Kd,Kp,Ki,qpps);
  roboclaw.SetM2VelocityPID(address,Kd,Kp,Ki,qpps); 
}

void loop() {

  port.update();
}

void run(const String &message) {
  auto action = command(message);

  if (action == "halt") {
    move(0, 0);

  } else if (action == "look") {
    float angle = argument(message, 1);
    look(angle);


  } else if (action == "on") {
    led.on();


  } else if (action == "off") {
    led.off();


  } else if (action == "hi") {
    port.send("Hi there!");

    /*__________________________MOTOR COMMANDS_________________________________*/
  } else if (action == "move") {
    String param = argumentString(message, 1);

    if (param == "-f" || param == "--forward") {
      float speed = argument(message, 2);
      roboclaw.ForwardM1(address, speed);
      roboclaw.ForwardM2(address, speed);
      port.send("Moving forwards at: " + String(speed));

    } else if (param == "-b" || param == "--backward") {
      float speed = argument(message, 2);
      roboclaw.BackwardM1(address, speed);
      roboclaw.BackwardM2(address, speed);
      port.send("Moving backwards at: " + String(speed));

    } else if (param == "-l" || param == "--left") {
      float speed = argument(message, 2);
      roboclaw.ForwardM2(address, speed);
      roboclaw.BackwardM1(address, speed);
      port.send("Moving left at: " + String(speed));

    } else if (param == "-r" || param == "--right") {
      float speed = argument(message, 2);
      roboclaw.ForwardM1(address, speed);
      roboclaw.BackwardM2(address, speed);
      port.send("Moving right at: " + String(speed));
    }

  } else if (action == "stop" || action == "s") {
    stop();
    port.send("Motors Stopped!");
    /*__________________________SET COMMANDS_______________________________*/

  } else if (action == "set") {
    String param = argumentString(message, 1);

    if (param == "-e1" || param == "--encoder1") {
      int32_t value = argument(message, 2);
      roboclaw.SetEncM1(address, value);
      port.send("Encoder 1 setup to: ");
      port.send(value);

    } else if (param == "-e2" || param == "--encoder2") {
      int32_t value = argument(message, 2);
      roboclaw.SetEncM2(address, value);
      port.send("Encoder 2 setup to: ");
      port.send(value);

    } else if (param == "-es" || param == "--encoders") {
      String confirmation = argumentString(message, 2);
      if (confirmation == "-r" || confirmation == "--reset") {
        roboclaw.ResetEncoders(address);
        port.send("Encoders reseted!");
      }
    }

    /*__________________________READ COMMAND_________________________________*/

  } else if (action == "read") {
    String param = argumentString(message, 1);

    if (param == "-e1" || param == "--encoder1") {
      port.send("Encoder 1: ");
      port.send(roboclaw.ReadEncM1(address));

    } else if (param == "-e2" || param == "--encoder2") {
      port.send("Encoder 2: ");
      port.send(roboclaw.ReadEncM2(address));
    } else if (param == "-es" || param == "--encoders") {
      port.send("Encoder 1: ");
      port.send(roboclaw.ReadEncM1(address));
      port.send("Encoder 2: ");
      port.send(roboclaw.ReadEncM2(address));

    } else if (param == "-t" || param == "--temperature") {
      uint16_t temp;
      roboclaw.ReadTemp(address, temp);
      port.send(temp);

    } else if (param == "-t2" || param == "--temperature2") {
      uint16_t temp2;
      roboclaw.ReadTemp2(address, temp2);
      port.send(temp2);

    } else if (param == "-b" || param == "--battery") {
      port.send(roboclaw.ReadMainBatteryVoltage(address));

    } else if (param == "-v" || param == "--version") {
      char version[32];
      if (roboclaw.ReadVersion(address, version)) {
        port.send(version);
      }

    } else if (param == "-s1" || param == "--speed1") {
      port.send("Speed M1: ");
      port.send(roboclaw.ReadSpeedM1(address));

    } else if (param == "-s2" || param == "--speed2") {
      port.send("Speed M2: ");
      port.send(roboclaw.ReadSpeedM2(address));

    } else if (param == "-ss" || param == "--speeds") {
      port.send("Speed M1: ");
      port.send(roboclaw.ReadSpeedM1(address));
      port.send("Speed M2: ");
      port.send(roboclaw.ReadSpeedM2(address));
    }
  } else {
    /*__________________________ERROR COMMANDS_______________________________*/
    port.send("Ops, '" + message + "' command not found!");
    for (int i = 0; i < 10; ++i) {
      port.send(token(message, i));
    }
  }
}


/*__________________________FUNCTIONS_________________________________*/


void move(float vx, float vy) {
  Serial.print("Roboclaw set velocity: ");
  Serial.print(vx);
  Serial.print(" ");
  Serial.println(vy);
}

void look(float angle) {
}

void forward(String Motor, int speed) {
  if (Motor == "m1") {
    roboclaw.ForwardM1(address, speed);
    port.send("Motor " + Motor + " at speed of: " + speed);
  } else if (Motor == "m2") {
    roboclaw.ForwardM2(address, speed);
    port.send("Motor " + Motor + " at speed of: " + speed);
  } else if (Motor == "ms") {
    roboclaw.ForwardM1(address, speed);
    roboclaw.ForwardM2(address, speed);
    port.send("Motor " + Motor + " at speed of: " + speed);
  }
}

void backward(String Motor, int speed) {
  if (Motor == "m1") {
    roboclaw.BackwardM1(address, speed);
    port.send("Motor " + Motor + " at speed of: " + speed);
  } else if (Motor == "m2") {
    roboclaw.BackwardM2(address, speed);
    port.send("Motor " + Motor + " at speed of: " + speed);
  } else if (Motor == "ms") {
    roboclaw.BackwardM1(address, speed);
    roboclaw.BackwardM2(address, speed);
    port.send("Motor " + Motor + " at speed of: " + speed);
  }
}

void right(float speed) {
  port.send("turning left");
  roboclaw.ForwardM1(address, speed);
  roboclaw.BackwardM2(address, speed);
}

void left(float speed) {
  port.send("turning right");
  roboclaw.ForwardM2(address, speed);
  roboclaw.BackwardM1(address, speed);
}

void stop() {
  roboclaw.ForwardM1(address, 0);
  roboclaw.ForwardM2(address, 0);
}

void readTemperature() {
  port.send("The temperature is 78 F");
}