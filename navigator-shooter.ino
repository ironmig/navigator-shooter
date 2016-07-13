#include <Servo.h>

class Shooter
{
  private:

  public:
    static const int pin = 5;
    static Servo controller;
    static int goal;

    static void init()
    {
      controller.attach(pin);
      goal = 1500;
    }
    static void set(int x)
    {
      goal = constrain(x,1000,2000);
    }
    static void on()
    {
      set(2000);
    }
    static void off()
    {
      set(1500);
    }
    static void run()
    {
      int cur = controller.read();
      if (cur != goal)
      {
        if (cur < goal)
          controller.write(cur+100);
        else controller.write(goal);
      }
    }
};
int Shooter::goal = 0;
Servo Shooter::controller = Servo();

class Comms
{
  private:
    static const char ON_CHAR = 'i';
    static const char OFF_CHAR = 'o';
  public:
    static void init() {Serial.begin(9600);}
    static void run()
    {
      if (Serial.available() > 0)
      {
        char c = Serial.read();
        if (c == ON_CHAR) Shooter::on();
        else if (c == OFF_CHAR) Shooter::off();
      }
    }
};

void setup()
{
  Comms::init();
  Shooter::init();
}

void loop()
{
  Comms::run();
  Shooter::run();
  delay(50);
}

