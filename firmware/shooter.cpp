#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>

#include <Arduino.h>

#include <Servo.h>

class Victor
{
  private:
    Servo controller;
    int goal;
    int cur;
    int pin;
    //Internal set command to write to controller PWM
    void _set(int s)
    {
      controller.writeMicroseconds(x);
      cur = x;
    }
  public:
    Victor(int p)
    {
      pin = p;
      controller = Servo();
      goal = 1500;
      controller.attach(pin);
      _set(goal);
    }
    void set(int speed)
    {
      goal = map(speed,-100,100, 1000,2000);
    }
    int get()
    {
      return map(cur,1000,2000,-100,100);
    }
    void off()
    {
      set(0);
    }
    void on()
    {
      set(100);
    }
    void reverse()
    {
      set(-100);
    }
    //Should be called in each loop so PWM slowly ramps up, doesn't work otherwise
    void run()
    {
      if (cur != goal)
      {
        if (goal == 1500) _set(1500);
        else if (goal < 1500)
        {
          _set(cur - 100);
        }
        else if (goal > 1500)
        {
          _set(cur + 100);
        }
      }
    }
};

class Feeder
{
  private:

  public:
    Victor motor;
    Feeder (int motorPin)
    {
      motor = Victor(motorPin);
    }
    void run()
    {
      
    }
};

static Feeder feeder;
static Victor shooter;

class Comms
{
  private:
    //ROS
    ros::NodeHandle nh;
    std_msgs::String str_msg;
    //ros::Publisher chatter;
    ros::Subscriber<std_msgs::String> sub;

    static void messageCallback(const std_msgs::String& str_msg)
    {
      String s = str_msg.data;
      if (s == "flyon")
        shooter.on();
      else if (s == "flyoff")
        shooter.off();
      else if (s == "feedon")
        feeder.motor.on();
      else if (s == "feedoff")
        feeder.motor.off();
      else if (s == "feedreverse")
        feeder.motor.reverse();
      else if (s == "ledon")
        digitalWrite(13,HIGH);
      else if (s == "ledoff")
        digitalWrite(13,LOW);
    }
  public:
    Comms() :
      shooter(5),
      feeder(6),
      str_msg(),
      sub("shooter_control",&messageCallback)
      //chatter("chatter", &str_msg)
    {
      pinMode(13,OUTPUT);
      nh.initNode();
      nh.subscribe(sub);
    }
    void run()
    {
      nh.spinOnce();
      shooter.run();
      feeder.run();
    }
};

static Comms com;
void setup()
{
  shooter = Victor(5);
  feeder = Feeder(6);
  com = Comms();
}

void loop()
{
  com.run();
  delay(100);
}
