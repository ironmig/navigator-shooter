#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>

#include <Arduino.h>

#include <Servo.h>

const int SHOOTER_PIN = 5;
const int FEEDER_MOTOR_PIN = 6;
const int FEEDER_IN_PIN = 7;

class Victor
{
  private:
    Servo controller;
    int goal;
    int cur;
    int pin;
    unsigned long stopAtTime;
    bool autoStop;
    //Internal set command to write to controller PWM
    void _set(int s)
    {
      controller.writeMicroseconds(s);
      cur = s;
    }
  public:
    Victor(int p)
    {
      pin = p;
      controller = Servo();
      goal = 1500;
      autoStop = false;
    }
    void init()
    {
      controller.attach(pin);
      _set(goal);
    }
    void set(int speed)
    {
		autoStop = false;
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
	  if (autoStop)
	  {
		  if (millis() >= stopAtTime)
		  {
			  autoStop = false;
			  off();
		  }
		}
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
    void stopAt(unsigned long time)
    {
		autoStop = true;
		stopAtTime = time;
	}
};
Victor shooter(SHOOTER_PIN);

class Feeder
{
  private:
	static const unsigned long MAX_SHOOT_TIME = 5000;
	static const unsigned long SHOOT_AFTER_FEED_TIME = 3000;
	int ir_input;
	bool shooting;
	bool ir_last;
	bool ir_cur;
	bool ir_last_last;
	bool has_been_one;
	void startShoot()
	{
		shooting = true;
		has_been_one = false;
		ir_last = true;
		shooter.on();
	}
	void stopShoot()
	{
		shooting = false;
		motor.off();
		unsigned long stopShootAt = millis() + SHOOT_AFTER_FEED_TIME;
		shooter.stopAtTime(stopShootAt);
	}
  public:
    Victor motor;
    Feeder (int motorPin, int irPin) :
      motor(motorPin)
    {
		ir_input = irPin;
		pinMode(irPin,INPUT);
		ir_last = false;
		ir_cur = false;
		started_shooting = 0;
    }
    void init()
    {
      motor.init();
    }
    void run()
    {
	  motor.run();
      if (shooting)
      {
		unsigned long elapsed = millis() - started_shooting;
		if (elapsed > MAX_SHOOT_TIME)
		{
			stopShoot();
			return;
		}
		ir_cur = getIR();
		//If ir has changed
		if (ir_cur != ir_last)
		{
			if (ir_cur && !ir_last)
			{
				has_been_one = true;
			}
			if (!has_been_one)
			{
				motor.on();
			}
			else if (has_been_one && ir_cur)
			{
				motor.on();
			}
			else if (!ir_cur && ir_last && has_been_one)
			{
				stopShoot();
			}
		}
				
		//at end
		ir_last = ir_cur;
	  }

    }
    void autoFeedOne()
    {
		startShoot();
		//motor.on();
	}
	void cancelAutoFeed()
	{
		shooting = false;
		shooter.off();
		motor.off();
	}
    bool getIR()
    {
		return digitalRead(ir_input);
	}
	void shootMode()
	{
		shooting = true;

	}
};
Feeder feeder(FEEDER_MOTOR_PIN,FEEDER_IN_PIN);

class Comms
{
  private:
    //ROS
    ros::NodeHandle nh;
    std_msgs::String str_msg;
    std_msgs::String ir_msg;
    ros::Publisher chatter;
    ros::Publisher ir_status;
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
      else if (s == "feedauto")
		feeder.autoFeedOne();
      else if (s == "ledon")
        digitalWrite(13,HIGH);
      else if (s == "ledoff")
        digitalWrite(13,LOW);
    }
  public:
    Comms() :
      str_msg(),
      ir_msg(),
      sub("shooter_control",&messageCallback),
      chatter("chatter", &str_msg),
      ir_status("ir",&ir_msg)
    {
      pinMode(13,OUTPUT);
    }
    void init()
    {
      nh.initNode();
      nh.subscribe(sub);
      nh.advertise(chatter);
      nh.advertise(ir_status);   
    }
    void run()
    {
      nh.spinOnce();
      
      
      int feeder_speed = feeder.motor.get();
      if (feeder_speed == 100) str_msg.data = "on";
      else if (feeder_speed == 0) str_msg.data = "off";
      else str_msg.data = "err";
      chatter.publish(&str_msg);
      
      bool ir_on = feeder.getIR();
      if (ir_on) ir_msg.data = "ir on";
      else if (!ir_on) ir_msg.data = "ir off";
      ir_status.publish(&ir_msg);
    }
};

Comms com;
void setup()
{
  shooter.init();
  feeder.init();
  com.init();
}

void loop()
{
  com.run();
  shooter.run();
  feeder.run();
  delay(100);
}
