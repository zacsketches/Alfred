//gw core
#include <clearinghouse.h>

//gw data structures
#include <Vector.h>
#include <Pair.h>

//gw messages
#include <messages/bump.h>
#include <messages/cmd_velocity.h>
#include <messages/cmd_led.h>

//gw components
#include <blocks/rover_plant.h>
#include <blocks/bumper_plant.h>
#include <blocks/light_plant.h>

//local headers
#include "Control_vec.h"
#include "helen_controller.h"

//other 3rd party libraries
#include <Wire.h>

//physical connections
const int rt_dir_pin = 2;
const int rt_pwm_pin = 3;
const int lt_dir_pin = 4;
const int lt_pwm_pin = 5;
const int rt_bumper_pin = 12;
const int lt_bumper_pin = 13;

/*------------Required to initiate the Glow Worm Framework---------------*/
gw::Clearinghouse ch;
int gw::Message::msg_count = 0;
int gw::Node::node_count = 0;

/*------------Create a set of global messsages---------------------------*/
Two_bumper_msg bumper_msg;
Cmd_velocity_msg cmd_velocity_msg;
Cmd_led_msg cmd_led_msg;

/*---------Construct Plant and Motors------------------------------*/
Rover_plant plant;       
//Motor(name, dir_pin, pwm_pin, Position::position)
gw::Motor* lt_motor = 
  new gw::Motor("lt_mtr", lt_dir_pin, lt_pwm_pin, Position::lt);  
gw::Motor* rt_motor = 
  new gw::Motor("rt_mtr", rt_dir_pin, rt_pwm_pin, Position::rt);  

/*---------Construct bumpers and bumper plant--------------------------*/
Bumper* lt_bumper = new Bumper("lt_bump", lt_bumper_pin, Position::lt);
Bumper* rt_bumper = new Bumper("rt_bump", rt_bumper_pin, Position::rt);
Bumper_plant bumper_plant;    

/*---------Construct LEDs and LED Plant-------------------------------*/
//	Led(const char* name, Port::port pt, State::state s = State::off)
Led* led1 = new Led("far_lt",  Port::p0);
Led* led2 = new Led("near_lt", Port::p1);
Led* led3 = new Led("mid",     Port::p2);
Led* led4 = new Led("near_rt", Port::p3);
Led* led5 = new Led("far_rt",  Port::p4);
Light_plant light_plant;

/*---------Construct Helen Keller controller----------------------------------*/
Helen_controller controller;

void setup() {
    Serial.begin(115200);
    Serial.println();    //blank line
  
	//inspect the contents of the clearinghouse
	ch.list();
  
    //attach motors to the plant
    plant.attach(lt_motor);
    plant.attach(rt_motor);
    plant.begin();
    plant.print();
	
	//attach bumpers to the bumper plant
	bumper_plant.attach(lt_bumper);
	bumper_plant.attach(rt_bumper);
	bumper_plant.begin();
	bumper_plant.print();
	
	//attach LEDs and begin the light_plant
	light_plant.attach(led1);
	light_plant.attach(led2);
	light_plant.attach(led3);
	light_plant.attach(led4);
	light_plant.attach(led5);
	light_plant.begin();
	light_plant.print();
	
	//begin the helen controller
	controller.begin();
	controller.print();

  delay(3000);
}

void loop() {
    bumper_plant.run();
    controller.run();
    plant.run();
    light_plant.run();
//   cmd_velocity_msg.print();
    cmd_led_msg.print();
}




