/*
	Whisker_bot is an integration of the Glow_worm 5pt Scanner class
	with the Dagu Rover5 chassis.
	
	The robot uses a single Parallax PING))) ultrasonic sensor mounted
	to a servo that rotates it to five different Lines of Bearing(LOBs).
	The range aquired at each LOB is used to wander around trying to
	do obstacle avoidance.
*/ 

//gw core
#include <clearinghouse.h>

//gw data structures
#include <Vector.h>
#include <Pair.h>

//gw messages
#include <messages/five_pt_scan.h>
#include <messages/cmd_velocity.h>
#include <messages/cmd_led.h>

//gw components
#include <blocks/scanner.h>
#include <blocks/rover_plant.h>
#include <blocks/light_plant.h>

//local headers
#include "control_vec.h"
#include "whisker_controller.h"

//other 3rd party libraries
#include <Servo.h>		//for the scanner
#include <Wire.h>		//for the light plant

//Physical connections
const int servo_pin = 6;
const int ping_pin = 7;
const int servo_ctr = 88;
const int span_width = 130;

const int rt_motor_dir = 2;
const int rt_motor_spd = 3;
const int lt_motor_dir = 4;
const int lt_motor_spd = 5;

/*------------Required to initiate the Glow Worm Framework---------------*/
gw::Clearinghouse ch;
int gw::Message::msg_count = 0;
int gw::Node::node_count = 0;

/*------------Create a set of global messsages---------------------------*/
Five_pt_scan_msg five_pt_scan_msg;
Cmd_velocity_msg cmd_velocity_msg;
Cmd_led_msg cmd_led_msg;

/*------------Construct the system components----------------------------*/
//Scanner
Scanner_5pt scanner(servo_pin, ping_pin, servo_ctr, span_width);
//Motors and plant
gw::Motor* mtr_lt = new gw::Motor("lt", lt_motor_dir, lt_motor_spd);  
gw::Motor* mtr_rt = new gw::Motor("rt", rt_motor_dir, rt_motor_spd);
Rover_plant plant(2);
//LEDs and light_plant
//  signature: Led(const char* name, Port::port pt, State::state s = State::off)
Led* led1 = new Led("far_lt",  Port::p0);
Led* led2 = new Led("near_lt", Port::p1);
Led* led3 = new Led("mid",     Port::p2);
Led* led4 = new Led("near_rt", Port::p3);
Led* led5 = new Led("far_rt",  Port::p4);
Light_plant light_plant;
//controller
Whisker_controller controller;

void setup() {
	Serial.begin(115200);
	Serial.println();

	ch.list();

	//start the scanner
	scanner.begin();
	scanner.print();

	//attach motors to the plant
	plant.attach(mtr_lt);
	plant.attach(mtr_rt);
	plant.print();

	//attach LEDs and begin the light_plant
	light_plant.attach(led1);
	light_plant.attach(led2);
	light_plant.attach(led3);
	light_plant.attach(led4);
	light_plant.attach(led5);
	light_plant.begin();
	light_plant.print();

	//begin the controller AFTER beginning the scanner
	//the scanner computes the scan point angles in its
	//begin method, and the controller needs this data in
	//its own begin method.
	controller.begin();
	controller.print();
	//  controller.show_trips();
	//  controller.show_scan_data();

	delay(3000);
}

void loop() {
  scanner.run();
//  five_pt_scan_msg.print();  

  controller.run();
//  controller.show_trips();
//  controller.show_scan_data(); 
//  controller.show_sensor_state();
//  controller.show_dc();
//  controller.show_bs();
//  controller.show_active_control();

  plant.run();
  controller.show_sensor_state();
  light_plant.run();

/*------Inspect global messages-----*/  
  cmd_velocity_msg.print();
  cmd_led_msg.print();
  //  five_pt_scan_msg.print();

   
}
