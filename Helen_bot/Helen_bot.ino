//gw core
#include <clearinghouse.h>

//gw data structures
#include <Vector.h>
#include <Pair.h>

//gw messages
#include <messages/bump.h>
#include <messages/cmd_velocity.h>

//gw components
#include <blocks/rover_plant.h>
#include <blocks/bumper_plant.h>
#include "helen_controller.h"

//local headers
#include "Control_vec.h"

//physical connections
const int rt_dir_pin = 2;
const int rt_pwm_pin = 3;
const int lt_dir_pin = 4;
const int lt_pwm_pin = 5;
const int rt_bumper_pin = 13;
const int lt_bumper_pin = 11;

/*------------Required to initiate the Glow Worm Framework---------------*/
gw::Clearinghouse ch;
int gw::Message::msg_count = 0;
int gw::Node::node_count = 0;

/*------------Create a set of global messsages---------------------------*/
Two_bumper_msg bumper_msg;
Cmd_velocity_msg cmd_velocity_msg;

/*---------Construct Plant and Motors------------------------------*/
Rover_plant plant;       
//Motor(name, dir_pin, pwm_pin, Position::position)
gw::Motor* lt_motor = 
  new gw::Motor("lt_mtr", lt_dir_pin, lt_pwm_pin, Position::lt);  
gw::Motor* rt_motor = 
  new gw::Motor("rt_mtr", rt_dir_pin, rt_pwm_pin, Position::rt);  

/*---------Construct bumpers and bumper plant--------------------------*/
Bumper* lt_bumper = new Bumper(lt_bumper_pin, Position::lt);
Bumper* rt_bumper = new Bumper(rt_bumper_pin, Position::rt);
Bumper_plant bumper_plant;    

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
	
	//attach bumpers to the bumper_controller
	bumper_plant.attach(lt_bumper);
	bumper_plant.attach(rt_bumper);
	bumper_plant.begin();
	bumper_plant.print();
	
	//begin the helen controller
	controller.begin();
	controller.print();

  delay(3000);
}

void loop() {
    bumper_plant.run();
    controller.run();
    plant.run();
   cmd_velocity_msg.print();
}




