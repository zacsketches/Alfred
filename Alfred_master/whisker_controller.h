#ifndef WHISKER_CONTROLLER_H
#define WHISKER_CONTROLLER_H NOV_2014

#include <arduino.h>

//gw core
#include <clearinghouse.h>

//gw data structures
#include <Vector.h>
#include <Pair.h>

//gw blocks
//#include <blocks/scanner.h>
//#include <blocks/rover_plant.h>
//#include <blocks/light_plant.h>

//gw messages
#include <messages/bump.h>
#include <messages/five_pt_scan.h>
#include <messages/cmd_velocity.h>
#include <messages/cmd_led.h>

//local components
#include "control_vec.h"

//debug control
#define INCLUDE_CONTROLLER_PRINT  0
#define INCLUDE_DEBUG_SCAN        1
#define INCLUDE_DEBUG_OTHER       0

//*******************************************************************
//*                         WB_CONTROLLER
//* The whisker and bumper controller uses the sonar sensor controlled
//* via the Scanner_5pt class and the bumpers controlled via 
//* the Simple_bumper class.  From this information the controller
//* publishes the cmd_velocity message.
//*
//* Danger_close distance and manuever time after a bump are set as 
//* constant values in the header.  These can be altered based on the 
//* physical performance of the robot.
//* 
//*******************************************************************


//External variables must be declared in the global space of the
//main Arduino sketch.
extern gw::Clearinghouse ch;
extern Two_bumper_msg bumper_msg;
extern Five_pt_scan_msg five_pt_scan_msg;
extern Cmd_velocity_msg cmd_velocity_msg;
extern Cmd_led_msg cmd_led_msg;

namespace Bumper_plant_state {
	//When the bumpers are all clear or maneuvering in reaction
	//to a bump
	enum bp {clear, maneuvering};
}

namespace Bumper_marker {
	//typed identification of which bumper was hit
	enum marker {none, lt, rt};
}

class Whisker_controller : public gw::Node {
private:
	Bumper_plant_state::bp bp_state;
  	Bumper_marker::marker bp_marker;
	
	//Maneuver time is the amount of time the bot reacts to a bump
	int mt;
	Time maneuver_finish;

    //When any of the three middle sensors are inside the 'danger close' 
    // range the bot will react differently than a normal whisker touch.  
    // This allows a custom action when the bot is about to run into
    // something in front of it.
    Danger_close_state::dc dc_state;    // default Danger_close_state::clear
    const static int dc_dist = 25;     //cm
    
    //Pointer to one of the base control vectors in control_vec.h
    const Cmd_velocity_msg* active_control_vec;
    
    // Vector of headings and their associated trip distances set to
    // default values in .begin().  Not set in the contructor so that 
    // the scanner can be fully constructed before the values are taken 
    // for the controller.
    //
    // Uses the constants defined below for defaults
    //
    // IT IS CRITICAL THAT THE SCANNER.BEGIN() METHOD BE CALLED BEFORE
    // THE CONTROLLER.BEGIN() METHOD.
    Vector<Trip_pt> trips;
    static const int side_trip =        100; //cm
    static const int off_the_bow_trip =  80; //cm
    static const int dead_ahead_trip =   50; //cm
	static const int clear_distance =   300; //cm

  	//Publisher/Subscribers and local copies
	gw::Publisher<Cmd_velocity_msg> pub;
	Cmd_velocity_msg local_msg;  
	
    gw::Publisher<Cmd_led_msg> pub2;
	Cmd_led_msg local_led_msg;
	
	gw::Subscriber<Five_pt_scan_msg> scan_sub;
    Five_pt_scan_msg local_scan_msg;
    
    gw::Subscriber<Two_bumper_msg> bumper_sub;
    Two_bumper_msg local_bumper_msg;

	Vector<Scan_pt> scan_data;      //a vector copy of the data in the local_scan_msg

  /*   SENSOR STATE DESCRIPTION    */
  //The least sig five bits represent whiskers.  When a whisker is too
  //close to an obstruction the sensor_state bit is set to 1 and
  //the controller determines the best move away from that obstruction
  /*  In the five position sensor the bits represent an object 
      within the trip sensor distance.
  
                              00100  
                        01000   |  00010
                            \   |  /
                             \  | /
                   10000------ Z------00001
  
      The angle for each whisker is determined by how the scanner is set 
      up.  Use the Scanner_5pt.print() function to see the headings.
  */
    byte sensor_state;
 
    // HELPER FUNCTIONS UNIQUE TO THIS CONTROLLER
	Bumper_plant_state::bp calc_bp_state(Bumper_marker::marker& bm);
    Danger_close_state::dc calc_dc_state();
    byte calc_sensor_state();
    void update_range(Vector<Scan_pt>& vec, const Scan_pt& pt);
    const Cmd_velocity_msg* get_base_vector();
    void publish_message();
	void publish_led_message();
    
public:
    Whisker_controller();

    void begin();   
    void run();

//    const int bump_maneuver_time() const { return mt; }
    const int dc_distance() const { return dc_dist; }
	
	#if INCLUDE_CONTROLLER_PRINT == 1
        void print();
	#endif
	#if INCLUDE_DEBUG_SCAN == 1
	    void show_scan_data();
        void show_sensor_state();
        void show_trips();
	#endif
	#if INCLUDE_DEBUG_OTHER == 1
        void show_active_control();
        void show_bs();
        void show_dc();
    #endif
};

#endif


