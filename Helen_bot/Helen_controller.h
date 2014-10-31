#ifndef HELEN_CONTROLLER_H
#define HELEN_CONTROLLER_H OCT_2014

#include <arduino.h>
#include "Control_vec.h"

//gw core
#include <clearinghouse.h>

//gw data structures
#include <Vector.h>
#include <Pair.h>

//gw messages
#include <messages/bump.h>
#include <messages/cmd_velocity.h>

//gw components

namespace Manuever_state {
	enum ms {clear, manuevering};
}

//*******************************************************************
//*                         HELEN_CONTROLLER
//*******************************************************************

extern Two_bumper_msg bumper_msg;
extern Cmd_velocity_msg cmd_velocity_msg;
extern Clearinghouse ch;

class Helen_controller : public gw::Node {
	//----------CLASS DATA-----------------
	//Manuever state is either clear or manuevering
	Manuever_state::ms man_state;
  	//Maneuver time is the amount of time the bot reacts to a bump
	int mt;

	//---------BOILER PLATE----------------
  	//Publisher/Subscribers and local copies
	gw::Subscriber<Two_bumper_msg> sub;
    Two_bumper_msg local_bumper_msg;
	
	gw::Publisher<Cmd_velocity_msg> pub;
	Cmd_velocity_msg local_v_msg;
        
public:
	Helen_controller()
		:Node("helen_cont"),
		sub(&bumper_msg, &ch, local_bumper_msg),
		pub(&cmd_velocity_msg, &ch, local_v_msg),
		mt(2000),	//milliseconds
		man_state(Manuever_state::clear)
	{}
	
	// From Base class
    //    char* name()
	//    int id()
	
	void begin() {}
    
	void run() {
		//update the control effort
		sub.update();
		
		//pointer test
//		const Cmd_velocity_msg* test = &Control::forward_easy;
//		test->print();
//		
		//default easy forward
		local_v_msg.update(&Control::forward_easy);
		
		//if right bumped then back_twist_CCW
		if(local_bumper_msg.pressed_rt==Bump_state::pressed) {
			local_v_msg.update(&Control::backtwist_CCW);
		}
		
		//if left bumped then back_twist_CW
		if(local_bumper_msg.pressed_lt==Bump_state::pressed) {
			local_v_msg.update(&Control::backtwist_CW);
		}
		
		pub.publish();
	}
	
	//maneuver time
	int maneuver_time() const { return mt; }
	void set_maneuver_time(const int val) { mt = val;}
	
	#if INCLUDE_PLANT_PRINT == 1
	    void print(){
			Serial.print(F("Node id: "));
			Serial.print(id());
			Serial.print("\t");
			Serial.println(name());   
			Serial.print("\t");
			Serial.print(F("\tSubscribed to: "));
			sub.subscribed_where();
			Serial.print(F("\tPublishing to: "));
			pub.publishing_where();
	    }
	#endif
};
#endif