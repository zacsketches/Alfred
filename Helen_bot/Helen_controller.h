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
	enum ms {clear, maneuvering};
}

//*******************************************************************
//*                         HELEN_CONTROLLER
//*******************************************************************

extern Two_bumper_msg bumper_msg;
extern Cmd_velocity_msg cmd_velocity_msg;
extern Cmd_led_msg cmd_led_msg;
extern Clearinghouse ch;

class Helen_controller : public gw::Node {
private:
	//----------CLASS DATA-----------------
	//Manuever state is either clear or manuevering
	Manuever_state::ms man_state;
  	//Maneuver time is the amount of time the bot reacts to a bump
	int mt;
	Time maneuver_start;
	Time maneuver_finish;
	const Cmd_velocity_msg* current_maneuver;

	//---------BOILER PLATE----------------
  	//Publisher/Subscribers and local copies
	gw::Subscriber<Two_bumper_msg> sub;
    Two_bumper_msg local_bumper_msg;
	
	gw::Publisher<Cmd_velocity_msg> pub;
	Cmd_velocity_msg local_v_msg;
	
	gw::Publisher<Cmd_led_msg> pub2;
	Cmd_led_msg local_led_msg;
	
        
public:
	Helen_controller()
		:Node("helen_cont"),
		sub(&bumper_msg, &ch, local_bumper_msg),
		pub(&cmd_velocity_msg, &ch, local_v_msg),
		pub2(&cmd_led_msg, &ch, local_led_msg),
		mt(2000),	//milliseconds
		man_state(Manuever_state::clear)
	{}
	
	// From Base class
    //    char* name()
	//    int id()
	
	void begin() {}
    
	void run() {
		Time now = millis();
		//update the control effort
		sub.update();
		
		/*---------------Find control vector--------------------*/
		//if clear then find the right vector, but if
		//maneuvering then continue until the maneuver
		//is complete.
		
		if(man_state  == Manuever_state::clear) {
			//default easy forward
			current_maneuver = &Control::forward_easy;
			local_v_msg.update(current_maneuver);

			//if right bumped then back_twist_CCW
			if(local_bumper_msg.pressed_rt==Bump_state::pressed) {
				man_state = Manuever_state::maneuvering;
				maneuver_finish = now + mt;
				current_maneuver = &Control::backtwist_CCW;
				local_v_msg.update(current_maneuver);
			}
			
			//if left bumped then back_twist_CW
			if(local_bumper_msg.pressed_lt==Bump_state::pressed) {
				man_state = Manuever_state::maneuvering;
				maneuver_finish = now + mt;
				current_maneuver = &Control::backtwist_CW;
				local_v_msg.update(current_maneuver);
			}
		} else {
			if(now > maneuver_finish) {
				man_state = Manuever_state::clear;
				local_v_msg.update(&Control::forward_easy);
			} else {
				local_v_msg.update(current_maneuver);
			}
		}

		/*---------------Write LED commands to local---------------*/
		if(man_state == Manuever_state::maneuvering) {
			if(current_maneuver == &Control::backtwist_CCW) { //bumped on the right
				local_led_msg.far_lt  = Led_state::off;
				local_led_msg.near_lt = Led_state::off;
				local_led_msg.mid     = Led_state::on;
				local_led_msg.near_rt = Led_state::on;
				local_led_msg.far_rt  = Led_state::on;
			}
			if(current_maneuver == &Control::backtwist_CW) { //bumped on the left
				local_led_msg.far_lt  = Led_state::on;
				local_led_msg.near_lt = Led_state::on;
				local_led_msg.mid     = Led_state::on;
				local_led_msg.near_rt = Led_state::off;
				local_led_msg.far_rt  = Led_state::off;
			}
		} else {
				local_led_msg.far_lt  = Led_state::off;
				local_led_msg.near_lt = Led_state::off;
				local_led_msg.mid     = Led_state::off;
				local_led_msg.near_rt = Led_state::off;
				local_led_msg.far_rt  = Led_state::off;
		}
		
		pub.publish();
		pub2.publish();
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

