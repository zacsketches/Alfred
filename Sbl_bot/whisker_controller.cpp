#include "whisker_controller.h"

/* void Whisker_controller::update_scan_vector(Vector<Scan_pt>& vec, Five_pt_scan_msg& msg) {
    update_range(vec, msg.p4);
    update_range(vec, msg.p2);
    update_range(vec, msg.p0);
    update_range(vec, msg.p1);
    update_range(vec, msg.p3);
} */

//*******************************************************************
//*                         CONSTRUCTOR
//*******************************************************************
Whisker_controller::Whisker_controller()
    :Node("whisker_cont"),
    bumper_sub(&bumper_msg, &ch, local_bumper_msg),
    scan_sub(&five_pt_scan_msg, &ch, local_scan_msg),
    pub(&cmd_velocity_msg, &ch, local_msg),
    pub2(&cmd_led_msg, &ch, local_led_msg),
    mt(2000),	//milliseconds
    bp_state(Bumper_plant_state::clear),
    bp_marker(Bumper_marker::none)
{    
    //b_state = Bump_state::clear;
    sensor_state = 0x00;
    dc_state = Danger_close_state::clear;
}

//*******************************************************************
//*                         REQUIRED VIRTUAL FUNCTIONS
//*******************************************************************
void Whisker_controller::begin() {
    scan_sub.update();  //load the local message with the headings in
                        //the scan
        
    //set up trip points for each heading
    int heading4 = local_scan_msg.p4.heading();
    int heading2 = local_scan_msg.p2.heading();
    int heading0 = local_scan_msg.p0.heading();
    int heading1 = local_scan_msg.p1.heading();
    int heading3 = local_scan_msg.p3.heading();
    
    // trips.reserve(5);
    trips.push_back(Trip_pt(heading4, side_trip,        0x10));
    trips.push_back(Trip_pt(heading2, off_the_bow_trip, 0x08));
    trips.push_back(Trip_pt(heading0, dead_ahead_trip,  0x04));
    trips.push_back(Trip_pt(heading1, off_the_bow_trip, 0x02));
    trips.push_back(Trip_pt(heading3, side_trip,        0x01));
    	
    //set up the original copy of the scan_data vector;
    scan_data.reserve(5);
    scan_data.push_back(Scan_pt(heading4, clear_distance));
    scan_data.push_back(Scan_pt(heading2, clear_distance));
    scan_data.push_back(Scan_pt(heading0, clear_distance));
    scan_data.push_back(Scan_pt(heading1, clear_distance));
    scan_data.push_back(Scan_pt(heading3, clear_distance));
}

void Whisker_controller::run() {
 
        scan_sub.update();
		bumper_sub.update();
        
	    update_range(scan_data, local_scan_msg.p4);
	    update_range(scan_data, local_scan_msg.p2);
	    update_range(scan_data, local_scan_msg.p0);
	    update_range(scan_data, local_scan_msg.p1);
	    update_range(scan_data, local_scan_msg.p3);

		bp_state = calc_bp_state(bp_marker);
        dc_state = calc_dc_state();
        sensor_state = calc_sensor_state();
        
        active_control_vec = get_base_vector();
        
		publish_message();
		publish_led_message();
}

//*******************************************************************
//*                         HELPER FUNCTIONS
//*******************************************************************

Bumper_plant_state::bp Whisker_controller::calc_bp_state(Bumper_marker::marker& bm) {
	Time now = millis();
    Bumper_plant_state::bp res = Bumper_plant_state::clear;
    
	//If there is no active bump maneuver then poll the bumpers, 
	//but if in the middle of maneuver then keep going until
	//its finished.
	if(bp_state  == Bumper_plant_state::clear) {
		//if right bumped then mark it
		if(local_bumper_msg.pressed_rt==Bump_state::pressed) {
			res = Bumper_plant_state::maneuvering;
			maneuver_finish = now + mt;
			bm = Bumper_marker::rt;
			}
		
		//if left bumped then mark it
		if(local_bumper_msg.pressed_lt==Bump_state::pressed) {
			res = Bumper_plant_state::maneuvering;
			maneuver_finish = now + mt;
			bm = Bumper_marker::lt;
		}
	} else {
		//Alfred is already maneuvering in this case
		if(now > maneuver_finish) {
			res = Bumper_plant_state::clear;
			bm = Bumper_marker::none;
		} else {
			res = Bumper_plant_state::maneuvering;
		}
	}
    return res;
}


void Whisker_controller::update_range(Vector<Scan_pt>& vec, const Scan_pt& pt) {
    for(int i = 0; i < vec.size(); ++i) {
        if(pt.heading() == vec[i].heading()){
            vec[i].set_range(pt.range());
        }
    }
}

Danger_close_state::dc Whisker_controller::calc_dc_state() {
/*
    TODO implement a more general version of danger_close() that is based on the 
    center heading of the scan +/- some offset instead of hard coded to the
    middle three sensors.
*/
    Danger_close_state::dc res = Danger_close_state::clear;

    //only take danger close from the middle three sensors
    int range0 = local_scan_msg.p0.range();
    int range1 = local_scan_msg.p1.range();
    int range2 = local_scan_msg.p2.range();

    if(range0 < dc_dist) res = Danger_close_state::danger_close;
    if(range1 < dc_dist) res = Danger_close_state::danger_close;
    if(range2 < dc_dist) res = Danger_close_state::danger_close;

    return res;
}

byte Whisker_controller::calc_sensor_state(){

    byte res = 0x00;
    
    // the Vector<Trip_pt> trips contains all the headings and trip distances for the
    // controller.  Compare the data in the vector scan_data against the trip distances
    // and set the result byte accordingly.  See the whisker diagram in the .h file 
    // to undertand the bit shifting required to create the mask for each heading.

    for(int i = 0; i < scan_data.size(); ++i) {
        for(int j = 0; j < trips.size(); ++j) {
            //if the scan heading matches the trip heading then compare ranges
            if(scan_data[i].heading() == trips[j].heading()){
                if(scan_data[i].range() < trips[j].range()) {
                    res |= trips[j].flag();
                }
            }
        }
    }
    return res;
}

const Cmd_velocity_msg* Whisker_controller::get_base_vector() {
	
	//Default to easy forward
    const Cmd_velocity_msg* res;
    res = &Control::forward_easy;
    
  	//************************************************************************
	//*                         BUMPS
	//************************************************************************
	//if the bp_state is maneuvering then figure out which direction and 
	//set the right vector.
	if(bp_state == Bumper_plant_state::maneuvering) {
		if(bp_marker == Bumper_marker::rt) res = &Control::backtwist_CCW;
		if(bp_marker == Bumper_marker::lt) res = &Control::backtwist_CW;
		return res;
	}
	
	//************************************************************************
	//*                         DANGER CLOSE
	//************************************************************************
    if(dc_state == Danger_close_state::danger_close) {
		res = &Control::back;
		return res;
    }

	//************************************************************************
	//*                         WHISKER TOUCHES
	//************************************************************************
    // see .h for details but the five whisker bits are represented by
	// 0x10 0x08 0x04 0x02 0x01
	byte b = sensor_state;
	
	//if clear on all bits
    if(b == B00000 ) {    //no bits set
        res = &Control::forward;
        return res;
    }
    
    //front sensor tripped
    if ((b & 0x4) == 0x04 ) {  //center sensor is tripped then rotate right
        res = &Control::twist_right;
        return res;
    }
	
    /*
    	TODO use this needle threader code
    
    //Both forward quarter sensors tripped then try to use the Needle Threader.
    //Saw this as a problem during experimentation
    if(bitRead(b, 2) && bitRead(b, 3)     ) {
        #if DEBUG_THREAD
            Serial.println(F("From Whisker_controller::set_control_vec, attempting to thread the needle"));
        #endif
        res = thread_the_needle();
        return res;         
    }  
    */  
    
    //forward quarter sensors tripped
    if ((b & 0x08)==0x08)           { // 01000 tripped then swerve right
        res = &Control::swerve_right;
        return res;
    }
    if ((b & 0x02)==0x02)			{ // 00010 tripped then swerve left
        res = &Control::swerve_left;
        return res;
    }

    //just a trip on the side sensors then ease right or left
    switch(b) {
        case 0x10: //left side trip... ease to the right
            res = &Control::ease_right;
            return res;
        case 0x01: //right side trip...ease to the left
            res = &Control::ease_left;
            return res;
    }
    
	return res;
}

void Whisker_controller::publish_message() {
    //update the local copy of the Cmd_velocity_msg
	//and then publish it.

    local_msg.l_dir = active_control_vec->l_dir;
    local_msg.l_spd = active_control_vec->l_spd;
    local_msg.r_dir = active_control_vec->r_dir;
    local_msg.r_spd = active_control_vec->r_spd;

    pub.publish();
}

void Whisker_controller::publish_led_message() {
	//update the local copy of the Cmd_led_msg and
	//then publish it. Sensor state shows which whiskers are tripped.
	//inspect each bit and set the appropriate led to Led_state::on
	local_led_msg.far_lt  = (sensor_state & 0x10) ? Led_state::on : Led_state::off;
	local_led_msg.near_lt = (sensor_state & 0x08) ? Led_state::on : Led_state::off;
	local_led_msg.mid     = (sensor_state & 0x04) ? Led_state::on : Led_state::off;
	local_led_msg.near_rt = (sensor_state & 0x02) ? Led_state::on : Led_state::off;
	local_led_msg.far_rt  = (sensor_state & 0x01) ? Led_state::on : Led_state::off;

	pub2.publish();
}

#if INCLUDE_CONTROLLER_PRINT == 1 
void Whisker_controller::print() {
    char buf[75];

    //print config data to serial
    Serial.println(F("Whisker controller configuration:"));
    Serial.print(F("\tNode id is: "));
    Serial.println(id());
    Serial.print(F("\tPublishing to: "));
    pub.publishing_where();
}
#endif
#if INCLUDE_DEBUG_SCAN == 1
    void Whisker_controller::show_scan_data(){
	    Serial.println(F("Vector<Scan_pt> scan_data contains:"));
	    for(int i = 0; i < scan_data.size(); ++i) {
	        Serial.print(F("\t"));
	        Serial.println(text(scan_data[i]));
	    }
    }
	void Whisker_controller::show_trips() {
	    Serial.println(F("Vector<Trip_pt> trips contains:"));
	    for(int i = 0; i < trips.size(); ++i) {
	        Serial.print(F("\t"));
	        Serial.println(text(trips[i]));
	    }    
	}
	void Whisker_controller::show_sensor_state() {
	    Serial.print(F("byte sensor_state contains:"));
	    char buf[10];
		sprintf(buf, "0x%02x", sensor_state);
	    Serial.println(buf);
	}
#endif
#if INCLUDE_DEBUG_OTHER == 1
	void Whisker_controller::show_active_control(){
	    Serial.print(F("Active control is: "));
	    active_control_vec->print();
	}
	void Whisker_controller::show_bs() {
	    Serial.print(F("Bumper state is: "));
	    Serial.println(text(b_state));
	}
	void Whisker_controller::show_dc() {
	    Serial.print(F("bool dc contains: "));
	    Serial.println(text(dc_state));
	}
#endif

