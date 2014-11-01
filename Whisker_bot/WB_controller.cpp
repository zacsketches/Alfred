#include <WB_controller.h>

//*******************************************************************
//*                         CONSTRUCTOR
//*******************************************************************
WB_controller::WB_controller()
    :Node("WB_controller"),
    lt_bump_sub(&lt_bumper_msg, &ch, lt_local_bumper_msg),
    rt_bump_sub(&rt_bumper_msg, &ch, rt_local_bumper_msg),
    scan_sub(&five_pt_scan_msg, &ch, local_scan_msg),
    pub(&cmd_velocity_msg, &ch, local_msg)
{    
    b_state = Bump_state::clear;
	sensor_state = 0x00;
    dc_state = Danger_close_state::clear;
}

//*******************************************************************
//*                         REQUIRED VIRTUAL FUNCTIONS
//*******************************************************************
void WB_controller::begin() {
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
    scan_data.push_back(Scan_pt(heading4, 0));
    scan_data.push_back(Scan_pt(heading2, 0));
    scan_data.push_back(Scan_pt(heading0, 0));
    scan_data.push_back(Scan_pt(heading1, 0));
    scan_data.push_back(Scan_pt(heading3, 0));
}

void WB_controller::run() {
  
        lt_bump_sub.update();
        rt_bump_sub.update();
        scan_sub.update();
        
	    update_range(scan_data, local_scan_msg.p4);
	    update_range(scan_data, local_scan_msg.p2);
	    update_range(scan_data, local_scan_msg.p0);
	    update_range(scan_data, local_scan_msg.p1);
	    update_range(scan_data, local_scan_msg.p3);

        b_state = calc_b_state();
        dc_state = calc_dc_state();
        sensor_state = calc_sensor_state();
        
        active_control_vec = get_base_vector();
        
		publish_message();
}



//*******************************************************************
//*                         HELPER FUNCTIONS
//*******************************************************************

#if INCLUDE_CONTROLLER_PRINT == 1 
void WB_controller::print() {
    char buf[75];

    //print config data to serial
    Serial.println(F("Whisker and Bumper Controller configuration:"));
    Serial.print(F("\tNode id is: "));
    Serial.println(id());
    Serial.print(F("\tPublishing to: "));
	pub.publishing_where();
	Serial.print(F("\tLocal messages are: \n\t\t"));
	lt_local_bumper_msg.print();
	Serial.print(F("\t\t"));
    rt_local_bumper_msg.print();
	Serial.print(F("\t\t"));
    local_scan_msg.print();
    Serial.print(F("\tLeft bumper subscribed to: "));
	lt_bump_sub.subscribed_where();
	Serial.print(F("\tRight bumper subscribed to: "));
	rt_bump_sub.subscribed_where();
    Serial.print(F("\tScan subscribed to: "));
	scan_sub.subscribed_where();
}
#endif
#if INCLUDE_DEBUG_SCAN == 1
    void WB_controller::show_scan_data(){
	    Serial.println(F("Vector<Scan_pt> scan_data contains:"));
	    for(int i = 0; i < scan_data.size(); ++i) {
	        Serial.print(F("\t"));
	        Serial.println(text(scan_data[i]));
	    }
    }
	void WB_controller::show_trips() {
	    Serial.println(F("Vector<Trip_pt> trips contains:"));
	    for(int i = 0; i < trips.size(); ++i) {
	        Serial.print(F("\t"));
	        Serial.println(text(trips[i]));
	    }    
	}
	void WB_controller::show_sensor_state() {
	    Serial.print(F("byte sensor_state contains:"));
	    char buf[10];
		sprintf(buf, "0x%02x", sensor_state);
	    Serial.println(buf);
	}
#endif
#if INCLUDE_DEBUG_OTHER == 1
	void WB_controller::show_active_control(){
	    Serial.print(F("Active control is: "));
	    active_control_vec->print();
	}
	void WB_controller::show_bs() {
	    Serial.print(F("Bumper state is: "));
	    Serial.println(text(b_state));
	}
	void WB_controller::show_dc() {
	    Serial.print(F("bool dc contains: "));
	    Serial.println(text(dc_state));
	}
#endif

Bump_state::bs WB_controller::calc_b_state() {
    Bump_state::bs res = Bump_state::clear;
    
    if (rt_local_bumper_msg.pressed) res = Bump_state::rt_bump;
    if (lt_local_bumper_msg.pressed) res = Bump_state::lt_bump;
    
    return res;
}

byte WB_controller::calc_sensor_state(){

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

void WB_controller::update_scan_vector(Vector<Scan_pt>& vec, Five_pt_scan_msg& msg) {
    update_range(vec, msg.p4);
    update_range(vec, msg.p2);
    update_range(vec, msg.p0);
    update_range(vec, msg.p1);
    update_range(vec, msg.p3);
}

void WB_controller::update_range(Vector<Scan_pt>& vec, const Scan_pt& pt) {
    for(int i = 0; i < vec.size(); ++i) {
        if(pt.heading() == vec[i].heading()){
            vec[i].set_range(pt.range());
        }
    }
}

Danger_close_state::dc WB_controller::calc_dc_state() {
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

const Cmd_velocity_msg* WB_controller::get_base_vector() {
    //constants to support bump sensor
	static unsigned long maneuver_finish_time;	//time servo ordered to move
	static bool right_pressed;
	static bool left_pressed;
	
	//Default to easy forward
    const Cmd_velocity_msg* res;
    res = &Control::forward_easy;
    
    /*
        TODO move bump reaction to its own function...same for danger close and
        whisker reactions.
    */
   	
	//************************************************************************
	//*                         BUMPS
	//************************************************************************
	//if the bumper is pressed then set a new maneuver finish time
    if(b_state != Bump_state::clear) {
		maneuver_finish_time = millis() + bump_maneuver_time(); 
        if(b_state == Bump_state::lt_bump) left_pressed = true;
        if(b_state == Bump_state::rt_bump) right_pressed = true;
	}
	//If time has elapsed for the bump maneuver then clear the press bits
	if (millis() > maneuver_finish_time){
		left_pressed = false;
		right_pressed = false;
	}
	//set the control vector for the two sides we could bump on
	if (millis() < maneuver_finish_time && left_pressed) {
		res = &Control::back_twist_cw;
		return res;
	}
	if (millis() < maneuver_finish_time && right_pressed) {
		res = &Control::back_twist_ccw;
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
            Serial.println(F("From WB_controller::set_control_vec, attempting to thread the needle"));
        #endif
        res = thread_the_needle();
        return res;         
    }  
    */  
    
    //forward quarter sensors tripped
    if ((b & 0x08)==0x08)           { // 00100 tripped then swerve right
        res = &Control::swerve_right;
        return res;
    }
    if ((b & 0x02)==0x02)			{ // 01000 tripped then swerve left
        res = &Control::swerve_left;
        return res;
    }

    //just a trip on the side sensors then ease right or left
    switch(b) {
        case 0x10: //left side trip... ease to the right
            res = &Control::ease_right;
            return res;
        case 0x01: //right side trip...ease to the left by slowing that side down
            res = &Control::ease_left;
            return res;
    }
    
	return res;
}

void WB_controller::publish_message() {
    //update the local copy of the Rover_plant_msg and use it
    //to update the Rover_plant_fb object

    local_msg.l_dir = active_control_vec->l_dir;
    local_msg.l_spd = active_control_vec->l_spd;
    local_msg.r_dir = active_control_vec->r_dir;
    local_msg.r_spd = active_control_vec->r_spd;

    pub.publish();
}


