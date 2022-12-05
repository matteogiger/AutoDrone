/*----------------------
Copyright 2022 Matteo Giger

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing,
software distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and limitations under the License.
----------------------*/

#include "main.h"



int main()
{
	//initialize ports
	Serial_Port* port = new Serial_Port();
	ldlidar::CmdInterfaceLinux* cmd_port = new ldlidar::CmdInterfaceLinux();

	//initialize interfaces
	ldlidar::LiPkg* lidar = new ldlidar::LiPkg();
	Autopilot_Interface autopilot_interface(port);

	//copy pointers for shutdown
	cmd_port_quit            = cmd_port;
	port_quit                = port;
	autopilot_interface_quit = &autopilot_interface;

	//handle interrupts
	signal(SIGINT, signal_handler);

	//start lidar communication
	std::string port_name = "/dev/ttyUSB0";
 	cmd_port->SetReadCallback(std::bind(&ldlidar::LiPkg::CommReadCallback, lidar, std::placeholders::_1, std::placeholders::_2));
	if (!cmd_port->Open(port_name)) 
	{
		std::cout << "Failed to open port to lidar!"<< std::endl;
	}

	//start pixhawk communication
	port->start();
	autopilot_interface.start();

	//autonomous flight program
	autonomous_flight(autopilot_interface, lidar);

	//stop pixhawk communication
	autopilot_interface.stop();
	port->stop();

	//stop lidar communication
	cmd_port->Close();


	//clean up
	delete port;
	port = nullptr;

  	delete lidar;
	lidar = nullptr;

	delete cmd_port;
	cmd_port = nullptr;

	return 0;
}


float distance_2d_squared(float x1, float y1, float x2, float y2)
{
	return ((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

void normalize_angle(float* ang)
{
    *ang = ( *ang - ( floor( *ang / 6.28318530718f ) * 6.28318530718f ) ) ;
}

float NextCollision(ldlidar::Points2D ldscan, float ang2fly)
{
    float low = 20;
    for (int i = 0; i < ldscan.size(); i++) {
		//float abs_ang_diff = abs(ldscan[i].angle - normal_ang2fly);
		float diff1 = (ldscan[i].angle - ang2fly) - ( floor( (ldscan[i].angle - ang2fly) / 6.28318530718f ) * 6.28318530718f );
		float diff2 = (ang2fly - ldscan[i].angle) - ( floor( (ang2fly - ldscan[i].angle) / 6.28318530718f ) * 6.28318530718f );
		float abs_ang_diff =  std::min(diff1, diff2);
		if ((abs_ang_diff < (3.141593 / 2)) && (sinf(abs_ang_diff) * ldscan[i].x < 1.5f) && (ldscan[i].x > 0.3))
		{
			if (ldscan[i].x < low)
                low = ldscan[i].x; 
		}
    }

	//distance to obstacle is returned on purpose
    return low;
}

float get_angle_to_destination(float x, float y)
{
	if (x < 0)
	{
		return (3.141592653f + atanf(y/x));
	}
	else if ( y < 0)
	{
		return (6.28318530718f + atanf(y/x));
	}
	else 
	{
		return atanf(y/x);
	}
}
 
int DecideAvoidanceDirection(ldlidar::Points2D ldscan, float desired_direction)
{	
	for (float deviation = 0.1f; deviation < 1.7f; deviation += 0.1f)
	{
		if (NextCollision(ldscan, desired_direction + deviation) > 6)
		{
			printf("[Decision] right avoid, deviation: %f\n", deviation);
			return 1;
		}
		else if (NextCollision(ldscan, desired_direction - deviation) > 6)
		{
			printf("[Decision] left avoid, deviation: %f\n", deviation);
			return 0;
		}
	}
	

	return 2;
}

bool avoidleft(Autopilot_Interface &api, ldlidar::LiPkg* lidar, float destx, float desty, std::vector<mavlink_local_position_ned_t>* waypoint_list)
{
    std::vector<mavlink_local_position_ned_t> temp;
    temp.push_back(api.current_messages.local_position_ned);
	int clear_meters = 6;
    
	ldlidar::Points2D ld;
	ldlidar::PointData closest; 

	int silent_lidar = 0;


    while (true) {
		mavlink_set_position_target_local_ned_t sp;
		closest.x = 13;
		ldlidar::Points2D close;

		if (lidar->IsFrameReady()) {
			ld = lidar->GetLaserScanData();
			for (int k = 0; k < ld.size(); k++)
			{
				if (ld[k].distance < 150)
				{
					ld.erase(ld.begin() + k);
					continue;
				}
				ld[k].x = float(ld[k].distance) / 1000;
				ld[k].angle /= 57.2957795f;
				ld[k].angle += api.current_messages.attitude.yaw;
				normalize_angle(&ld[k].angle);
				if (ld[k].x < 5)
				{
					close.push_back(ld[k]);
					if (closest.x > ld[k].x) 
						closest = ld[k]; 
				}
				
			
			}
			silent_lidar = 0;
		}
		else {
			silent_lidar++;
			if (silent_lidar > 2) {
				printf("[Left Avoid] Last lidar scan is n cycles old\n");
				return false;
			}
		}

		
		float d_x = 0;
		float d_y = 0;

		for (int m = 0; m < close.size(); m++)
		{
			float diff1 = (close[m].angle - closest.angle) - ( floor( (close[m].angle - closest.angle) / 6.28318530718f ) * 6.28318530718f );
			float diff2 = (closest.angle - close[m].angle) - ( floor( (closest.angle - close[m].angle) / 6.28318530718f ) * 6.28318530718f );
			if (std::min(diff1, diff2) > 1.5707963267f)
			{
				std::cout << "[Left Avoid] FDCV conflict" << std::endl;
				closest.x = 13;
				break;
			}
				
		}
	

		if (closest.x < 5 && 1 < closest.x) 
		{ 
			d_x = -cos(closest.angle) * (5 - closest.x); 
			d_y = -sin(closest.angle) * (5 - closest.x); 

			//std::cout << "correction: " + std::to_string(d_x) + " " + std::to_string(d_y) << std::endl;
		}


		float turnang = get_angle_to_destination(destx - api.current_messages.local_position_ned.x, desty - api.current_messages.local_position_ned.y);
		float pass = 10;

		for (float test_ang = turnang; test_ang > (turnang - 1.61f); test_ang -= 0.1f)
		{
			if (NextCollision(ld, test_ang) > clear_meters)
			{
				pass = test_ang;
				//std::cout << "L - " + std::to_string(pass) << std::endl;
				break;
			}
		}

		if (pass == 10)
		{
			set_velocity(0, 0, 0.0, sp);
			api.update_setpoint(sp);
			printf("[Left Avoid] Failed\n");
			std::cout << "[Left Avoid] using temporary waypoints to return" << std::endl;
			std::reverse(temp.begin(), temp.end());
            waypoint_mode(api, lidar, &temp);
            return false;
		}
		else if (pass == turnang)
		{
			set_velocity(cosf(turnang) + d_x, sinf(turnang) + d_y, 0.0, sp);
			api.update_setpoint(sp);
			std::cout << "L Avoid Complete - SUCCESSFUL" << std::endl;
			waypoint_list->insert(waypoint_list->end(), temp.begin(), temp.end());
			std::cout << "[Left Avoid] temporary waypoints added to actual waypoints" << std::endl;
			return true;
		}
		else
		{
			set_velocity(cosf(pass) + d_x, sinf(pass) + d_y, 0.0, sp);
			api.update_setpoint(sp);
			 if (distance_2d_squared(temp.back().x, temp.back().y, api.current_messages.local_position_ned.x, api.current_messages.local_position_ned.y) > 1) {
                temp.push_back(api.current_messages.local_position_ned);
                std::cout << "[Left Avoid] temporary waypoint added" << std::endl;
            }
		}
    }

    std::cout << "[Left Avoid] failed - ???" << std::endl;
    return false;
}

bool avoidright(Autopilot_Interface &api, ldlidar::LiPkg* lidar, float destx, float desty, std::vector<mavlink_local_position_ned_t>* waypoint_list)
{
    std::vector<mavlink_local_position_ned_t> temp;
    temp.push_back(api.current_messages.local_position_ned);
	int clear_meters = 6;
    
	ldlidar::Points2D ld;
	ldlidar::PointData closest; 

	int silent_lidar = 0;


    while (true) {
		mavlink_set_position_target_local_ned_t sp;
		ldlidar::Points2D close;
		closest.x = 13;

       	if (lidar->IsFrameReady()) {
			ld = lidar->GetLaserScanData();
			for (int k = 0; k < ld.size(); k++)
			{
				if (ld[k].distance < 150)
				{
					ld.erase(ld.begin() + k);
					continue;
				}
				ld[k].x = float(ld[k].distance) / 1000;
				ld[k].angle /= 57.2957795f;
				ld[k].angle += api.current_messages.attitude.yaw;
				normalize_angle(&ld[k].angle);
				if (5 > ld[k].x)
				{
					close.push_back(ld[k]);
					if (closest.x > ld[k].x)
						closest = ld[k];
				}
			}
			silent_lidar = 0;
		}
		else {
			silent_lidar++;
			if (silent_lidar > 2) {
				printf("[Right Avoid] Last lidar scan is n cycles old\n");
				return false;
			}
		}

		float d_x = 0;
		float d_y = 0;

		for (int m = 0; m < close.size(); m++)
		{
			float diff1 = (close[m].angle - closest.angle) - ( floor( (close[m].angle - closest.angle) / 6.28318530718f ) * 6.28318530718f );
			float diff2 = (closest.angle - close[m].angle) - ( floor( (closest.angle - close[m].angle) / 6.28318530718f ) * 6.28318530718f );
			if (std::min(diff1, diff2) > 1.5707963267f)
			{
				std::cout << "[Right Avoid] FDCV conflict" << std::endl;
				closest.x = 13;
				break;
			}
				
		}
	

		if (closest.x < 5 && 1 < closest.x) 
		{ 
			d_x = -cos(closest.angle) * (5 - closest.x); 
			d_y = -sin(closest.angle) * (5 - closest.x); 

			//std::cout << "correction: " + std::to_string(d_x) + " " + std::to_string(d_y) << std::endl;
		}
		
		float turnang = get_angle_to_destination(destx - api.current_messages.local_position_ned.x, desty - api.current_messages.local_position_ned.y);
		float pass = 10;

		for (float test_ang = turnang; test_ang < (turnang + 1.61f); test_ang += 0.1f)
		{
			if (NextCollision(ld, test_ang) > clear_meters)
			{
				pass = test_ang;
				//std::cout << "R - " + std::to_string(pass) << std::endl;
				break;
			}
		}

		if (pass == 10)
		{
			set_velocity(0, 0, 0.0, sp);
			api.update_setpoint(sp);
			printf("[Right Avoid] Failed\n");
            std::cout << "[Right Avoid] using temporary waypoints to return" << std::endl;
            std::reverse(temp.begin(), temp.end());
            waypoint_mode(api, lidar, &temp);
            std::cout << "[Right Avoid] temporary waypoints discarded" << std::endl;
            return false;
		}
		else if (pass == turnang)
		{
			set_velocity(cosf(turnang) + d_x, sinf(turnang) + d_y, 0.0, sp);
			api.update_setpoint(sp);
			std::cout << "[Right Avoid] - SUCCESSFUL" << std::endl;
            waypoint_list->insert(waypoint_list->end(), temp.begin(), temp.end());
            std::cout << "[Right Avoid] temporary waypoints added to actual waypoints" << std::endl;
			return true;
		}
		else
		{
			set_velocity(cosf(pass) + d_x, sinf(pass)+ d_y, 0.0, sp);
			api.update_setpoint(sp);
            if (distance_2d_squared(temp.back().x, temp.back().y, api.current_messages.local_position_ned.x, api.current_messages.local_position_ned.y) > 1) {
                temp.push_back(api.current_messages.local_position_ned);
                std::cout << "[Right Avoid] temporary waypoint added" << std::endl;
            }
		}
    
    }

    
    std::cout << "[Right Avoid] failed - ???" << std::endl;
    return false;
    

}

bool avoidaltitude(Autopilot_Interface &api, ldlidar::LiPkg* lidar, float destx, float desty)
{
	int clear_meters = 6;
    
	ldlidar::Points2D ld;
	ldlidar::PointData closest;
	 

	int silent_lidar = 0;
	float turnang = get_angle_to_destination(destx - api.current_messages.local_position_ned.x, desty - api.current_messages.local_position_ned.y);
	float max_altitude = api.current_messages.local_position_ned.z - 3;

    while (api.current_messages.local_position_ned.z > max_altitude) {
		mavlink_set_position_target_local_ned_t sp;
		ldlidar::Points2D close;
		closest.x = 13;

       if (lidar->IsFrameReady()) {
			ld = lidar->GetLaserScanData();
			for (int k = 0; k < ld.size(); k++)
			{
				if (ld[k].distance < 150)
				{
					ld.erase(ld.begin() + k);
					continue;
				}
				ld[k].x = float(ld[k].distance) / 1000;
				ld[k].angle /= 57.2957795f;
				ld[k].angle += api.current_messages.attitude.yaw;
				normalize_angle(&ld[k].angle);
				if (5 > ld[k].x)
				{
					close.push_back(ld[k]);
					if (closest.x > ld[k].x)
						closest = ld[k];
				}				
			}
			silent_lidar = 0;
		}
		else {
			silent_lidar++;
			if (silent_lidar > 2) {
				printf("[Alt Avoid] Last lidar scan is n cycles old\n");
				return false;
			}
		}

	
		float d_x = 0;
		float d_y = 0;

		for (int m = 0; m < close.size(); m++)
		{
			float diff1 = (close[m].angle - closest.angle) - ( floor( (close[m].angle - closest.angle) / 6.28318530718f ) * 6.28318530718f );
			float diff2 = (closest.angle - close[m].angle) - ( floor( (closest.angle - close[m].angle) / 6.28318530718f ) * 6.28318530718f );
			if (std::min(diff1, diff2) > 1.5707963267f)
			{
				std::cout << "[Altitude Avoid] FDCV conflict" << std::endl;
				closest.x = 13;
				break;
			}
				
		}
	

		if (closest.x < 5 && 1 < closest.x) 
		{ 
			d_x = -cos(closest.angle) * (5 - closest.x); 
			d_y = -sin(closest.angle) * (5 - closest.x); 

			//std::cout << "correction: " + std::to_string(d_x) + " " + std::to_string(d_y) << std::endl;
		}

        if (NextCollision(ld, turnang) > 8)
        {
			float safety_distance_to_ground = api.current_messages.local_position_ned.z - 2;

   			while (api.current_messages.local_position_ned.z > safety_distance_to_ground) {

				set_velocity(d_x, d_y, -0.4, sp);
				api.update_setpoint(sp);
				usleep(100000);
			}
            std::cout << "[Altitude Avoid] - SUCCESSFUL" << std::endl;
            return true;
        }

		set_velocity(0, 0, -0.4, sp);
		api.update_setpoint(sp);
		
    }

    
    std::cout << "[Altitude Avoid] failed - over 3 meters" << std::endl;
    return false;
    

  
}

std::vector<mavlink_local_position_ned_t> exploration_mode(Autopilot_Interface &api, ldlidar::LiPkg* lidar, float destination_x, float destination_y)
{
	std::cout << "[Exploration Mode] start" << std::endl;
	ldlidar::Points2D latest_scan;
	if (lidar->IsFrameReady()) 
		latest_scan = lidar->GetLaserScanData();
	
	int silent_lidar;

	float angle_to_destination = 0;

	std::vector<mavlink_local_position_ned_t> waypoints;

	waypoints.push_back(api.current_messages.local_position_ned);

	while (distance_2d_squared(api.current_messages.local_position_ned.x, api.current_messages.local_position_ned.y, destination_x, destination_y) > 0.25)
	{
		//measure loop duration
		auto start_time = std::chrono::steady_clock::now();

		if (api.current_messages.battery_status.battery_remaining != 0 && api.current_messages.battery_status.battery_remaining < 40)
			printf("[Battery] Warning!\n");

		//initialize mavlink_set_position_target_local_ned_t struct
		mavlink_set_position_target_local_ned_t sp;
		
		//check if lidar data is available, if not available report to detect lidar failure
		if (lidar->IsFrameReady()) {
			latest_scan = lidar->GetLaserScanData();
			for (int i = 0; i < latest_scan.size(); i++)
			{
				if (latest_scan[i].distance < 150)
				{
					latest_scan.erase(latest_scan.begin() + i);
					continue;
				}
				latest_scan[i].x = float(latest_scan[i].distance) / 1000;
				latest_scan[i].angle /= 57.2957795f;
				latest_scan[i].angle += api.current_messages.attitude.yaw;
				normalize_angle(&latest_scan[i].angle);
				
			}
			silent_lidar = 0;
		}
		else {
			silent_lidar++;
		}

		//calculate angle that should be flown to reach destination
		angle_to_destination = get_angle_to_destination(destination_x - api.current_messages.local_position_ned.x, destination_y - api.current_messages.local_position_ned.y);
		
		//if path to destination is clear, fly on
		if (NextCollision(latest_scan, angle_to_destination) > std::min(5.f, std::sqrt(distance_2d_squared(destination_x, destination_y, api.current_messages.local_position_ned.x, api.current_messages.local_position_ned.y))))
		{
			set_velocity(cosf(angle_to_destination), sinf(angle_to_destination), 0, sp);
			api.update_setpoint(sp);
			
		}
		else //path is blocked by obstacles
		{ 
			printf("[Exploration Mode] obstacle detected\n");
			set_velocity(0, 0, 0.0f, sp);
			api.update_setpoint(sp);
			waypoints.push_back(api.current_messages.local_position_ned);
			std::cout << "[Exploration Mode] waypoint added" << std::endl;

			int decision_avoid = DecideAvoidanceDirection(latest_scan, angle_to_destination);

			switch(decision_avoid)
			{
				case 0:
				case 1:
					if (decision_avoid == 0)
					{
						printf("[Exploration Mode] left avoid started!\n");
						if (avoidleft(api, lidar, destination_x, destination_y, &waypoints))
							break;
						else 
						{
							if (avoidright(api, lidar, destination_x, destination_y, &waypoints))
								break;
						}
					}
					else
					{
						printf("[Exploration Mode] right avoid started!\n");
						if (avoidright(api, lidar, destination_x, destination_y, &waypoints))
							break;
						else
						{
							if (avoidleft(api, lidar, destination_x, destination_y, &waypoints))
								break;
						}
					}
				case 2:
					printf("[Exploration Mode] altitude avoid started!\n");
					if (avoidaltitude(api, lidar, destination_x, destination_y))
						break;
				default:
					break;
			}

			waypoints.push_back(api.current_messages.local_position_ned);
            std::cout << "[Exploration Mode] waypoint added" << std::endl;

			//we reset start_time because the drone could be avoiding for way longer than typical loop duration
			start_time = std::chrono::steady_clock::now();
		}
    
    	usleep(1000*100);  // sleep 100ms  == 10Hz

		if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-start_time).count() > 500) {
			printf("[Exploration Mode] loop frequency below 2Hz\n");
     		break; //I'm aware that write thread assures we have 4Hz stream, but the plan is to remove that in future to increase safety (failsafe is triggered when algorithm is stuck)
		}

		if (silent_lidar > 2) {
			printf("[Exploration Mode] last received lidar scan is too old\n");
     		break;
		}
 	}

	waypoints.push_back(api.current_messages.local_position_ned);
	std::cout << "[Exploration Mode] waypoint added" << std::endl;

	std::cout << "[Exploration Mode] end" << std::endl;
	return waypoints;
}

void waypoint_mode(Autopilot_Interface &api, ldlidar::LiPkg* lidar, std::vector<mavlink_local_position_ned_t>* waypoints)
{
	std::cout << "[Waypoint Mode] start" << std::endl;

	for (int i = 0; i < waypoints->size(); i++) {

		//initialize mavlink_set_position_target_local_ned_t struct
		mavlink_set_position_target_local_ned_t sp;

		//altitude correction
        if (abs(api.current_messages.local_position_ned.z - waypoints->at(i).z) > 0.3) {
            while (api.current_messages.local_position_ned.z > waypoints->at(i).z) {

                set_velocity(0, 0, -0.4f, sp);
            }
            while (api.current_messages.local_position_ned.z < waypoints->at(i).z) {

                set_velocity(0, 0, 0.4f, sp);
            }
        }
        
        //fly to next waypoint
        std::vector<mavlink_local_position_ned_t> intermediate_waypoints = exploration_mode(api, lidar, waypoints->at(i).x, waypoints->at(i).y);

		//save path adjustment
        if (intermediate_waypoints.size() > 2)
        {
            waypoints->insert(std::next(waypoints->begin(), i), std::next(intermediate_waypoints.begin()), std::prev(intermediate_waypoints.end()));
            std::cout << "[Waypoint Mode] path adjusted" << std::endl;
            i += int(intermediate_waypoints.size()) - 2;
        }
		
        std::cout << "[Waypoint Mode] waypoint reached" << std::endl;
    }

	std::cout << "[Waypoint Mode] end" << std::endl;
}


void takeoff(Autopilot_Interface &api)
{
	mavlink_set_position_target_local_ned_t sp;
	for (int i = 0; i < 100; i++) { //TODO: check has ascended to 2m, (we do not want to check using coordinate fram as 0 is not necessarily ground level)

		set_velocity(0, 0, -0.2, sp);
		sp.type_mask |= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_TAKEOFF;
		api.update_setpoint(sp);
		usleep(1000*100);
	}
}

void land(Autopilot_Interface &api)
{
	mavlink_set_position_target_local_ned_t sp;
	for (int i = 0; i < 160; i++) { //TODO: check has landed, (we do not want to check using coordinate fram as 0 is not necessarily ground level)

		set_velocity(0, 0, 0.1, sp);
		sp.type_mask |= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_LAND;
		api.update_setpoint(sp);
		usleep(1000*200);
	}
}


void autonomous_flight(Autopilot_Interface &api, ldlidar::LiPkg* lidar)
{
	usleep(100000);
	api.request_message(105); // highres imu single

	usleep(10000);
	api.request_message(147); //battery single

	usleep(10000);
	api.request_message(30); //attitude single


	usleep(10000);
	api.request_messagestream(147, 15000000); // battery stream

	usleep(10000);
	api.request_messagestream(32, 100000); // pos stream

	usleep(10000);
	api.request_messagestream(235, 1000000); // high lat stream (for failsafe detection)

	usleep(20000);

	printf("Please check if the following data seems plausible");
	printf("    pos  (NED):  % f % f % f (m)\n", api.current_messages.local_position_ned.x, api.current_messages.local_position_ned.y, api.current_messages.local_position_ned.z );
	printf("    acc  (NED):  % f % f % f (m/s^2)\n", api.current_messages.highres_imu.xacc , api.current_messages.highres_imu.yacc , api.current_messages.highres_imu.zacc );
	printf("    att  (p/r/y):  % f % f % f \n\n", api.current_messages.attitude.pitch , api.current_messages.attitude.roll , api.current_messages.attitude.yaw);
	printf("    altitude:    %f (m) \n"     , api.current_messages.highres_imu.pressure_alt);
	printf("    temperature: %f C \n"       , api.current_messages.highres_imu.temperature );
	printf("    battery: %i % \n"       , api.current_messages.battery_status.battery_remaining);

	float target_x;
	float target_y;

	printf("target x coordinate: ");
	std::cin >> target_x;

	printf("\ntarget y coordinate: ");
	std::cin >> target_y;

	printf("\n");
	usleep(3000000);

	
	takeoff(api);

	std::vector<mavlink_local_position_ned_t> way_1 = exploration_mode(api, lidar, target_x, target_y);

  	std::cout << "Exploration mode has collected " << way_1.size() << " waypoints." << std::endl;

        std::cout << "reversing waypoint list..." << std::endl;    
        std::reverse(way_1.begin(), way_1.end());

        waypoint_mode(api, lidar, &way_1);

	land(api);


	return;
}

void signal_handler(int sig)
{

	try {
		autopilot_interface_quit->handle_quit(sig);
		port_quit->stop();
		cmd_port_quit->Close();
	}
	catch (int error){}

	exit(0);
}


