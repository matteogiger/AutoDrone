/*----------------------
Copyright 2022 Matteo Giger

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing,
software distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and limitations under the License.
----------------------*/

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <common/mavlink.h>
#include "autopilot_interface.h"
#include "serial_port.h"
#include "lipkg.h"


int main();

void autonomous_flight(Autopilot_Interface &autopilot_interface, ldlidar::LiPkg* lidar);

void waypoint_mode(Autopilot_Interface &api, ldlidar::LiPkg* lidar, std::vector<mavlink_local_position_ned_t>* waypoints);
std::vector<mavlink_local_position_ned_t> exploration_mode(Autopilot_Interface &api, ldlidar::LiPkg* lidar, float destination_x, float destination_y);

Autopilot_Interface* autopilot_interface_quit;
Serial_Port* port_quit;
ldlidar::CmdInterfaceLinux* cmd_port_quit;
void signal_handler( int sig );



