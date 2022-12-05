/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file autopilot_interface.cpp
 *
 * @brief Autopilot interface functions
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */

/*
* original: https://github.com/mavlink/c_uart_interface_example/blob/master/autopilot_interface.cpp
*	
* added request_message, request_messagestream, some additional message handling e.x high_latency2 
* removed offboard_control_mode enable/disable, set_position, set_acceleration, set_yaw, set_yaw_rate, arm/disarm
* changed set_velocity to accelerate smoothly
*/

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "autopilot_interface.h"


// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
uint64_t
get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}


// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Autopilot_Interface::
Autopilot_Interface(Serial_Port *port_)
{
	// initialize attributes
	write_count = 0;

	reading_status = 0;      // whether the read thread is running
	writing_status = 0;      // whether the write thread is running
	control_status = 0;      // whether the autopilot is in offboard control mode
	time_to_exit   = false;  // flag to signal thread exit

	read_tid  = 0; // read thread id
	write_tid = 0; // write thread id

	system_id    = 0; // system id
	autopilot_id = 0; // autopilot component id
	companion_id = 0; // companion computer component id

	current_messages.sysid  = system_id;
	current_messages.compid = autopilot_id;

	port = port_; // port management object

}

Autopilot_Interface::
~Autopilot_Interface()
{}


// ------------------------------------------------------------------------------
//   Update Setpoint
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
update_setpoint(mavlink_set_position_target_local_ned_t setpoint)
{
	std::lock_guard<std::mutex> lock(current_setpoint.mutex);
	current_setpoint.data = setpoint;
}


/*
 * Set target local ned velocity
 *
 * Modifies a mavlink_set_position_target_local_ned_t struct with target VX VY VZ
 * velocities in the Local NED frame, in meters per second.
 */

mavlink_local_position_ned_t cur_pos;

void
set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp)
{
	sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY;

	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;

	
    if (vx == 0 && vy == 0) {
        sp.vx = 0;
        sp.vy = 0;
        sp.vz = vz;
    }
	else
	{
		float length = sqrtf(vx*vx + vy*vy + vz*vz);

		if (length == 0)
			return;

		sp.vx = (((vx / length) - cur_pos.vx) * 0.5f + cur_pos.vx) * 0.6f;
		sp.vy = (((vy / length) - cur_pos.vy) * 0.5f + cur_pos.vy) * 0.6f;
		sp.vz  = vz / length * 0.6f;

		float length_new = sqrtf(sp.vx*sp.vx + sp.vy*sp.vy + sp.vz*sp.vz);

		if (length_new > 1.1f)
            return;

		if (!std::isfinite(sp.vx) || !std::isfinite(sp.vy) || !std::isfinite(sp.vz))
		{
			sp.vx = 0;
			sp.vy = 0;
			sp.vz = 0;
			printf("Infinite Velocities = [ %.4f , %.4f , %.4f ] \n", vx, vy, vz);
		}
	}




	if (sp.vz > 0.2)
	{
		printf("Z VEL INVALID = [ %.4f ] \n", sp.vz);
		sp.vz = 0;
	}

	if (abs(sp.vx) > 1 || abs(sp.vy) > 1 || abs(sp.vz) > 2)
	{
		printf("TOO HIGH VELOCITY = [ %.4f , %.4f , %.4f ] \n", vx, vy, vz);
		sp.vx = 0;
		sp.vy = 0;
		sp.vz = 0;
		return;
	}

	//printf("VELOCITY SETPOINT UVW = [ %.4f , %.4f , %.4f ] \n", sp.vx, sp.vy, sp.vz);

}



// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_messages()
{
	bool success;               // receive success flag
	bool received_all = false;  // receive only one message
	Time_Stamps this_timestamps;

	// Blocking wait for new data
	while ( !received_all and !time_to_exit )
	{
		// ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------
		mavlink_message_t message;
		success = port->read_message(message);

		// ----------------------------------------------------------------------
		//   HANDLE MESSAGE
		// ----------------------------------------------------------------------
		if( success )
		{

			// Store message sysid and compid.
			// Note this doesn't handle multiple message sources.
			current_messages.sysid  = message.sysid;
			current_messages.compid = message.compid;

			// Handle Message ID
			switch (message.msgid)
			{

				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					//printf("MAVLINK_MSG_ID_HEARTBEAT\n");
					mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
					current_messages.time_stamps.heartbeat = get_time_usec();
					this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
					//printf("Flight Mode: %i\n",current_messages.heartbeat.custom_mode);
					break;
				}

				case MAVLINK_MSG_ID_SYS_STATUS:
				{
					//printf("MAVLINK_MSG_ID_SYS_STATUS\n");
					mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
					current_messages.time_stamps.sys_status = get_time_usec();
					this_timestamps.sys_status = current_messages.time_stamps.sys_status;
					//printf("Battery Voltage: %i\n",current_messages.sys_status.voltage_battery);
					//printf("Battery Remaining: %i\n",current_messages.sys_status.battery_remaining);
					break;
				}

				case MAVLINK_MSG_ID_BATTERY_STATUS:
				{
					//printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
					mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
					current_messages.time_stamps.battery_status = get_time_usec();
					this_timestamps.battery_status = current_messages.time_stamps.battery_status;
					//printf("Battery Remaining2: %i\n",current_messages.battery_status.battery_remaining);
					break;
				}

				case MAVLINK_MSG_ID_RADIO_STATUS:
				{
					//printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
					mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
					current_messages.time_stamps.radio_status = get_time_usec();
					this_timestamps.radio_status = current_messages.time_stamps.radio_status;
					break;
				}

				case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
				{
					//printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED msg received:)\n");
					mavlink_msg_local_position_ned_decode(&message, &(current_messages.local_position_ned));
					current_messages.time_stamps.local_position_ned = get_time_usec();
					this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;
					cur_pos = current_messages.local_position_ned;
					//printf("    Velocity:  % f % f % f\n", current_messages.local_position_ned.vx , current_messages.local_position_ned.vy , current_messages.local_position_ned.vz );
					break;
				}

				case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
				{
					//printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT msg received\n");
					mavlink_msg_global_position_int_decode(&message, &(current_messages.global_position_int));
					current_messages.time_stamps.global_position_int = get_time_usec();
					this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED msg received\n");
					mavlink_msg_position_target_local_ned_decode(&message, &(current_messages.position_target_local_ned));
					current_messages.time_stamps.position_target_local_ned = get_time_usec();
					this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
					break;
				}

				case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
				{
					//printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT received\n");
					mavlink_msg_position_target_global_int_decode(&message, &(current_messages.position_target_global_int));
					current_messages.time_stamps.position_target_global_int = get_time_usec();
					this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
					break;
				}

				case MAVLINK_MSG_ID_HIGHRES_IMU:
				{
					//printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
					mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
					current_messages.time_stamps.highres_imu = get_time_usec();
					this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
					break;
				}

				case MAVLINK_MSG_ID_ATTITUDE:
				{
					//printf("MAVLINK_MSG_ID_ATTITUDE\n");
					mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
					current_messages.time_stamps.attitude = get_time_usec();
					this_timestamps.attitude = current_messages.time_stamps.attitude;
					//printf("Attitude:  % f % f\n", current_messages.attitude.pitch , current_messages.attitude.roll);
					break;
				}

				case MAVLINK_MSG_ID_HIGH_LATENCY2:
				{
					mavlink_msg_high_latency2_decode(&message, &(current_messages.high_latency2));
					current_messages.time_stamps.high_latency2 = get_time_usec();
					this_timestamps.high_latency2 = current_messages.time_stamps.high_latency2;
					if (current_messages.high_latency2.failure_flags != 0)
						printf("%i\n", current_messages.high_latency2.failure_flags);
				}

				default:
				{
					// printf("Warning, did not handle message id %i\n",message.msgid);
					break;
				}


			} // end: switch msgid

		} // end: if read message

		// Check for receipt of all items
		received_all =
				this_timestamps.heartbeat                  &&
//				this_timestamps.battery_status             &&
//				this_timestamps.radio_status               &&
//				this_timestamps.local_position_ned         &&
//				this_timestamps.global_position_int        &&
//				this_timestamps.position_target_local_ned  &&
//				this_timestamps.position_target_global_int &&
//				this_timestamps.highres_imu                &&
//				this_timestamps.attitude                   &&
				this_timestamps.sys_status
				;

		// give the write thread time to use the port
		if ( writing_status > false ) {
			usleep(100); // look for components of batches at 10kHz
		}

	} // end: while not received all

	return;
}

// ------------------------------------------------------------------------------
//   Request Message
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
request_message(int message_id)
{
	// Prepare command for off-board mode
	mavlink_command_long_t com = { 0 };
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = 512; //request message command
	com.confirmation     = true;
	com.param1           = message_id; 
	com.param2           = 0;
	com.param7           = 0;

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = port->write_message(message);
	

	// Done!
	return len;
	
}

// ------------------------------------------------------------------------------
//   Request Message Stream
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
request_messagestream(int message_id, int interval)
{
	// Prepare command for off-board mode
	mavlink_command_long_t com = { 0 };
	com.target_system    = system_id;
	com.target_component = autopilot_id;
	com.command          = 511; //request message command
	com.confirmation     = true;
	com.param1           = message_id; 
	com.param2           = interval;
	com.param7           = 1;

	// Encode
	mavlink_message_t message;
	mavlink_msg_command_long_encode(system_id, companion_id, &message, &com);

	// Send the message
	int len = port->write_message(message);
	

	// Done!
	return len;
	
}

// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
write_message(mavlink_message_t message)
{
	// do the write
	int len = port->write_message(message);

	// book keep
	write_count++;

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   Write Setpoint Message
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_setpoint()
{
	// --------------------------------------------------------------------------
	//   PACK PAYLOAD
	// --------------------------------------------------------------------------

	// pull from position target
	mavlink_set_position_target_local_ned_t sp;
	{
		std::lock_guard<std::mutex> lock(current_setpoint.mutex);
		sp = current_setpoint.data;
	}

	// double check some system parameters
	if ( not sp.time_boot_ms )
		sp.time_boot_ms = (uint32_t) (get_time_usec()/1000);
	sp.target_system    = system_id;
	sp.target_component = autopilot_id;


	// --------------------------------------------------------------------------
	//   ENCODE
	// --------------------------------------------------------------------------

	mavlink_message_t message;
	mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id, &message, &sp);


	// --------------------------------------------------------------------------
	//   WRITE
	// --------------------------------------------------------------------------

	// do the write
	int len = write_message(message);

	// check the write
	if ( len <= 0 )
		fprintf(stderr,"WARNING: could not send POSITION_TARGET_LOCAL_NED \n");
	//	else
	//		printf("%lu POSITION_TARGET  = [ %f , %f , %f ] \n", write_count, position_target.x, position_target.y, position_target.z);

	return;
}


// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start()
{
	int result;

	// --------------------------------------------------------------------------
	//   CHECK PORT
	// --------------------------------------------------------------------------

	if ( !port->is_running() ) // PORT_OPEN
	{
		fprintf(stderr,"ERROR: port not open\n");
		throw 1;
	}


	// --------------------------------------------------------------------------
	//   READ THREAD
	// --------------------------------------------------------------------------

	printf("START READ THREAD \n");

	result = pthread_create( &read_tid, NULL, &start_autopilot_interface_read_thread, this );
	if ( result ) throw result;

	// now we're reading messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   CHECK FOR MESSAGES
	// --------------------------------------------------------------------------

	printf("CHECK FOR MESSAGES\n");

	while ( not current_messages.sysid )
	{
		if ( time_to_exit )
			return;
		usleep(500000); // check at 2Hz
	}

	printf("Found\n");

	// now we know autopilot is sending messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   GET SYSTEM and COMPONENT IDs
	// --------------------------------------------------------------------------

	// This comes from the heartbeat, which in theory should only come from
	// the autopilot we're directly connected to it.  If there is more than one
	// vehicle then we can't expect to discover id's like this.
	// In which case set the id's manually.

	// System ID
	if ( not system_id )
	{
		system_id = current_messages.sysid;
		printf("GOT VEHICLE SYSTEM ID: %i\n", system_id );
	}

	// Component ID
	if ( not autopilot_id )
	{
		autopilot_id = current_messages.compid;
		printf("GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
		printf("\n");
	}


	// --------------------------------------------------------------------------
	//   GET INITIAL POSITION
	// --------------------------------------------------------------------------

	// Wait for initial position ned
	while ( not ( current_messages.time_stamps.local_position_ned && //CAN'T RUN WITHOUT GPS SIGNAL position lock
				  current_messages.time_stamps.attitude            )  )
	{
		if ( time_to_exit )
			return;
		printf("Position or attitude message missing!\n");
		usleep(500000);
	}

	// copy initial position ned
	Mavlink_Messages local_data = current_messages;
	initial_position.x        = local_data.local_position_ned.x;
	initial_position.y        = local_data.local_position_ned.y;
	initial_position.z        = local_data.local_position_ned.z;
	initial_position.vx       = local_data.local_position_ned.vx;
	initial_position.vy       = local_data.local_position_ned.vy;
	initial_position.vz       = local_data.local_position_ned.vz;
	initial_position.yaw      = local_data.attitude.yaw;
	initial_position.yaw_rate = local_data.attitude.yawspeed;

	printf("INITIAL POSITION XYZ = [ %.4f , %.4f , %.4f ] \n", initial_position.x, initial_position.y, initial_position.z);
	printf("INITIAL POSITION YAW = %.4f \n", initial_position.yaw);
	printf("\n");

	// we need this before starting the write thread


	// --------------------------------------------------------------------------
	//   WRITE THREAD
	// --------------------------------------------------------------------------
	printf("START WRITE THREAD \n");

	result = pthread_create( &write_tid, NULL, &start_autopilot_interface_write_thread, this );
	if ( result ) throw result;

	// wait for it to be started
	while ( not writing_status )
		usleep(100000); // 10Hz

	// now we're streaming setpoint commands
	printf("\n");


	// Done!
	return;

}


// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
stop()
{
	// --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
	printf("CLOSE THREADS\n");

	// signal exit
	time_to_exit = true;

	// wait for exit
	pthread_join(read_tid ,NULL);
	pthread_join(write_tid,NULL);

	// now the read and write threads are closed
	printf("\n");

	// still need to close the port separately
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_read_thread()
{

	if ( reading_status != 0 )
	{
		fprintf(stderr,"read thread already running\n");
		return;
	}
	else
	{
		read_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_write_thread(void)
{
	if ( not writing_status == false )
	{
		fprintf(stderr,"write thread already running\n");
		return;
	}

	else
	{
		write_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
handle_quit( int sig )
{
	try {
		stop();

	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop autopilot interface\n");
	}

}



// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_thread()
{
	reading_status = true;

	while ( ! time_to_exit )
	{
		read_messages();
		usleep(100000); // Read batches at 10Hz
	}

	reading_status = false;

	return;
}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_thread(void)
{
	// signal startup
	writing_status = 2;

	// prepare an initial setpoint, just stay put
	mavlink_set_position_target_local_ned_t sp;
	sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY &
				   MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE;
	sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
	sp.vx       = 0.0;
	sp.vy       = 0.0;
	sp.vz       = 0.0;
	sp.yaw_rate = 0.0;

	// set position target
	{
		std::lock_guard<std::mutex> lock(current_setpoint.mutex);
		current_setpoint.data = sp;
	}

	// write a message and signal writing
	write_setpoint();
	writing_status = true;

	// Pixhawk needs to see off-board commands at minimum 2Hz,
	// otherwise it will go into fail safe
	while ( !time_to_exit )
	{
		usleep(250000);   // Stream at 4Hz
		write_setpoint();
	}

	// signal end
	writing_status = false;

	return;

}

// End Autopilot_Interface


// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void*
start_autopilot_interface_read_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_read_thread();

	// done!
	return NULL;
}

void*
start_autopilot_interface_write_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_write_thread();

	// done!
	return NULL;
}



