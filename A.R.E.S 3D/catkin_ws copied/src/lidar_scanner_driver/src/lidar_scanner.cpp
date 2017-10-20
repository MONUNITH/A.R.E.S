/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Renee Love
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Case Western Reserve University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <lidar_scanner_driver/lidar_scanner.h>
#include <ros/console.h>
#include <boost/asio.hpp>
#include <std_msgs/UInt16.h>

#include <iostream>
#include "sensor_msgs/LaserScan.h"
#include "ros/ros.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)


using namespace std;

namespace lidar_scanner_driver 
{
	LidarScanner::LidarScanner(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io)
	: port_(port), baud_rate_(baud_rate), shutting_down_(false), serial_(io, port_) 
	{
		serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
	}

	void LidarScanner::poll(sensor_msgs::LaserScan::Ptr scan) 
	{
		uint8_t num_points_ = 0;
		bool scan_ready = false;

		rpms=0;

		scan->angle_min = 0.0;
		scan->angle_max = 2.0*M_PI;
		scan->angle_increment = (2.0*M_PI/360.0);
		scan->range_min = 0.01;
		scan->range_max = 6.0;
		scan->ranges.resize(360);
		scan->intensities.resize(360);
		scan->time_increment = 0;
		scan->scan_time = 0;	

		int scan_position;
		int distance;
		double degreesPerSecond;
		int scan_time_ms;
		char dummy;

		ros::Time start_scan_time;
		ros::Time end_scan_time;
		double scan_duration;

		while (!shutting_down_ && !scan_ready) 
		{
			boost::asio::streambuf response[360];
			int i=0,j=0,k=0;
			
			//for(i=0;i<360;i++)
			//{
			//	response[i].consume(response[i].size() + 1);
			//}
			
			// char response[360][40];
			// std::string str[360];

			start_scan_time = ros::Time::now();
			for(i=0;i<360;i++)
			{
				boost::asio::read_until(serial_, response[i], "\r" );
				//cin >> response[i];
				// std::getline(cin, str[i],'\n');
				// cout<<str[i]<<"*";
			}
			end_scan_time = ros::Time::now();
			scan_duration = (end_scan_time - start_scan_time).toSec() * 1e-3;

			float angle_min = DEG2RAD(0.0f);
			float angle_max = DEG2RAD(359.0f);
			for(i=0;i<360;i++)
			{
				std::istream response_stream(&response[i]);
				response_stream >> scan_position;
				response_stream >> dummy;
				response_stream >> distance;
				//response_stream >> dummy;
				//response_stream >> degreesPerSecond;
				//response_stream >> dummy;
				//response_stream >> scan_time_ms;
				
				// std::vector<int> vect;
				// stringstream ss(str[i]);
				// while (ss >> i)
				// {
				// 	vect.push_back(i);

				// 	if (ss.peek() == ',')
				// 		ss.ignore();
				// }
				// for(int i=0; i<vect.size(); ++i)
				// 	cout << vect[i] << ' ';
			 //    // scan_position = vect[0];
			 //    // distance = vect[1];
			 //    // degreesPerSecond = vect[2];
			 //    // scan_time_ms = vect[3];

				scan->ranges[scan_position] = distance / 100.0; //centimeter to meter conversion
				//scan->intensities[scan_position] = distance;
				//scan->time_increment += degreesPerSecond/360; //seconds between scan poins
			}
			
			ROS_INFO( "Read Point: %f, %f, %f" , angle_min, angle_max, scan_duration );

			// if ( scan_time_ms > 25 )
			// {
			// 	ROS_WARN( "LIDAR-Lite sampling took %d milliseconds", scan_time_ms );
			// }
			
			scan->angle_min =  M_PI - angle_min;
			scan->angle_max =  M_PI - angle_max;
			scan->angle_increment = (scan->angle_max - scan->angle_min) / (double)(360.0);
			scan->scan_time = scan_duration;
			scan->time_increment = scan_duration / (double)(360.0);
			scan->range_min = 0.01;
			scan->range_max = 8.0;

			scan_ready = true;

			//rpms = degreesPerSecond * 60.0 / 360.0;
		} 
	}
};


/* --------------------------------------------------------------------------
-----------------------------------------------------------------------------
-----------------------------------------------------------------------------*/


// typedef struct _response_measurement_node_t {
//     uint8_t    sync_quality;      // syncbit:1;syncbit_inverse:1;quality:6;
//     uint16_t   angle_q6_checkbit; // check_bit:1;angle_q6:15;
//     uint16_t   distance_q2;
// } __attribute__((packed)) _response_measurement_node_t;

// namespace lidar_scanner_driver 
// {
// 	std::string serial_port;
// 	int serial_baudrate = 115200;
// 	std::string frame_id;
// 	bool inverted = false;
// 	bool angle_compensate = true;
//   	boost::asio::io_service io;


// 	LidarScanner::LidarScanner(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io)
// 	: port_(port), baud_rate_(baud_rate), shutting_down_(false), serial_(io, port_) 
// 	{
// 		serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
// 	}

// 	void getcycleData()
// 	{
// 		uint8_t num_points_ = 0;
// 		bool scan_ready = false;

// 		int scan_position;
// 		int distance;
// 		double degreesPerSecond;
// 		int scan_time_ms;
// 		char dummy;

// 		while (ros::ok() && !scan_ready) 
// 		{

// 			boost::asio::streambuf response;
// 			boost::asio::read_until(serial_port, response, "\r" );

// 			std::istream response_stream(&response);
// 			response_stream >> scan_position;
// 			response_stream >> dummy;
// 			response_stream >> distance;
// 			response_stream >> dummy;
// 			response_stream >> degreesPerSecond;
// 			response_stream >> dummy;
// 			response_stream >> scan_time_ms;
// 			if ( ++num_points_ >= 359)
// 			{
// 				scan_ready = true;
// 			}
// 		}
// 	}

// 	void publish_scan(_response_measurement_node_t *nodes, size_t node_count, ros::Time start, double scan_time, bool inverted, float angle_min, float angle_max, std::string frame_id)
// 	{
// 		static int scan_count = 0;
// 		sensor_msgs::LaserScan scan_msg;

// 		scan_msg.header.stamp = start;
// 		scan_msg.header.frame_id = frame_id;
// 		scan_count++;

// 		bool reversed = (angle_max > angle_min);
// 		if ( reversed ) {
// 			scan_msg.angle_min =  M_PI - angle_max;
// 			scan_msg.angle_max =  M_PI - angle_min;
// 		} else {
// 			scan_msg.angle_min =  M_PI - angle_min;
// 			scan_msg.angle_max =  M_PI - angle_max;
// 		}
// 		scan_msg.angle_increment =
// 		(scan_msg.angle_max - scan_msg.angle_min) / (double)(node_count-1);

// 		scan_msg.scan_time = scan_time;
// 		scan_msg.time_increment = scan_time / (double)(node_count-1);
// 		scan_msg.range_min = 0.15;
// 		scan_msg.range_max = 8.0;

// 		scan_msg.intensities.resize(node_count);
// 		scan_msg.ranges.resize(node_count);
// 		bool reverse_data = (!inverted && reversed) || (inverted && !reversed);
// 		if (!reverse_data) {
// 			for (size_t i = 0; i < node_count; i++) {
// 				float read_value = (float) nodes[i].distance_q2/4.0f/1000;
// 				if (read_value == 0.0)
// 					scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
// 				else
// 					scan_msg.ranges[i] = read_value;
// 				scan_msg.intensities[i] = (float) (nodes[i].sync_quality >> 2);
// 			}
// 		} else {
// 			for (size_t i = 0; i < node_count; i++) {
// 				float read_value = (float)nodes[i].distance_q2/4.0f/1000;
// 				if (read_value == 0.0)
// 					scan_msg.ranges[node_count-1-i] = std::numeric_limits<float>::infinity();
// 				else
// 					scan_msg.ranges[node_count-1-i] = read_value;
// 				scan_msg.intensities[node_count-1-i] = (float) (nodes[i].sync_quality >> 2);
// 			}
// 		}
// 	}

// 	void LidarScanner::poll(sensor_msgs::LaserScan::Ptr scan) 
// 	{

// 		std::string serial_port;
// 		int serial_baudrate = 115200;
// 		std::string frame_id;
// 		bool inverted = false;
// 		bool angle_compensate = true;

// 		ros::Time start_scan_time;
// 		ros::Time end_scan_time;
// 		double scan_duration;
// 		while(ros::ok) // edit condition
// 		{
// 			_response_measurement_node_t nodes[360*2];
// 			size_t count = _countof(nodes);

// 			start_scan_time = ros::Time::now();
// 			getcycleData();                  //write return value for error on not completing read
// 			end_scan_time = ros::Time::now();
// 			scan_duration = (end_scan_time - start_scan_time).toSec() * 1e-3;
// 			float angle_min = DEG2RAD(0.0f);
// 			float angle_max = DEG2RAD(359.0f);


// 			if (angle_compensate) {
// 				const int angle_compensate_nodes_count = 360;
// 				const int angle_compensate_multiple = 1;
// 				int angle_compensate_offset = 0;
// 				_response_measurement_node_t angle_compensate_nodes[angle_compensate_nodes_count];
// 				memset(angle_compensate_nodes, 0, angle_compensate_nodes_count*sizeof(_response_measurement_node_t));
// 				int i = 0, j = 0;
// 				for( ; i < count; i++ ) {
// 					if (nodes[i].distance_q2 != 0) {
// 						float angle = (float)((nodes[i].angle_q6_checkbit >> 1)/64.0f);
// 						int angle_value = (int)(angle * angle_compensate_multiple);
// 						if ((angle_value - angle_compensate_offset) < 0) angle_compensate_offset = angle_value;
// 						for (j = 0; j < angle_compensate_multiple; j++) {
// 							angle_compensate_nodes[angle_value-angle_compensate_offset+j] = nodes[i];
// 						}
// 					}
// 				}

// 				publish_scan(angle_compensate_nodes, angle_compensate_nodes_count,
// 					start_scan_time, scan_duration, inverted,
// 					angle_min, angle_max,
// 					frame_id);
// 			} else {
// 				int start_node = 0, end_node = 0;
// 				int i = 0;
//                     // find the first valid node and last valid node
// 				while (nodes[i++].distance_q2 == 0);
// 				start_node = i-1;
// 				i = count -1;
// 				while (nodes[i--].distance_q2 == 0);
// 				end_node = i+1;

// 				angle_min = DEG2RAD((float)(nodes[start_node].angle_q6_checkbit >> 1)/64.0f);
// 				angle_max = DEG2RAD((float)(nodes[end_node].angle_q6_checkbit >> 1)/64.0f);

// 				publish_scan(&nodes[start_node], end_node-start_node +1,
// 					start_scan_time, scan_duration, inverted,
// 					angle_min, angle_max,
// 					frame_id);
// 			}
// 		}
// 	}
// };