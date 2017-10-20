# /*********************************************************************
#  * Software License Agreement (BSD License)
#  *
#  *  Copyright (c) 2015, Renee Love
#  *  All rights reserved.
#  *
#  *  Redistribution and use in source and binary forms, with or without
#  *  modification, are permitted provided that the following conditions
#  *  are met:
#  *
#  *   * Redistributions of source code must retain the above copyright
#  *     notice, this list of conditions and the following disclaimer.
#  *   * Redistributions in binary form must reproduce the above
#  *     copyright notice, this list of conditions and the following
#  *     disclaimer in the documentation and/or other materials provided
#  *     with the distribution.
#  *   * Neither the name of Case Western Reserve University nor the names of its
#  *     contributors may be used to endorse or promote products derived
#  *     from this software without specific prior written permission.
#  *
#  *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  *  POSSIBILITY OF SUCH DAMAGE.
#  *********************************************************************/


# import lidar_scanner_driver/lidar_scanner.h
# import ros/console.h

def poll():
	rpms=0;

	scan = angle_min = 0.0;
	scan = angle_max = 2.0*M_PI;
	scan = angle_increment = (2.0*M_PI/360.0);
	scan = range_min = 0.01;
	scan = range_max = 6.0;
	scan = ranges.resize(360);
	scan = intensities.resize(360);

	scan_position = 0
	distance = 0
	degreesPerSecond = 0.0;
	scan_time_ms = 0
	dummy = ""


	while (!shutting_down_ && !scan_ready) :
		
