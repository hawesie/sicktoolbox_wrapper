///////////////////////////////////////////////////////////////////////////////
// this program just uses sicktoolbox to get laser scans, and then publishes
// them as ROS messages
//
// Copyright (C) 2008, Morgan Quigley
//
// I am distributing this code under the BSD license:
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, 
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in the 
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its 
//     contributors may be used to endorse or promote products derived from 
//     this software without specific prior written permission.
//   
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.

#include <csignal>
#include <cstdio>
#include <math.h>
#include <limits>
#include <sicktoolbox/SickPLS.hh>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <diagnostic_updater/diagnostic_updater.h> // Publishing over the diagnostics channels.
#include <diagnostic_updater/publisher.h>
using namespace SickToolbox;
using namespace std;

// Tick-tock transition variable, controls if the driver outputs NaNs and Infs
bool use_rep_117_;

void publish_scan(diagnostic_updater::DiagnosedPublisher<sensor_msgs::LaserScan> *pub, uint32_t *range_values,
                  uint32_t n_range_values, double scale, ros::Time start,
                  double scan_time, bool inverted, float angle_min,
                  float angle_max, std::string frame_id)
{
  static int scan_count = 0;
  sensor_msgs::LaserScan scan_msg;
  scan_msg.header.frame_id = frame_id;
  scan_count++;
  if (inverted) {
    scan_msg.angle_min = angle_max;
    scan_msg.angle_max = angle_min;
  } else {
    scan_msg.angle_min = angle_min;
    scan_msg.angle_max = angle_max;
  }
  scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)(n_range_values-1);
  scan_msg.scan_time = scan_time;
  scan_msg.time_increment = scan_time / (2*M_PI) * scan_msg.angle_increment;
  scan_msg.range_min = 0;
  if (scale == 0.01) {
    scan_msg.range_max = 81;
  }
  else if (scale == 0.001) {
    scan_msg.range_max = 8.1;
  }
  scan_msg.ranges.resize(n_range_values);
  scan_msg.header.stamp = start;

  if(use_rep_117_){ // Output NaNs and Infs where appropriate
    for (size_t i = 0; i < n_range_values; i++) {

      //PLS runs at 5m/50m... out of ranges seems to be 5105 5110
      if( scan_msg.ranges[i] > 5000) {
	scan_msg.ranges[i] = numeric_limits<float>::infinity();
      }
      else {
	scan_msg.ranges[i] = (float)range_values[i] * (float)scale;
      }
    }
  } else { // Use legacy output
    for (size_t i = 0; i < n_range_values; i++) {
      scan_msg.ranges[i] = (float)range_values[i] * (float)scale;
    }
  }

  pub->publish(scan_msg);
}

SickPLS::sick_pls_measuring_units_t StringToPLSMeasuringUnits(string units)
{
  if (units.compare("cm") == 0)
    return SickPLS::SICK_MEASURING_UNITS_CM;
  
  return SickPLS::SICK_MEASURING_UNITS_UNKNOWN;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "sickPLS");
  ros::NodeHandle nh;
  ros::NodeHandle nh_ns("~");
  string port;
  int baud;
  bool inverted;
  std::string frame_id;
  double scan_time = 0;
  //double angle_increment = 0.5;
  float angle_min = 0.0;
  float angle_max = 0.0;
	
  // Diagnostic publisher parameters
  double desired_freq;
  nh_ns.param<double>("desired_frequency", desired_freq, 75.0);
  double min_freq;
  nh_ns.param<double>("min_frequency", min_freq, desired_freq);
  double max_freq;
  nh_ns.param<double>("max_frequency", max_freq, desired_freq);
  double freq_tolerance; // Tolerance before error, fractional percent of frequency.
  nh_ns.param<double>("frequency_tolerance", freq_tolerance, 0.3);
  int window_size; // Number of samples to consider in frequency
  nh_ns.param<int>("window_size", window_size, 30);
  double min_delay; // The minimum publishing delay (in seconds) before error.  Negative values mean future dated messages.
  nh_ns.param<double>("min_acceptable_delay", min_delay, 0.0);
  double max_delay; // The maximum publishing delay (in seconds) before error.
  nh_ns.param<double>("max_acceptable_delay", max_delay, 0.2);
  std::string hardware_id;
  nh_ns.param<std::string>("hardware_id", hardware_id, "SICK PLS");
	
  // Set up diagnostics
  diagnostic_updater::Updater updater;
  updater.setHardwareID(hardware_id);
  diagnostic_updater::DiagnosedPublisher<sensor_msgs::LaserScan> scan_pub(nh.advertise<sensor_msgs::LaserScan>("scan", 10), updater, 
									  diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq, freq_tolerance, window_size),
									  diagnostic_updater::TimeStampStatusParam(min_delay, max_delay));
	
  nh_ns.param("port", port, string("/dev/sickpls"));
  nh_ns.param("baud", baud, 38400);
  nh_ns.param("inverted", inverted, false);
  nh_ns.param<std::string>("frame_id", frame_id, "laser");
	
  // Check whether or not to support REP 117
  std::string key;
  if (nh.searchParam("use_rep_117", key))
    {
      nh.getParam(key, use_rep_117_);
    } else {
    use_rep_117_ = false;
  }
	
  if(!use_rep_117_){ // Warn the user that they need to update their code.
    ROS_WARN("The use_rep_117 parameter has not been set or is set to false.  Please see: http://ros.org/wiki/rep_117/migration");
  }

  SickPLS::sick_pls_baud_t desired_baud = SickPLS::IntToSickBaud(baud);
  if (desired_baud == SickPLS::SICK_BAUD_UNKNOWN)
    {
      ROS_ERROR("Baud rate must be in {9600, 19200, 38400, 500000}");
      return 1;
    }

  uint32_t range_values[SickPLS::SICK_MAX_NUM_MEASUREMENTS] = {0};
  uint32_t n_range_values = 0;
  SickPLS sick_PLS(port);
  double scale = 0;
  //double angle_offset;

  // fixed properties of the laser
  //double angle = 180.0;

  try
    {
      sick_PLS.Initialize(desired_baud);

      SickPLS::sick_pls_measuring_units_t actual_units = sick_PLS.GetSickMeasuringUnits();

      if (actual_units == SickPLS::SICK_MEASURING_UNITS_CM) {
	scale = 0.01;
      }
      else {
	ROS_ERROR("Invalid measuring unit.");
	return 1;
      }

      // The scan time is always 1/75 because that's how long it takes
      // for the mirror to rotate. If we have a higher resolution, the
      // SICKs interleave the readings, so the net result is we just
      // shift the measurements.
      

      //SickPLS::sick_pls_scan_resolution_t scan_resolution = SickPLS::SICK_SCAN_RESOLUTION_50;
      

      //TODO work out what this should be for the PLS
      //if (angle == 180 || sick_PLS.IsSickLMSFast()) {
      scan_time = 1.0 / 75;
      
      //	else if ( scan_resolution == SickPLS::SICK_SCAN_RESOLUTION_50) {
      // 0.5 degrees
      //scan_time = 2.0 / 75;   // 26.66 ms
      //}
      
      // The increment for the slower PLS is still 1.0 even if its set to
      // 0.5 or 0.25 degrees resolution because it is interleaved. So for
      // 0.5 degrees, two scans are needed, offset by 0.5 degrees. These
      // show up as two seperate LaserScan messages.
      
      //angle_increment = 0.5;
      
      //angle_offset = (180.0-angle)/2;
    }
  catch (...)
    {
      ROS_ERROR("Initialize failed! are you using the correct device path?");
      return 2;
    }
  try
    {
      while (ros::ok()) {
	
	// There's no inteleaving
	sick_PLS.GetSickScan(range_values, n_range_values);
	angle_min = -M_PI/2;
	angle_max = M_PI/2;
	
	// Figure out the time that the scan started. Since we just
	// fished receiving the data, we'll assume that the mirror is at
	// 180 degrees now, or half a scan time. In other words, we
	// assume a zero transfer time of the data.
	ros::Time end_of_scan = ros::Time::now();
	ros::Time start = end_of_scan - ros::Duration(scan_time / 2.0);

	publish_scan(&scan_pub, range_values, n_range_values, scale, start, scan_time, inverted,
		     angle_min, angle_max, frame_id);
	ros::spinOnce();
	// Update diagnostics
	updater.update();
      }
    }
  catch (...)
    {
      ROS_ERROR("Unknown error.");
      return 1;
    }

  try
    {
      sick_PLS.Uninitialize();
    }
  catch (...)
    {
      ROS_ERROR("Error during uninitialize");
      return 1;
    }
  ROS_INFO("Success.\n");

  return 0;
}

