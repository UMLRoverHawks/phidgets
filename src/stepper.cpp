/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  ROS driver for Phidgets stepper motor controller
 *  Copyright (c) 2010, Bob Mottram
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

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <phidget21.h>
#include <std_msgs/String.h>
#include "phidgets/stepper_params.h"

// handle
CPhidgetStepperHandle phid;

// stepper controller state publisher
ros::Publisher stepper_pub;

bool initialised = false;
long long current_position = 0;
double holding_current = 0;
double moving_current = 0;
double current = 0,_max_current,_min_current;
double accel,_min_accel,_max_accel;
double vel,_min_vel,_max_vel;
int eng;
int stopped;

void UpdateCurrent()
{
  CPhidgetStepper_getCurrentPosition(phid,0,&current_position);
  CPhidgetStepper_getStopped(phid, 0, &stopped);
  CPhidgetStepper_getEngaged(phid, 0, &eng);
  CPhidgetStepper_getAcceleration(phid, 0, &accel);
  CPhidgetStepper_getVelocity(phid, 0, &vel);
  if (stopped && current != holding_current)
  {
    if (CPhidgetStepper_setCurrentLimit(phid,0,holding_current) == EPHIDGET_OK)
      current = holding_current;
    else
      ROS_ERROR("Failed to set current to holding_current");
  }
  else if (current != moving_current)
  {
    if (CPhidgetStepper_setCurrentLimit(phid,0,moving_current) == EPHIDGET_OK)
      current = moving_current;
    else
      ROS_ERROR("Failed to set current to moving_current");
  }
}

bool boundCheck(double desired, double min, double max)
{
  return (desired < max && desired > min);
}

int AttachHandler(CPhidgetHandle phid, void *userptr)
{
    int serial_number;
    const char *name;

    CPhidget_getDeviceName (phid, &name);
    CPhidget_getSerialNumber(phid, &serial_number);
    ROS_INFO("%s Serial number %d attached!",
			 name, serial_number);

    return 0;
}

int DetachHandler(CPhidgetHandle phid, void *userptr)
{
    int serial_number;
    const char *name;

    CPhidget_getDeviceName (phid, &name);
    CPhidget_getSerialNumber(phid, &serial_number);
    ROS_INFO("%s Serial number %d detached!",
			 name, serial_number);

    return 0;
}

int ErrorHandler(CPhidgetHandle phid, void *userptr,
				 int ErrorCode, const char *Description)
{
    ROS_INFO("Error handled. %d - %s", ErrorCode, Description);
    return 0;
}

int PositionChangeHandler(CPhidgetStepperHandle stepper,
						  void *usrptr, int Index,
						  long long Value)
{
    if (initialised) {
        UpdateCurrent();
        if (current_position != Value)
        {
          ROS_ERROR("YOUR ASSUMPTIONS ARE FALSE. MUAHAHAHAHA!");
        }
        phidgets::stepper_params p;
        p.position = current_position;
        p.engage = eng;
        p.velocity = vel;
        p.acceleration = accel;
        stepper_pub.publish(p);
    }
    return 0;
}

int display_properties(CPhidgetStepperHandle phid)
{
    int serial_number, version, num_motors;
    const char* ptr;

    CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
    CPhidget_getSerialNumber((CPhidgetHandle)phid,
							 &serial_number);
    CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);

    CPhidgetStepper_getMotorCount (phid, &num_motors);

    ROS_INFO("%s", ptr);
    ROS_INFO("Serial Number: %d", serial_number);
    ROS_INFO("Version: %d", version);
    ROS_INFO("Number of motors %d", num_motors);

    return 0;
}

bool attach(CPhidgetStepperHandle &phid,
			int serial_number)
{
    //create the object
    CPhidgetStepper_create(&phid);

    // Set the handlers to be run when the device is plugged
	// in or opened from software, unplugged or closed from
	// software, or generates an error.
    CPhidget_set_OnAttach_Handler((CPhidgetHandle)phid,
								  AttachHandler, NULL);
    CPhidget_set_OnDetach_Handler((CPhidgetHandle)phid,
								  DetachHandler, NULL);
    CPhidget_set_OnError_Handler((CPhidgetHandle)phid,
								 ErrorHandler, NULL);

    // Registers a callback that will run when the motor
	// position is changed.
    // Requires the handle for the Phidget, the function
	// that will be called, and an arbitrary pointer that
	// will be supplied to the callback function (may be NULL).
    CPhidgetStepper_set_OnPositionChange_Handler(phid,
												 PositionChangeHandler,
												 NULL);

    //open the device for connections
    CPhidget_open((CPhidgetHandle)phid, serial_number);

    // get the program to wait for an stepper control
	//  device to be attached
    if (serial_number == -1) {
        ROS_INFO("Waiting for Stepper Motor Control " \
				 "Phidget to be attached....");
    }
    else {
        ROS_INFO("Waiting for Stepper Motor Control " \
				 "Phidget %d to be attached....",
				 serial_number);
    }
    int result;
    if ((result =
		 CPhidget_waitForAttachment((CPhidgetHandle)phid,
									10000))) {
        const char *err;
        CPhidget_getErrorDescription(result, &err);
		ROS_ERROR("Problem waiting for attachment: %s", err);
		return false;
    }
    else return true;
}

/*!
 * \brief disconnect the stepper motor controller
 */
void disconnect(
				CPhidgetStepperHandle &phid)
{
    ROS_INFO("Closing...");
    CPhidget_close((CPhidgetHandle)phid);
    CPhidget_delete((CPhidgetHandle)phid);
}

// Request to change stepper position
void stepperCallback(const phidgets::stepper_params::ConstPtr& s)
{
    if (initialised) {
        if (!s->engage)
          CPhidgetStepper_setEngaged(phid, 0, 0);
        if (boundCheck(s->acceleration, _min_accel, _min_accel)  && accel != s->acceleration)
        {
          accel = s->acceleration;
          CPhidgetStepper_setAcceleration (phid, 0, (double)s->acceleration);
        }
        else if (accel == s->acceleration)
          ROS_WARN("Invalid acceleration requested ( %d <=   %d   <= %d )", _min_accel,accel,_max_accel);
        if (boundCheck(s->velocity, _min_vel, _min_vel) && vel != s->velocity)
        {
          vel = s->velocity;
          CPhidgetStepper_setVelocityLimit (phid, 0, (double)s->velocity);
        }
        else if (vel == s->velocity)
          ROS_WARN("Invalid velocity requested ( %d <=   %d   <= %d )", _min_vel,vel,_max_vel);
        CPhidgetStepper_setTargetPosition (phid, 0, s->position);
        if (s->reset_position) {
          CPhidgetStepper_setCurrentPosition(phid, 0, 0);
        }
        if (s->engage)
          CPhidgetStepper_setEngaged(phid, 0, 1);
    }
}

int main(int argc, char* argv[])
{
    // Set up ROS stuff
    ros::init(argc, argv, "phidgets_stepper");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    // Get motor information
    int serial_number = -1;
    nh.getParam("serial", serial_number);
    std::string name = "stepper";
    nh.getParam("name", name);
    if (serial_number==-1) {
        nh.getParam("serial_number", serial_number);
    }
    std::string topic_path = "phidgets/";
    nh.getParam("topic_path", topic_path);

    // Loop rate
    int frequency = 30;
    nh.getParam("frequency", frequency);

    // Current limits.
    nh.getParam("holding_current",holding_current);
    nh.getParam("moving_current",moving_current);

    // Initialize the motor
    if (attach(phid, serial_number)) {
		display_properties(phid);

        const int buffer_length = 100;        
        std::string topic_name = topic_path + name;
        std::string service_name = name;
        if (serial_number > -1) {
            char ser[10];
            sprintf(ser,"%d", serial_number);
            topic_name += "/";
            topic_name += ser;
            service_name += "/";
            service_name += ser;
        }
        CPhidgetStepper_getAccelerationMin(phid, 0, &_max_accel);
        CPhidgetStepper_getAccelerationMin(phid, 0, &_min_accel);
        CPhidgetStepper_getVelocityMax(phid, 0, &_max_vel);
        CPhidgetStepper_getVelocityMin(phid, 0, &_min_vel);
        CPhidgetStepper_getCurrentMax(phid, 0, &_max_current);
        CPhidgetStepper_getCurrentMin(phid, 0, &_min_current);

        // Publishes to /phidgets/stepper/serial_number
        stepper_pub = n.advertise<phidgets::stepper_params>(topic_name, buffer_length, true);

        //publish all the things once, then let latch
        UpdateCurrent();
        phidgets::stepper_params p;
        p.position = current_position;
        p.engage = eng;
        p.velocity = vel;
        p.acceleration = accel;
        stepper_pub.publish(p);

        // start service which can be used to set motor position
        // Subscribes to /stepper/serial_number
        ros::Subscriber sub = n.subscribe(service_name, 1, stepperCallback);

        initialised = true;
        ros::Rate loop_rate(frequency);

        while (ros::ok()) {
            UpdateCurrent();
            ros::spinOnce();
            loop_rate.sleep();
        }

        disconnect(phid);
    }
    return 0;
}

