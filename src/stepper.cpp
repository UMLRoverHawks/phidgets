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
#include "phidgets/stepper_params_better.h"

// handle
CPhidgetStepperHandle phid;

// stepper controller state publisher
ros::Publisher stepper_pub;

bool initialised = false;
long long current_position = 0;
double holding_current = 0;
double moving_current = 0;
double current = 0;

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
    current_position = Value; // Update global variable keeping track of position
    if (initialised) {
        phidgets::stepper_params_better m;
        m.position = Value;
        stepper_pub.publish(m);
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
    CPhidgetStepper_setCurrentLimit(phid,0,0.1);
    CPhidget_close((CPhidgetHandle)phid);
    CPhidget_delete((CPhidgetHandle)phid);
}

// Request to change stepper position
void stepperCallback(const phidgets::stepper_params_better::ConstPtr& ptr)
{
    if (initialised) {
        phidgets::stepper_params_better s = *ptr;

        /*
        // Lower the current limit
        if( s.lower_current && (current != holding_current) ) {
            CPhidgetStepper_setCurrentLimit(phid,0,(float)holding_current);
            current = holding_current;
        }
        */

        // Request current position
        if ( s.request_position ) {
            // Request position from motor. Should be stored in current_position.
            CPhidgetStepper_getCurrentPosition(phid,0,&current_position);

            // Publish a message to report current position.
            phidgets::stepper_params_better m;
            m.position = current_position;
            m.spin_motor = false;
            m.request_position = true;
            stepper_pub.publish(m);
            ROS_INFO("Position published as requested.");
            return; // We don't want to actually move the motors, so stop here.
        }

        // Spin the motor
        if (s.spin_motor) {
            CPhidgetStepper_setEngaged(phid, 0, 1);
            CPhidgetStepper_setAcceleration (phid, 0, (double)s.acceleration);
            CPhidgetStepper_setVelocityLimit (phid, 0, (double)s.velocity);
            CPhidgetStepper_setTargetPosition (phid, 0, s.position);
            CPhidgetStepper_setEngaged(phid, 0, 1); // Not sure if this needs to go before or after.
        }
        else {
            CPhidgetStepper_setEngaged(phid, 0, 0);
        }
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
        // Publishes to /phidgets/stepper/serial_number
        stepper_pub =
			n.advertise<phidgets::stepper_params_better>(topic_name,
												  buffer_length);

        // start service which can be used to set motor position
        // Subscribes to /stepper/serial_number
        ros::Subscriber sub = n.subscribe(service_name, 1, stepperCallback);

        initialised = true;
        ros::Rate loop_rate(frequency);

        CPhidgetStepper_setCurrentLimit(phid,0,(float)moving_current);

        while (ros::ok()) {
            ros::spinOnce();
            loop_rate.sleep();
        }

        disconnect(phid);
    }
    return 0;
}

