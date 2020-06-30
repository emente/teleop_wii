#include <stdio.h>                      /* for printf */
#include "wiiuse.h"                     /* for wiimote_t, classic_ctrl_t, etc */
#ifndef WIIUSE_WIN32
#include <unistd.h>                     
#endif
#include <math.h>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ros/console.h"

#define MAX_WIIMOTES 1

wiimote** wiimotes;

ros::Publisher pub;
  
float lastRoll; //left-right neg=left
float lastPitch; //up-down neg=up

float currRoll=0;
float currPitch=0;

const float deadbandCutoff = 0.1f;

float joystickLinearScaledDeadband(float value) {
    if( fabs(value) < deadbandCutoff) {
        return 0;
    } else {
        return ( value - (fabs(value)/value)*deadbandCutoff ) / (1.0 - deadbandCutoff) ;
    }
}

float normalized(float v) {
	if (v>1) v=1;
	if (v<-1) v=-1;
	return v;
}

void handle_event(struct wiimote_t* wm) {
	if (WIIUSE_USING_ACC(wm)) {
		if (IS_JUST_PRESSED(wm, WIIMOTE_BUTTON_B)) {
			lastPitch = wm->orient.pitch;
			lastRoll = wm->orient.roll;
		}

		if (not IS_PRESSED(wm, WIIMOTE_BUTTON_B) || IS_PRESSED(wm,WIIMOTE_BUTTON_A)) {
			currRoll=0;
			currPitch=0;
		} else {
			currPitch = joystickLinearScaledDeadband(normalized((-wm->orient.roll+lastRoll)/90));
			currRoll = joystickLinearScaledDeadband(normalized((-wm->orient.pitch+lastPitch)/50));
		} 
	}
}

/**
 *	@brief Callback that handles a read event.
 *
 *	@param wm		Pointer to a wiimote_t structure.
 *	@param data		Pointer to the filled data block.
 *	@param len		Length in bytes of the data block.
 *
 *	This function is called automatically by the wiiuse library when
 *	the wiimote has returned the full data requested by a previous
 *	call to wiiuse_read_data().
 *
 *	You can read data on the wiimote, such as Mii data, if
 *	you know the offset address and the length.
 *
 *	The \a data pointer was specified on the call to wiiuse_read_data().
 *	At the time of this function being called, it is not safe to deallocate
 *	this buffer.
 */
void handle_read(struct wiimote_t* wm, byte* data, unsigned short len) {
/*	int i = 0;

	printf("\n\n--- DATA READ [wiimote id %i] ---\n", wm->unid);
	printf("finished read of size %i\n", len);
	for (; i < len; ++i) {
		if (!(i % 16)) {
			printf("\n");
		}
		printf("%x ", data[i]);
	}
	printf("\n\n");
	*/
}


/**
 *	@brief Callback that handles a controller status event.
 *
 *	@param wm				Pointer to a wiimote_t structure.
 *	@param attachment		Is there an attachment? (1 for yes, 0 for no)
 *	@param speaker			Is the speaker enabled? (1 for yes, 0 for no)
 *	@param ir				Is the IR support enabled? (1 for yes, 0 for no)
 *	@param led				What LEDs are lit.
 *	@param battery_level	Battery level, between 0.0 (0%) and 1.0 (100%).
 *
 *	This occurs when either the controller status changed
 *	or the controller status was requested explicitly by
 *	wiiuse_status().
 *
 *	One reason the status can change is if the nunchuk was
 *	inserted or removed from the expansion port.
 */
void handle_ctrl_status(struct wiimote_t* wm) {
	ROS_INFO_STREAM("battery level: " << wm->battery_level);
/*
	printf("attachment:      %i\n", wm->exp.type);
	printf("speaker:         %i\n", WIIUSE_USING_SPEAKER(wm));
	printf("ir:              %i\n", WIIUSE_USING_IR(wm));
	printf("leds:            %i %i %i %i\n", WIIUSE_IS_LED_SET(wm, 1), WIIUSE_IS_LED_SET(wm, 2), WIIUSE_IS_LED_SET(wm, 3), WIIUSE_IS_LED_SET(wm, 4));
	printf("battery:         %f %%\n", wm->battery_level);
*/
}


/**
 *	@brief Callback that handles a disconnection event.
 *
 *	@param wm				Pointer to a wiimote_t structure.
 *
 *	This can happen if the POWER button is pressed, or
 *	if the connection is interrupted.
 */
void handle_disconnect(wiimote* wm) {
	ROS_INFO("wiimote disconnected");
}


void test(struct wiimote_t* wm, byte* data, unsigned short len) {
	printf("test: %i [%x %x %x %x]\n", len, data[0], data[1], data[2], data[3]);
}

short any_wiimote_connected(wiimote** wm, int wiimotes) {
	int i;
	if (!wm) {
		return 0;
	}

	for (i = 0; i < wiimotes; i++) {
		if (wm[i] && WIIMOTE_IS_CONNECTED(wm[i])) {
			return 1;
		}
	}

	return 0;
}

void timerCallback(const ros::TimerEvent& e) {
	geometry_msgs::Twist msg;
	msg.linear.x = currRoll;
	msg.angular.z = currPitch;
	pub.publish(msg);
}

int eventcount=0;
void timer2Callback(const ros::TimerEvent& e) {
	switch (++eventcount) {
	case 1:wiiuse_set_leds(wiimotes[0], WIIMOTE_LED_1);break;
	case 2:wiiuse_set_leds(wiimotes[0], WIIMOTE_LED_2);break;
	case 3:wiiuse_set_leds(wiimotes[0], WIIMOTE_LED_3);break;
	case 4:wiiuse_set_leds(wiimotes[0], WIIMOTE_LED_4);break;
	case 5:wiiuse_set_leds(wiimotes[0], WIIMOTE_LED_3);break;
	case 6:wiiuse_set_leds(wiimotes[0], WIIMOTE_LED_2);eventcount=0;break;
	}
}


/**
 *	@brief main()
 *
 *	Connect to up to two wiimotes and print any events
 *	that occur on either device.
 */
int main(int argc, char** argv) {
	int found, connected;
	uint batterycount=0;

	ros::init(argc, argv, "teleop_wii");
	ros::NodeHandle n;
	ros::Timer timer = n.createTimer(ros::Duration(0.05), timerCallback);
	ros::Timer timer2 = n.createTimer(ros::Duration(1), timer2Callback);
	pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	/*
	 *	Initialize an array of wiimote objects.
	 *
	 *	The parameter is the number of wiimotes I want to create.
	 */
	wiimotes =  wiiuse_init(MAX_WIIMOTES);

	while(ros::ok()) {

		/*
		*	Find wiimote devices
		*
		*	Now we need to find some wiimotes.
		*	Give the function the wiimote array we created, and tell it there
		*	are MAX_WIIMOTES wiimotes we are interested in.
		*
		*	Set the timeout to be 1 seconds.
		*
		*	This will return the number of actual wiimotes that are in discovery mode.
		*/
		found = wiiuse_find(wiimotes, MAX_WIIMOTES, 3);
		if (!found) {
			continue;
		}

		/*
		*	Connect to the wiimotes
		*
		*	Now that we found some wiimotes, connect to them.
		*	Give the function the wiimote array and the number
		*	of wiimote devices we found.
		*
		*	This will return the number of established connections to the found wiimotes.
		*/
		connected = wiiuse_connect(wiimotes, MAX_WIIMOTES);
		if (connected) {
			ROS_INFO("Connected to wiimote");
		} else {
			continue;
		}

		/*
		*	Now set the LEDs and rumble for a second so it's easy
		*	to tell which wiimotes are connected (just like the wii does).
		*/
		wiiuse_set_leds(wiimotes[0], WIIMOTE_LED_2 | WIIMOTE_LED_3);
		wiiuse_rumble(wiimotes[0], 1);

	#ifndef WIIUSE_WIN32
		usleep(200000);
	#else
		Sleep(100);
	#endif

		wiiuse_rumble(wiimotes[0], 0);
		wiiuse_set_ir(wiimotes[0], 0); // off
		wiiuse_set_motion_plus(wiimotes[0], 0); // off
		wiiuse_motion_sensing(wiimotes[0], 1); // on

		/*
		*	Maybe I'm interested in the battery power of the 0th
		*	wiimote.  This should be WIIMOTE_ID_1 but to be sure
		*	you can get the wiimote associated with WIIMOTE_ID_1
		*	using the wiiuse_get_by_id() function.
		*
		*	A status request will return other things too, like
		*	if any expansions are plugged into the wiimote or
		*	what LEDs are lit.
		*/
		wiiuse_status(wiimotes[0]);

		/*
		*	This is the main loop
		*
		*	wiiuse_poll() needs to be called with the wiimote array
		*	and the number of wiimote structures in that array
		*	(it doesn't matter if some of those wiimotes are not used
		*	or are not connected).
		*
		*	This function will set the event flag for each wiimote
		*	when the wiimote has things to report.
		*/
		while (any_wiimote_connected(wiimotes, MAX_WIIMOTES)) {
			if (not ros::ok()) 
				goto lbl_disconnect;

			if (wiiuse_poll(wiimotes, MAX_WIIMOTES)>0) {
				if (batterycount++>8000) {
					batterycount=0;
					wiiuse_status(wiimotes[0]);
				}


				/*
				*	This happens if something happened on any wiimote.
				*	So go through each one and check if anything happened.
				*/
				int i = 0;
				for (; i < MAX_WIIMOTES; ++i) {
					switch (wiimotes[i]->event) {
						case WIIUSE_EVENT:
							handle_event(wiimotes[i]);
							break;

						case WIIUSE_STATUS:
							handle_ctrl_status(wiimotes[i]);
							break;

						case WIIUSE_DISCONNECT:
						case WIIUSE_UNEXPECTED_DISCONNECT:
							handle_disconnect(wiimotes[i]);
							goto lbl_disconnect;
							break;

						case WIIUSE_READ_DATA:
							/*
							*	Data we requested to read was returned.
							*	Take a look at wiimotes[i]->read_req
							*	for the data.
							*/
							break;

						default:
							break;
					}
				}
			}

			ros::spinOnce();
		}

		lbl_disconnect:
			geometry_msgs::Twist msg;
			msg.linear.x = 0;
			msg.angular.z = 0;
			pub.publish(msg);
			ros::spinOnce();
	}

	wiiuse_cleanup(wiimotes, MAX_WIIMOTES);
	return 0;
}
