/*
Copyright 2016 Preferred Networks, Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

This file incorporates code from http://www.narimatsu.net/blog/?p=10208
*/

/*
 * ros subscriber
 *
 * * magnet *
 * 40:m_off 41:m_on
 *
 * * valve for vacuum cleaner *
 * 10:v_off 11:v_on
 *
 * * valve for air *
 * 20:a_off 21:a_on
 *
 * * valve for pump *
 * 30:p_off 31:p_on
 */

#include <ros.h>
#include <std_srvs/Empty.h>
#include<std_msgs/Int16.h>
#include<std_msgs/String.h>
using std_srvs::Empty;

ros::NodeHandle nh;
std_msgs::String str_msg;
ros::Publisher pub("/txt_me_and_die",&str_msg);
int pin[4] = {2,7,8,10};

void on(int node){
	while(digitalRead(node) == LOW){
		digitalWrite(node,HIGH);
		delay(5);
	}
}

void off(int node){
	int max_count = 0;
	while(digitalRead(node) == HIGH){
		digitalWrite(node,LOW);
		delay(5);
	}
}

void callback(const std_msgs::Int16& msg)
{
	switch(msg.data){
		/* vac cleaner*/
		case 10:
			off(pin[0]);
			break;
		case 11:
			on(pin[0]);
			break;
		/* air valve*/
		case 20:
			off(pin[1]);
			break;
		case 21:
			on(pin[1]);
			break;
		/* pump */
		case 30:
			off(pin[2]);
			break;
		case 31:
			on(pin[2]);
			break;
		/* magnet */
		case 40:
			off(pin[3]);
			break;
		case 41:
			on(pin[3]);
			break;
		default:
			nh.loginfo("Status code does not match");
	}
}

ros::Subscriber<std_msgs::Int16> s("relay_command", &callback);


void setup()
{
	for(int i = 0; i < 4; ++i){
		pinMode(pin[i], OUTPUT);
		digitalWrite(pin[i],LOW);
	}
	nh.initNode();
	nh.subscribe(s);
	nh.advertise(pub);
	pinMode(13,OUTPUT);
}

void loop()
{
	nh.spinOnce();
	delay(100);
}

