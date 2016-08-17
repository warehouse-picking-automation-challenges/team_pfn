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

#include <Wire.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
using std_srvs::Empty;
#define FSensor 											0x29			/* i2c address */
#define LedPin  											13			

#define SYSTEM_INTERRUPT_CLEAR				0x15
#define	SYSTEM_FRESH_OUT_OF_RESET			0x16
#define SYSRANGE_START								0x18
#define	SYSALS_START									0x38
#define	RESULT_RANGE_STATUS						0x4d
#define	RESULT_ALS_STATUS							0x4e
#define	RESULT_INTERRUPT_STATUS_GPIO	0x4f
#define	RESULT_ALS_VAL								0x50
#define	RESULT_RANGE_VAL							0x62
ros::NodeHandle nh;
std_msgs::Int16 int_msg;
ros::Publisher sensor("distance_right", &int_msg);

void on(const Empty::Request & req, Empty::Response & res)
{
	delay(3);
}

ros::ServiceServer<Empty::Request, Empty::Response> srv("d_r", &on);
void callback(const std_msgs::Empty& toggle_msg){
	byte r;
	int t = 10;
	int_msg.data = int(get_range());
	sensor.publish(&int_msg);
}

ros::Subscriber<std_msgs::Empty> sub("called_dist_r", callback);

void setup()
{
	Wire.begin();
	pinMode(LedPin, OUTPUT);
	nh.initNode();
	nh.advertise(sensor);
	nh.subscribe(sub);
	nh.advertiseService(srv);
}

void loop()
{
	byte r;
	int t = 10;
	
	delay(t);
	nh.spinOnce();
	delay(t);
}

byte get_range(void)
{
	byte range, s;
	byte rgst, alst;
	if(reg_get(SYSTEM_FRESH_OUT_OF_RESET))
		fsense_init();
	
	reg_put( SYSRANGE_START, 0x01);
	while(reg_get( RESULT_INTERRUPT_STATUS_GPIO) & _BV(2) == 0);

	rgst = (reg_get(RESULT_RANGE_STATUS) >> 4);
	if(rgst != 0)
		return 200 + rgst;

	alst = (reg_get(RESULT_ALS_STATUS) >> 4);
	if(alst != 0)
		return 220 + alst;

	return(reg_get(RESULT_RANGE_VAL));
}

void reg_put(int addr, byte data)
{
	Wire.beginTransmission(FSensor);
	Wire.write((byte)(addr >> 8));
	Wire.write((byte)addr);
	Wire.write(data);
	Wire.endTransmission();
}

byte reg_get(int addr)
{
	reg_addr(addr);
	int rlen = Wire.requestFrom(FSensor, 1);
	if(rlen > 0 && Wire.available())
		return Wire.read();
	return 0;
}

byte reg_addr(int addr)
{
	Wire.beginTransmission(FSensor);
	Wire.write((byte)(addr >> 8));
	Wire.write((byte)addr);
	return Wire.endTransmission();
}

void fsense_init(void)
{
	reg_put(SYSTEM_FRESH_OUT_OF_RESET, 0);
	reg_put(0x0207, 0x01);
	reg_put(0x0208, 0x01);
	reg_put(0x0096, 0x00);
	reg_put(0x0097, 0xfd);
	reg_put(0x00e3, 0x00);
	reg_put(0x00e4, 0x04);
	reg_put(0x00e5, 0x02);
	reg_put(0x00e6, 0x01);
	reg_put(0x00e7, 0x03);
	reg_put(0x00f5, 0x02);
	reg_put(0x00d9, 0x05);
	reg_put(0x00db, 0xce);
	reg_put(0x00dc, 0x03);
	reg_put(0x00dd, 0xf8);
	reg_put(0x009f, 0x00);
	reg_put(0x00a3, 0x3c);
	reg_put(0x00b7, 0x00);
	reg_put(0x00bb, 0x3c);
	reg_put(0x00b2, 0x09);
	reg_put(0x00ca, 0x09);
	reg_put(0x0198, 0x01);
	reg_put(0x01b0, 0x17);
	reg_put(0x01ad, 0x00);
	reg_put(0x00ff, 0x05);
	reg_put(0x0100, 0x05);
	reg_put(0x0199, 0x05);
	reg_put(0x01a6, 0x1b);
	reg_put(0x01ac, 0x3e);
	reg_put(0x01a7, 0x1f);
	reg_put(0x0030, 0x00);
	
	reg_put(0x0011, 0x10); 
	reg_put(0x010a, 0x30); 
	reg_put(0x003f, 0x46); 
	reg_put(0x0031, 0xFF); 
	reg_put(0x0040, 0x63); 
	reg_put(0x002e, 0x01); 
	reg_put(0x001b, 0x09); 
	reg_put(0x003e, 0x31);
	reg_put(0x0014, 0x24); 
}

/*** Local variables:		***/
/*** mode:c++				***/
/*** tab-width:2		***/
/*** End:					***/
