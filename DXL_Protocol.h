/*
 * DXL_Protocol.h
 *
 *  Created on: Dec 13, 2023
 *      Author: minim
 */

#ifndef CPP_DXL_CLASS_V2_DXL_PROTOCOL_H_
#define CPP_DXL_CLASS_V2_DXL_PROTOCOL_H_


// Protocol version
#define PROTOCOL_VERSION1               1.0                 // See which protocol version is used in the Dynamixel
#define PROTOCOL_VERSION2               2.0
// Default setting
#define DXL2_ID                         2                   // Dynamixel#2 ID: 2
#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller

#define ADDR_PRO_DRIVE_MODE				10
#define ADDR_OPERATING_MODE				11
#define ADDR_STATUS_RETURN_LV			68
#define ADDR_PRO_ACCELE					108
#define ADDR_PRO_VELOCITY				112

#define ADDR_PRO_TORQUE_ENABLE          64
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque


#endif /* CPP_DXL_CLASS_V2_DXL_PROTOCOL_H_ */
