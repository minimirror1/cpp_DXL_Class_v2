/*
 * DXL_Class.h
 *
 *  Created on: Dec 13, 2023
 *      Author: minim
 */

#ifndef INC_DXL_Class_H_
#define INC_DXL_Class_H_

#include "cpp_Motor_Class_v2/Motor.h"
#include "cpp_DXL_SDK_v2/src/DynamixelSDK.h"
#include "DXL_Protocol.h"

class DXL_motor : public Motor {
public:
    DXL_motor(uint8_t gID, uint8_t sID, MotorType motorType) :
    	Motor(gID, sID, motorType){
        // Set IDs and other parameters for DXL_motor
    }

    /* Motor Class 상속 */
    /* input 필수 기능 */
    /* init */
    void setSettingInfo(uint8_t dir, uint16_t angle, uint16_t initPosi, uint16_t reducer_ratio);

    /* control */
    void setPosition(uint16_t targetPosition);

    /* output 필수 기능*/
    uint16_t getPosition() const;//const 상태변경x 읽기전용

    /* 공통 funtion */
    void init();
    void defaultPosi();

private:
    /* 속성 */
	bool f_assign;	//true : 할당됨, false : 할당안됨
	dynamixel::PortHandler *portHandler_;
	dynamixel::PacketHandler *packetHandler_;


};
#endif /* INC_DXL_Class_H_ */
