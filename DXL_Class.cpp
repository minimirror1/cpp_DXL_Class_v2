#include "cpp_Motor_Class_v2/Motor.h"
#include "cpp_DXL_SDK_v2/src/DynamixelSDK.h"
#include "DXL_Protocol.h"
#include "DXL_Class.h"

/* Motor Class 상속 */
/* input 필수 기능 */
//init
void DXL_motor::setSettingInfo(uint8_t dir, uint16_t angle, uint16_t initPosi, uint16_t reducer_ratio)
{

}

//control
void DXL_motor::setPosition(uint16_t targetPosition)
{

}


/* output 필수 기능*/
uint16_t DXL_motor::getPosition() const
{
    return 0;
}


/* 공통 funtion */
void DXL_motor::init()
{
	if(!f_assign) return;

	int dxl_comm_result = COMM_SUCCESS;
	uint8_t dxl_error = 0;

	operatingStatus_ = Status_Init;

	//torque off
	dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
	if(dxl_comm_result != COMM_SUCCESS)
		operatingStatus_ = Status_InitError;

	//rs485 응답 lv - 2 : 모든 패킷에 응답
	packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_STATUS_RETURN_LV, 2, &dxl_error);
	//drive mode(10) - 0x04 : Time-based Profile
	packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_PRO_DRIVE_MODE, 4, &dxl_error);
	//overating mode(11) - 0x03 : Position control
	packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_OPERATING_MODE, 3, &dxl_error);
	//profile acc(108) - 250 : 가속 목표 시간 250ms
	packetHandler_->write4ByteTxRx(portHandler_, sID_, ADDR_PRO_ACCELE, 250, &dxl_error);
	//profile velocity(112) - 500 : 도달 목표 시간 500ms
	packetHandler_->write4ByteTxRx(portHandler_, sID_, ADDR_PRO_VELOCITY, 500, &dxl_error);
	//status return lv (68) - 1 : 읽기 명령에만 응답
	packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_STATUS_RETURN_LV, 1, &dxl_error);
	//torque on
	dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, sID_,	ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
	if(dxl_comm_result != COMM_SUCCESS)
			operatingStatus_ = Status_InitError;

	osDelay(10);
	dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, sID_, ADDR_PRO_PRESENT_POSITION,(uint32_t*)&monitor_.raw_current_posi, &dxl_error);
	if(dxl_comm_result != COMM_SUCCESS)
			operatingStatus_ = Status_InitError;

	if(operatingStatus_ == Status_Init)
		operatingStatus_ = Status_PreRun;


}
void DXL_motor::defaultPosi()
{

}
