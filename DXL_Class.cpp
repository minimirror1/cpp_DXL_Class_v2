#include "cpp_Motor_Class_v2/Motor.h"
#include "cpp_DXL_SDK_v2/src/DynamixelSDK.h"
#include "DXL_Protocol.h"
#include "DXL_Class.h"


DXL_motor::DXL_motor(uint8_t gID, uint8_t sID, MotorType motorType, Serial *serial) :
	Motor(gID, sID, motorType){
	// Set IDs and other parameters for DXL_motor
	f_assign = false;
	operatingStatus_ = Status_None;

	serial_= serial;
	protocol_version_ = PROTOCOL_VERSION2;

	portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME, serial_);
	packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version_);
}

/* Motor Class 상속 */
/* input 필수 기능 */
//init
void DXL_motor::setSettingData_op(uint8_t gID, uint8_t sID, uint32_t data_1, uint32_t data_2)
{
	if(id_check(gID, sID) == false)
		return;

	//if(operatingStatus_ == Status_SettingInfo){
		operatingStatus_ = Statis_SettingOk;

		dxl_setting_.homeCnt_ = data_1;

		int tempLimitPosi = setting_.angle/360 * DXL_MAX_POSI;
		dxl_setting_.rangeCnt_ = (setting_.dir == DXL_ROT_CW)? tempLimitPosi : -tempLimitPosi;

		f_assign = true;
	//}
}

//control
void DXL_motor::setPosition(uint16_t targetPosition)
{
	if(!f_assign) return;

	monitor_.mrs_current_posi = targetPosition;

	float ratio = (float)targetPosition/4095;
	monitor_.raw_command_posi = dxl_setting_.homeCnt_ + (dxl_setting_.rangeCnt_ * ratio);
	packetHandler_->write4ByteTxOnly(portHandler_, sID_, ADDR_PRO_GOAL_POSITION, monitor_.raw_command_posi);
}

void DXL_motor::setRawPosition(int32_t targetPosition){
	if(!f_assign) return;
	monitor_.raw_command_posi = targetPosition;
	packetHandler_->write4ByteTxOnly(portHandler_, sID_, ADDR_PRO_GOAL_POSITION, targetPosition);
}

/* output 필수 기능*/
uint16_t DXL_motor::getPosition() const
{
    return monitor_.raw_current_posi;
}
int32_t DXL_motor::getDefaultPosi() const
{
    int32_t cntCalc = (float)dxl_setting_.homeCnt_ + (dxl_setting_.rangeCnt_ * (float)setting_.initPosi/4095);
    return cntCalc;
}


/* 공통 funtion */
void DXL_motor::init()
{
	int dxl_comm_result = COMM_SUCCESS;
	uint8_t dxl_error = 0;

	operatingStatus_ = Status_Init;

	//led off
	packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_PRO_LED, 0, &dxl_error);
	osDelay(10);
	packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_PRO_LED, 0, &dxl_error);
	osDelay(10);
	//rs485 응답 lv - 2 : 모든 패킷에 응답
	packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_STATUS_RETURN_LV, 2, &dxl_error);
	//torque off
    for (int retryCount = 0; retryCount < 5; ++retryCount) {
        dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if (dxl_comm_result == COMM_SUCCESS) {
            break; // 성공하면 루프 탈출
        } else {
            osDelay(10); // 실패한 경우 재시도 전에 딜레이 추가 (필요에 따라 조정)
        }
    }

	//drive mode(10) - 0x04 : Time-based Profile
	packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_PRO_DRIVE_MODE, 4, &dxl_error);
	//overating mode(11) - 0x03 : Position control
	packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_OPERATING_MODE, 3, &dxl_error);
	//profile acc(108) - 250 : 가속 목표 시간 250ms
	packetHandler_->write4ByteTxRx(portHandler_, sID_, ADDR_PRO_ACCELE, 250, &dxl_error);
	//profile velocity(112) - 500 : 도달 목표 시간 500ms
	packetHandler_->write4ByteTxRx(portHandler_, sID_, ADDR_PRO_VELOCITY, 500, &dxl_error);
	//torque on
	dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
	if(dxl_comm_result != COMM_SUCCESS)
		operatingStatus_ = Status_InitError;

	osDelay(10);
	dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, sID_, ADDR_PRO_PRESENT_POSITION,(uint32_t*)&monitor_.raw_current_posi, &dxl_error);
	if(dxl_comm_result != COMM_SUCCESS)
		operatingStatus_ = Status_InitError;

	//status return lv (68) - 1 : 읽기 명령에만 응답
	packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_STATUS_RETURN_LV, 1, &dxl_error);

	if(operatingStatus_ == Status_Init)
		operatingStatus_ = Status_PreRun;

	packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_PRO_LED, 1, &dxl_error);


}










