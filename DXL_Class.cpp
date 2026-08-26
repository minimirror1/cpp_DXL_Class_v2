#include "cpp_Motor_Class_v2/Motor.h"
#include "cpp_DXL_SDK_v2/src/DynamixelSDK.h"
#include "DXL_Protocol.h"
#include "DXL_Class.h"


DXL_motor::DXL_motor(uint8_t gID, uint8_t sID, MotorType motorType, Serial *serial) :
	Motor(gID, sID, motorType){
	// Set IDs and other parameters for DXL_motor
	f_assign = false;
	operatingStatus_ = Status_None;
	commFailureCount = 0;  // 통신 실패 카운터 초기화

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

		// float -> int 변환에서 소수점 이하가 버려지나(반올림 아님) 오차가 최대 1카운트라 무시함.
		// setting_.angle 은 float 이므로 여기까지 소수점 각도가 그대로 전달된다.
		int tempLimitPosi = setting_.angle/360 * DXL_MAX_POSI;
		dxl_setting_.rangeCnt_ = (setting_.dir == DXL_ROT_CW)? tempLimitPosi : -tempLimitPosi;

		f_assign = true;
	//}
}

//control
void DXL_motor::setPosition(int32_t targetPosition)
{
	if(!f_assign) return;

	// 상위 명령 스케일(0 ~ MRS_CMD_MAX) 밖의 값은 경계로 고정한다.
	// 수신 타입이 uint16_t 라 4095 를 넘는 값이 들어올 수 있으며, 그대로 두면
	// ratio 가 1.0 을 넘어 설정된 구동 범위를 벗어난 카운트가 모터로 나간다.
	// 여기서 막으면 raw 값은 [homeCnt_, homeCnt_ + rangeCnt_] 안에 머문다.
	if(targetPosition < 0)			targetPosition = 0;
	if(targetPosition > MRS_CMD_MAX)	targetPosition = MRS_CMD_MAX;

	monitor_.mrs_current_posi = targetPosition;

	float ratio = (float)targetPosition/MRS_CMD_MAX;
	int temp_raw_posi = dxl_setting_.homeCnt_ + (dxl_setting_.rangeCnt_ * ratio);

	if(monitor_.raw_command_posi != temp_raw_posi){
		monitor_.raw_command_posi = temp_raw_posi;
		packetHandler_->write4ByteTxOnly(portHandler_, sID_, ADDR_PRO_GOAL_POSITION, monitor_.raw_command_posi);
	}

#if 0
	if(monitor_.raw_command_posi != temp_raw_posi){
		if(com_limit.delay(50)){
			monitor_.raw_command_posi = temp_raw_posi;
			packetHandler_->write4ByteTxOnly(portHandler_, sID_, ADDR_PRO_GOAL_POSITION, monitor_.raw_command_posi);
		}
	}
#endif
}

void DXL_motor::timeCheckPosition(){
	if(!f_assign) return;

	if(com_limit.delay(50)){
		float ratio = (float)monitor_.mrs_current_posi/MRS_CMD_MAX;
		int temp_raw_posi = dxl_setting_.homeCnt_ + (dxl_setting_.rangeCnt_ * ratio);

		if(monitor_.raw_command_posi != temp_raw_posi){
			monitor_.raw_command_posi = temp_raw_posi;
			packetHandler_->write4ByteTxOnly(portHandler_, sID_, ADDR_PRO_GOAL_POSITION, monitor_.raw_command_posi);
		}
	}
}


void DXL_motor::setRawPosition(int32_t targetPosition){
	if(!f_assign) return;
	monitor_.raw_command_posi = targetPosition;
	packetHandler_->write4ByteTxOnly(portHandler_, sID_, ADDR_PRO_GOAL_POSITION, targetPosition);
}

/* TODO 조그 동작 재확인 필요 (미해결, 동작 확인 후 정리할 것)
 *
 * 1) 용도와 가드가 어긋남
 *    조그는 구동 범위를 입력받기 전에 임의로 움직여보는 용도인데,
 *    f_assign 은 setSettingData_op() 에서 homeCnt_/rangeCnt_ 를 설정할 때
 *    비로소 true 가 된다. 즉 MRS_RX_DATA_OP 수신 전에는 조그가 아무 동작도
 *    하지 않는다. 임시값으로 DATA1/DATA_OP 를 먼저 보내는 운용이라면
 *    현행 유지가 맞고, 아니라면 가드 조건을 손봐야 한다.
 *
 * 2) 범위 검사 기준이 raw 카운트 기준의 0~4095 임
 *    실제 구동 범위는 [homeCnt_, homeCnt_ + rangeCnt_] 이며 CCW 면
 *    rangeCnt_ 가 음수라 상/하한이 뒤바뀐다. homeCnt_ 가 0 이 아니거나
 *    CCW 인 경우 범위를 벗어난 조그가 허용되거나, 반대로 정상 구간인데도
 *    차단될 수 있다. 1) 의 운용 방식이 확정되어야 올바른 기준을 정할 수 있음.
 *
 * 3) mrs_current_posi 를 갱신하지 않음
 *    현재는 allTimeCheckPosi() 가 dxl_task.cpp 에서 비활성이라 무해하지만,
 *    되살리면 50ms 뒤 timeCheckPosition() 이 조그 이전 위치로 되돌린다.
 */
void DXL_motor::setJogMove(int jogCounter){
	if(!f_assign) return;

	if( (monitor_.raw_command_posi + jogCounter) < 0 )
		return;
	if( 4095 < (monitor_.raw_command_posi + jogCounter))
		return;

	monitor_.raw_command_posi += jogCounter;
	packetHandler_->write4ByteTxOnly(portHandler_, sID_, ADDR_PRO_GOAL_POSITION, monitor_.raw_command_posi);
}

/* output 필수 기능*/
uint16_t DXL_motor::getCurrentPosition() const
{
	uint8_t dxl_error = 0;
	int dxl_comm_result = COMM_SUCCESS;
    for (int retryCount = 0; retryCount < 5; ++retryCount) {
        dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, sID_, ADDR_PRO_PRESENT_POSITION,(uint32_t*)&monitor_.raw_current_posi, &dxl_error);
        if (dxl_comm_result == COMM_SUCCESS) {
            break; // 성공하면 루프 탈출
        } else {
            osDelay(10); // 실패한 경우 재시도 전에 딜레이 추가 (필요에 따라 조정)
        }
    }

    return monitor_.raw_current_posi;
}
int32_t DXL_motor::getDefaultPosi() const
{
    int32_t cntCalc = (float)dxl_setting_.homeCnt_ + (dxl_setting_.rangeCnt_ * (float)setting_.initPosi/MRS_CMD_MAX);
    return cntCalc;
}

uint8_t DXL_motor::getHardwareErrorStatus() const
{
    // 할당되지 않은 모터의 경우 bit7 = 1
    if(!f_assign) {
        return HW_ERROR_NOT_ASSIGNED;
    }
    
    uint8_t hardware_error_status = 0;
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_SUCCESS;
    
    dxl_comm_result = packetHandler_->read1ByteTxRx(
        portHandler_, 
        sID_, 
        ADDR_PRO_HARDWARE_ERROR_STATUS, 
        &hardware_error_status, 
        &dxl_error
    );

    if (dxl_comm_result == COMM_SUCCESS) {
        // 통신 성공 시 실패 카운터 리셋
        commFailureCount = 0;
        return hardware_error_status;
    }
    else {
        // 통신 실패 카운터 증가
        commFailureCount++;
        
        // 3회 연속 실패 시에만 통신 실패 에러 반환
        if(commFailureCount >= 3) {
            return HW_ERROR_COMM_FAILURE; // 통신 실패 시 bit6 = 1
        }
        else {
            // 아직 3회 미만이면 이전 상태 유지 (에러 없음으로 처리)
            return 0;
        }
    }
}


/* 공통 funtion */
void DXL_motor::init()
{
	int dxl_comm_result = COMM_SUCCESS;
	uint8_t dxl_error = 0;

	operatingStatus_ = Status_Init;


    
	/* [사용 금지] reboot 비활성화
	 *
	 * reboot(INST_REBOOT)은 Dynamixel의 래치된 Hardware Error Status(주소 70)를
	 * 클리어한다. init()이 호출될 때마다 에러 상태가 지워지므로
	 * getHardwareErrorStatus()가 하드웨어 에러를 영영 검출하지 못한다.
	 *
	 * 이 코드를 비활성화한 뒤 에러 감지가 정상 동작함을 실기에서 확인함.
	 * 아래 초기화 시퀀스가 필요한 레지스터를 모두 명시적으로 재설정하므로
	 * reboot 없이도 초기화는 정상 완료된다.
	 *
	 * 다시 활성화하면 하드웨어 에러 감지가 다시 동작하지 않는다. */
//	for (int retryCount = 0; retryCount < 5; ++retryCount) {
//        dxl_comm_result = packetHandler_->reboot(portHandler_, sID_, &dxl_error);
//		if (dxl_comm_result == COMM_SUCCESS) {
//            break; // 성공하면 루프 탈출
//        } else {
//            osDelay(10); // 실패한 경우 재시도 전에 딜레이 추가 (필요에 따라 조정)
//        }
//    }

	//led off
//	packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_PRO_LED, 0, &dxl_error);
//	osDelay(10);
//	packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_PRO_LED, 0, &dxl_error);
//	osDelay(10);
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
    for (int retryCount = 0; retryCount < 5; ++retryCount) {
        dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_PRO_DRIVE_MODE, 4, &dxl_error);
        if (dxl_comm_result == COMM_SUCCESS) {
            break; // 성공하면 루프 탈출
        } else {
            osDelay(10); // 실패한 경우 재시도 전에 딜레이 추가 (필요에 따라 조정)
        }
    }

	//overating mode(11) - 0x03 : Position control
    for (int retryCount = 0; retryCount < 5; ++retryCount) {
        dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_OPERATING_MODE, 3, &dxl_error);
        if (dxl_comm_result == COMM_SUCCESS) {
            break; // 성공하면 루프 탈출
        } else {
            osDelay(10); // 실패한 경우 재시도 전에 딜레이 추가 (필요에 따라 조정)
        }
    }

	//profile acc(108) - 25 : 가속 목표 시간 25ms
    for (int retryCount = 0; retryCount < 5; ++retryCount) {
		dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, sID_, ADDR_PRO_ACCELE, 25, &dxl_error);
		if (dxl_comm_result == COMM_SUCCESS) {
			break; // 성공하면 루프 탈출
		} else {
			osDelay(10); // 실패한 경우 재시도 전에 딜레이 추가 (필요에 따라 조정)
		}
	}

	//profile velocity(112) - 50 : 도달 목표 시간 50ms
    for (int retryCount = 0; retryCount < 5; ++retryCount) {
		dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, sID_, ADDR_PRO_VELOCITY, 50, &dxl_error);
		if (dxl_comm_result == COMM_SUCCESS) {
			break; // 성공하면 루프 탈출
		} else {
			osDelay(10); // 실패한 경우 재시도 전에 딜레이 추가 (필요에 따라 조정)
		}
	}

	//torque on
    for (int retryCount = 0; retryCount < 5; ++retryCount) {
    	dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
		if (dxl_comm_result == COMM_SUCCESS) {
			break; // 성공하면 루프 탈출
		} else {
			if(retryCount >= 4)
				operatingStatus_ = Status_InitError;
			osDelay(10); // 실패한 경우 재시도 전에 딜레이 추가 (필요에 따라 조정)
		}
	}

	osDelay(10);


	for (int retryCount = 0; retryCount < 5; ++retryCount) {
		dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, sID_, ADDR_PRO_PRESENT_POSITION,(uint32_t*)&monitor_.raw_current_posi, &dxl_error);
		if (dxl_comm_result == COMM_SUCCESS) {
			monitor_.raw_command_posi = monitor_.raw_current_posi;
			break; // 성공하면 루프 탈출
		} else {
			if(retryCount >= 4)
				operatingStatus_ = Status_InitError;
			osDelay(10); // 실패한 경우 재시도 전에 딜레이 추가 (필요에 따라 조정)
		}
	}

	//status return lv (68) - 1 : 읽기 명령에만 응답
	for (int retryCount = 0; retryCount < 5; ++retryCount) {
		dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_STATUS_RETURN_LV, 1, &dxl_error);
		if (dxl_comm_result == COMM_SUCCESS) {
			break; // 성공하면 루프 탈출
		} else {
			osDelay(10); // 실패한 경우 재시도 전에 딜레이 추가 (필요에 따라 조정)
		}
	}

	if(operatingStatus_ == Status_Init){
		operatingStatus_ = Status_PreRun;
		for (int retryCount = 0; retryCount < 5; ++retryCount) {
			dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_PRO_LED, 1, &dxl_error);
			if (dxl_comm_result == COMM_SUCCESS) {
				break; // 성공하면 루프 탈출
			} else {
				osDelay(10); // 실패한 경우 재시도 전에 딜레이 추가 (필요에 따라 조정)
			}
		}
	}
}

void DXL_motor::multiTurnInit(){
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
    for (int retryCount = 0; retryCount < 5; ++retryCount) {
        dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_PRO_DRIVE_MODE, 4, &dxl_error);
        if (dxl_comm_result == COMM_SUCCESS) {
            break; // 성공하면 루프 탈출
        } else {
            osDelay(10); // 실패한 경우 재시도 전에 딜레이 추가 (필요에 따라 조정)
        }
    }

	//overating mode(11) - 0x04 : Multi-turn
    for (int retryCount = 0; retryCount < 5; ++retryCount) {
        dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_OPERATING_MODE, 4, &dxl_error);
        if (dxl_comm_result == COMM_SUCCESS) {
            break; // 성공하면 루프 탈출
        } else {
            osDelay(10); // 실패한 경우 재시도 전에 딜레이 추가 (필요에 따라 조정)
        }
    }

	//profile acc(108) - 25 : 가속 목표 시간 25
    for (int retryCount = 0; retryCount < 5; ++retryCount) {
		dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, sID_, ADDR_PRO_ACCELE, 25, &dxl_error);
		if (dxl_comm_result == COMM_SUCCESS) {
			break; // 성공하면 루프 탈출
		} else {
			osDelay(10); // 실패한 경우 재시도 전에 딜레이 추가 (필요에 따라 조정)
		}
	}

	//profile velocity(112) - 50 : 도달 목표 시간 50
    for (int retryCount = 0; retryCount < 5; ++retryCount) {
		dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, sID_, ADDR_PRO_VELOCITY, 50, &dxl_error);
		if (dxl_comm_result == COMM_SUCCESS) {
			break; // 성공하면 루프 탈출
		} else {
			osDelay(10); // 실패한 경우 재시도 전에 딜레이 추가 (필요에 따라 조정)
		}
	}

	//torque on
    for (int retryCount = 0; retryCount < 5; ++retryCount) {
    	dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
		if (dxl_comm_result == COMM_SUCCESS) {
			break; // 성공하면 루프 탈출
		} else {
			if(retryCount >= 4)
				operatingStatus_ = Status_InitError;
			osDelay(10); // 실패한 경우 재시도 전에 딜레이 추가 (필요에 따라 조정)
		}
	}

	osDelay(10);


	for (int retryCount = 0; retryCount < 5; ++retryCount) {
		dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, sID_, ADDR_PRO_PRESENT_POSITION,(uint32_t*)&monitor_.raw_current_posi, &dxl_error);
		if (dxl_comm_result == COMM_SUCCESS) {
			monitor_.raw_command_posi = monitor_.raw_current_posi;
			break; // 성공하면 루프 탈출
		} else {
			if(retryCount >= 4)
				operatingStatus_ = Status_InitError;
			osDelay(10); // 실패한 경우 재시도 전에 딜레이 추가 (필요에 따라 조정)
		}
	}

	//status return lv (68) - 1 : 읽기 명령에만 응답
	for (int retryCount = 0; retryCount < 5; ++retryCount) {
		dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_STATUS_RETURN_LV, 1, &dxl_error);
		if (dxl_comm_result == COMM_SUCCESS) {
			break; // 성공하면 루프 탈출
		} else {
			osDelay(10); // 실패한 경우 재시도 전에 딜레이 추가 (필요에 따라 조정)
		}
	}

	if(operatingStatus_ == Status_Init){
		operatingStatus_ = Status_PreRun;
		for (int retryCount = 0; retryCount < 5; ++retryCount) {
			dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_PRO_LED, 1, &dxl_error);
			if (dxl_comm_result == COMM_SUCCESS) {
				break; // 성공하면 루프 탈출
			} else {
				osDelay(10); // 실패한 경우 재시도 전에 딜레이 추가 (필요에 따라 조정)
			}
		}
	}
}









