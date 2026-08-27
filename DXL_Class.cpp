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

	dxl_setting_.caseNum_ = 0;
	dxl_setting_.p0_ = 0;
	dxl_setting_.pMid_ = 0;
	dxl_setting_.p4095_ = 0;

	serial_= serial;
	protocol_version_ = PROTOCOL_VERSION2;

	portHandler_ = dynamixel::PortHandler::getPortHandler(DEVICENAME, serial_);
	packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version_);

}

/* 매핑 대응점을 모터의 물리 위치 범위로 잘라낸다.
 *
 * Position control mode 의 유효 카운트는 0 ~ DXL_MAX_POSI 이다.
 * 상위 설정값 조합에 따라 대응점이 이 범위를 벗어날 수 있는데
 * (예: Case 1 에서 시작점 1000, 90도, 역방향이면 끝점이 -23),
 * 그대로 두면 범위 밖 카운트가 모터로 나간다.
 *
 * 잘라낸 만큼 실제 구동 범위는 설정보다 좁아진다.
 * 즉 이 클램프가 동작했다는 것은 상위 설정값이 모터 사양을 벗어났다는 뜻이며,
 * 의도한 각도가 나오지 않으므로 설정값 자체를 점검해야 한다. */
static int32_t clampToPosiRange(int32_t cnt)
{
	if(cnt < 0)				return 0;
	if(cnt > DXL_MAX_POSI)	return DXL_MAX_POSI;
	return cnt;
}

/* Motor Class 상속 */
/* input 필수 기능 */
//init
void DXL_motor::setSettingData_op(uint8_t gID, uint8_t sID,
                                  uint16_t caseNum, uint16_t minPosi,
                                  uint16_t refPosi, uint16_t maxPosi)
{
	if(id_check(gID, sID) == false)
		return;

	// 호출측(dxl_task)에서 이미 걸러내지만, 잘못된 값이 매핑에 반영되면
	// 모터가 엉뚱한 위치로 움직이므로 여기서도 한 번 더 막는다.
	if(isSupportedCase(caseNum) == false)
		return;

	// float -> int 변환에서 소수점 이하가 버려지나(반올림 아님) 오차가 최대 1카운트라 무시함.
	// setting_.angle 은 float 이므로 여기까지 소수점 각도가 그대로 전달된다.
	// Case 2, 3 은 각도를 쓰지 않으므로 이 값이 의미를 갖지 않는다.
	const float angleCnt = setting_.angle / 360 * DXL_MAX_POSI;
	// Case 1, 4 의 각도를 어느 방향으로 펼칠지 결정한다.
	const int32_t dirSign = (setting_.dir == DXL_ROT_CW)? 1 : -1;

	int32_t p0 = 0;
	int32_t pMid = 0;
	int32_t p4095 = 0;

	switch(caseNum)
	{
	case DXL_MAP_CASE_START_ANGLE:
		// 시작점에서 동작 각도만큼 펼친다.
		p0    = (int32_t)refPosi;
		p4095 = p0 + dirSign * (int32_t)angleCnt;
		break;

	case DXL_MAP_CASE_TWO_POINT:
		// 양끝점을 그대로 사용한다. min > max 면 역방향이 된다.
		p0    = (int32_t)minPosi;
		p4095 = (int32_t)maxPosi;
		break;

	case DXL_MAP_CASE_THREE_POINT:
		// 중앙점을 경계로 두 구간을 따로 매핑한다.
		p0    = (int32_t)minPosi;
		pMid  = (int32_t)refPosi;
		p4095 = (int32_t)maxPosi;
		break;

	case DXL_MAP_CASE_CENTER_SYMMETRIC: {
		// 중앙점 기준으로 전체 각도의 절반씩 양쪽으로 펼친다.
		const int32_t halfCnt = (int32_t)(angleCnt / 2);
		p0    = (int32_t)refPosi - dirSign * halfCnt;
		p4095 = (int32_t)refPosi + dirSign * halfCnt;
		break;
	}

	default:
		return;
	}

	// 대응점을 모터 물리 범위로 고정한다. 세 점이 모두 범위 안이면
	// 그 사이를 보간하는 mrsToRaw() 의 결과도 항상 범위 안에 머문다.
	dxl_setting_.caseNum_ = caseNum;
	dxl_setting_.p0_      = clampToPosiRange(p0);
	dxl_setting_.pMid_    = clampToPosiRange(pMid);
	dxl_setting_.p4095_   = clampToPosiRange(p4095);

	// 상위 재부팅이나 설정값 변경으로 재수신될 수 있으므로 이전 상태와 무관하게 반영한다.
	// 이미 Status_Run 이던 모터도 설정 수신 단계로 되돌려 초기위치 재이동을 거치게 한다.
	operatingStatus_ = Statis_SettingOk;
	f_assign = true;
}

/* 상위 명령값 -> 모터 raw 카운트 */
int32_t DXL_motor::mrsToRaw(int32_t mrsCmd) const
{
	// 상위 명령 스케일(0 ~ MRS_CMD_MAX) 밖의 값은 경계로 고정한다.
	// 수신 타입이 uint16_t 라 4095 를 넘는 값이 들어올 수 있으며, 그대로 두면
	// 설정된 구동 범위를 벗어난 카운트가 모터로 나간다.
	if(mrsCmd < 0)			mrsCmd = 0;
	if(mrsCmd > MRS_CMD_MAX)	mrsCmd = MRS_CMD_MAX;

	if(dxl_setting_.caseNum_ == DXL_MAP_CASE_THREE_POINT){
		if(mrsCmd < MRS_CMD_MID){
			return dxl_setting_.p0_
				+ (int32_t)((float)(dxl_setting_.pMid_ - dxl_setting_.p0_) * mrsCmd / MRS_CMD_MID);
		}
		return dxl_setting_.pMid_
			+ (int32_t)((float)(dxl_setting_.p4095_ - dxl_setting_.pMid_)
				* (mrsCmd - MRS_CMD_MID) / (MRS_CMD_MAX - MRS_CMD_MID));
	}

	// Case 1, 2, 4 는 두 끝점 사이의 단일 선형 구간이다.
	return dxl_setting_.p0_
		+ (int32_t)((float)(dxl_setting_.p4095_ - dxl_setting_.p0_) * mrsCmd / MRS_CMD_MAX);
}

//control
void DXL_motor::setPosition(int32_t targetPosition)
{
	if(!f_assign) return;

	// 클램프는 mrsToRaw() 안에서 처리하지만, 모니터 값에도 같은 범위를 남겨야
	// timeCheckPosition() 이 재계산할 때 동일한 결과가 나온다.
	if(targetPosition < 0)			targetPosition = 0;
	if(targetPosition > MRS_CMD_MAX)	targetPosition = MRS_CMD_MAX;

	monitor_.mrs_current_posi = targetPosition;

	int32_t temp_raw_posi = mrsToRaw(targetPosition);

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
		int32_t temp_raw_posi = mrsToRaw(monitor_.mrs_current_posi);

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
 *    f_assign 은 setSettingData_op() 에서 매핑 대응점을 설정할 때
 *    비로소 true 가 된다. 즉 MRS_RX_DATA_OP 수신 전에는 조그가 아무 동작도
 *    하지 않는다. 임시값으로 DATA1/DATA_OP 를 먼저 보내는 운용이라면
 *    현행 유지가 맞고, 아니라면 가드 조건을 손봐야 한다.
 *
 * 2) 범위 검사 기준이 raw 카운트 기준의 0~4095 임
 *    실제 구동 범위는 [p0_, p4095_] 이며 역방향이면 상/하한이 뒤바뀐다.
 *    p0_ 가 0 이 아니거나 역방향인 경우 범위를 벗어난 조그가 허용되거나,
 *    반대로 정상 구간인데도 차단될 수 있다.
 *    1) 의 운용 방식이 확정되어야 올바른 기준을 정할 수 있음.
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
    // 초기 위치도 상위 명령값(DATA1 의 init_position)이므로 동작 중 명령과 같은 식으로 환산한다.
    // Case 3 의 구간 분기도 그대로 적용된다.
    return mrsToRaw(setting_.initPosi);
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









