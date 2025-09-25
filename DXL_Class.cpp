#include "cpp_Motor_Class_v2/Motor.h"
#include "cpp_DXL_SDK_v2/src/DynamixelSDK.h"
#include "DXL_Protocol.h"
#include "DXL_Class.h"
#include "DXL_ErrorTypes.h"


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
void DXL_motor::setPosition(int32_t targetPosition)
{
	if(!f_assign) return;

	monitor_.mrs_current_posi = targetPosition;

	float ratio = (float)targetPosition/4095;
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
		float ratio = (float)monitor_.mrs_current_posi/4095;
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
    int32_t cntCalc = (float)dxl_setting_.homeCnt_ + (dxl_setting_.rangeCnt_ * (float)setting_.initPosi/4095);
    return cntCalc;
}


/* 공통 funtion */
void DXL_motor::init()
{
    // Phase 2: 개선된 초기화 사용 (기존 순서와 항목 정확히 유지)
    InitResult result = improvedInit();
    
    // 결과에 따른 상태 설정 (기존 예외처리 구조 유지)
    switch (result) {
        case INIT_SUCCESS:
            // 이미 improvedInit()에서 Status_PreRun으로 설정됨
            break;
            
        case INIT_NOT_CONNECTED:
        case INIT_COMM_FAIL:
        case INIT_HW_ERROR:
        case INIT_PROTO_ERROR:
        case INIT_TIMEOUT:
        default:
            // 모든 실패 경우는 Status_InitError로 설정 (상위에서 예외처리)
            operatingStatus_ = Status_InitError;
            break;
    }
    
    // Phase 2: 에러 정보를 구체적으로 저장하여 나중에 상위로 전달 가능
    // (기존 코드와 호환성 유지)
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

// =============================================================================
// Phase 2: 개선된 초기화 로직 구현
// =============================================================================

InitResult DXL_motor::improvedInit() {
    int dxl_comm_result = COMM_SUCCESS;
    uint8_t dxl_error = 0;
    const int max_retries = 5;  // 기존과 동일한 재시도 횟수
    
    // 에러 상태 초기화
    error_status_.reset();
    operatingStatus_ = Status_Init;
    
    // === 기존 초기화 순서 정확히 따르기 ===
    
    // 1단계: LED OFF (기존과 동일: 2번 실행)
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_PRO_LED, 0, &dxl_error);
    detectAndClassifyErrors(dxl_comm_result, dxl_error);
    osDelay(10);
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_PRO_LED, 0, &dxl_error);
    detectAndClassifyErrors(dxl_comm_result, dxl_error);
    osDelay(10);
    
    // 통신 실패 시 조기 반환
    if (dxl_comm_result != COMM_SUCCESS) {
        if (dxl_comm_result == COMM_RX_TIMEOUT || dxl_comm_result == COMM_RX_FAIL) {
            operatingStatus_ = Status_InitError;
            return INIT_NOT_CONNECTED;
        } else {
            operatingStatus_ = Status_InitError;
            return INIT_COMM_FAIL;
        }
    }
    
    // 2단계: Status Return Level = 2 (모든 패킷에 응답)
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_STATUS_RETURN_LV, 2, &dxl_error);
    detectAndClassifyErrors(dxl_comm_result, dxl_error);
    
    if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) {
        operatingStatus_ = Status_InitError;
        return (dxl_comm_result != COMM_SUCCESS) ? INIT_COMM_FAIL : INIT_PROTO_ERROR;
    }
    
    // 3단계: Torque OFF (기존과 동일한 재시도 로직)
    for (int retryCount = 0; retryCount < max_retries; ++retryCount) {
        dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        detectAndClassifyErrors(dxl_comm_result, dxl_error);
        if (dxl_comm_result == COMM_SUCCESS) {
            break;
        } else {
            osDelay(10);
        }
    }
    
    if (dxl_comm_result != COMM_SUCCESS) {
        operatingStatus_ = Status_InitError;
        return INIT_COMM_FAIL;
    }
    
    // 4단계: Drive Mode = 4 (Time-based Profile)
    for (int retryCount = 0; retryCount < max_retries; ++retryCount) {
        dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_PRO_DRIVE_MODE, 4, &dxl_error);
        detectAndClassifyErrors(dxl_comm_result, dxl_error);
        if (dxl_comm_result == COMM_SUCCESS) {
            break;
        } else {
            osDelay(10);
        }
    }
    
    // 5단계: Operating Mode = 3 (Position Control Mode)
    for (int retryCount = 0; retryCount < max_retries; ++retryCount) {
        dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_OPERATING_MODE, 3, &dxl_error);
        detectAndClassifyErrors(dxl_comm_result, dxl_error);
        if (dxl_comm_result == COMM_SUCCESS) {
            break;
        } else {
            osDelay(10);
        }
    }
    
    // 6단계: Profile Acceleration = 25 (4Byte 값)
    for (int retryCount = 0; retryCount < max_retries; ++retryCount) {
        dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, sID_, ADDR_PRO_ACCELE, 25, &dxl_error);
        detectAndClassifyErrors(dxl_comm_result, dxl_error);
        if (dxl_comm_result == COMM_SUCCESS) {
            break;
        } else {
            osDelay(10);
        }
    }
    
    // 7단계: Profile Velocity = 50 (4Byte 값)
    for (int retryCount = 0; retryCount < max_retries; ++retryCount) {
        dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, sID_, ADDR_PRO_VELOCITY, 50, &dxl_error);
        detectAndClassifyErrors(dxl_comm_result, dxl_error);
        if (dxl_comm_result == COMM_SUCCESS) {
            break;
        } else {
            osDelay(10);
        }
    }
    
    // 8단계: Torque ON
    for (int retryCount = 0; retryCount < max_retries; ++retryCount) {
        dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        detectAndClassifyErrors(dxl_comm_result, dxl_error);
        if (dxl_comm_result == COMM_SUCCESS) {
            break;
        } else {
            if(retryCount >= 4)
                operatingStatus_ = Status_InitError;
            osDelay(10);
        }
    }
    
    osDelay(10);
    
    // 9단계: 현재 위치 읽기
    for (int retryCount = 0; retryCount < max_retries; ++retryCount) {
        dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, sID_, ADDR_PRO_PRESENT_POSITION, (uint32_t*)&monitor_.raw_current_posi, &dxl_error);
        detectAndClassifyErrors(dxl_comm_result, dxl_error);
        if (dxl_comm_result == COMM_SUCCESS) {
            monitor_.raw_command_posi = monitor_.raw_current_posi;
            break;
        } else {
            if(retryCount >= 4)
                operatingStatus_ = Status_InitError;
            osDelay(10);
        }
    }
    
    // 10단계: Status Return Level = 1 (읽기 명령에만 응답)
    for (int retryCount = 0; retryCount < max_retries; ++retryCount) {
        dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_STATUS_RETURN_LV, 1, &dxl_error);
        detectAndClassifyErrors(dxl_comm_result, dxl_error);
        if (dxl_comm_result == COMM_SUCCESS) {
            break;
        } else {
            osDelay(10);
        }
    }
    
    // 11단계: 성공 시 상태 변경 및 LED ON
    if(operatingStatus_ == Status_Init){
        operatingStatus_ = Status_PreRun;
        error_status_.is_motor_connected = true;
        
        for (int retryCount = 0; retryCount < max_retries; ++retryCount) {
            dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, sID_, ADDR_PRO_LED, 1, &dxl_error);
            if (dxl_comm_result == COMM_SUCCESS) {
                break;
            } else {
                osDelay(10);
            }
        }
        
        return INIT_SUCCESS;
    }
    
    // 초기화 실패
    operatingStatus_ = Status_InitError;
    return INIT_COMM_FAIL;
}

// =============================================================================
// Phase 1: 에러 분석 함수들 구현
// =============================================================================

DXL_CommError DXL_motor::analyzeCommResult(int dxl_comm_result) {
    switch (dxl_comm_result) {
        case COMM_SUCCESS:          return DXL_COMM_NONE;
        case COMM_PORT_BUSY:        return DXL_COMM_BUSY;
        case COMM_TX_FAIL:          return DXL_COMM_TXFAIL;
        case COMM_RX_FAIL:          return DXL_COMM_RXFAIL;
        case COMM_TX_ERROR:         return DXL_COMM_TXERROR;
        case COMM_RX_TIMEOUT:       return DXL_COMM_TIMEOUT;
        case COMM_RX_CORRUPT:       return DXL_COMM_CORRUPT;
        case COMM_NOT_AVAILABLE:    return DXL_COMM_NOT_AVAILABLE;
        default:                    return DXL_COMM_UNKNOWN;
    }
}

ProtocolError DXL_motor::analyzeStatusPacketError(uint8_t error_field) {
    // Alert 비트 확인 및 설정 (개선: 항상 정확한 상태로 설정)
    error_status_.alert_bit_set = (error_field & 0x80) != 0;
    
    // Error Number 추출 (Bit 6~0)
    uint8_t error_number = error_field & 0x7F;
    
    switch (error_number) {
        case 0x00:  return PROTO_NONE;
        case 0x01:  return PROTO_RESULT_FAIL;
        case 0x02:  return PROTO_INSTRUCTION;
        case 0x03:  return PROTO_CRC;
        case 0x04:  return PROTO_DATA_RANGE;
        case 0x05:  return PROTO_DATA_LENGTH;
        case 0x06:  return PROTO_DATA_LIMIT;
        case 0x07:  return PROTO_ACCESS;
        default:    return PROTO_NONE;  // 알 수 없는 에러 번호는 NONE으로 처리
    }
}

HardwareError DXL_motor::readAndAnalyzeHardwareError() {
    if (!error_status_.alert_bit_set) {
        return HW_NONE;  // Alert 비트가 설정되지 않은 경우
    }
    
    uint8_t hw_error_status = 0;
    uint8_t dxl_error = 0;
    
    // Hardware Error Status(70) 읽기 (개선: 상수 사용)
    int dxl_comm_result = packetHandler_->read1ByteTxRx(
        portHandler_, sID_, DXL_HARDWARE_ERROR_STATUS_ADDR, &hw_error_status, &dxl_error);
    
    if (dxl_comm_result != COMM_SUCCESS) {
        return HW_READ_ERROR;  // 통신 실패
    }
    
    // 개선: dxl_error도 확인
    if (dxl_error != 0) {
        return HW_READ_ERROR;  // Status Packet에서 에러 발생
    }
    
    // 각 비트 확인 (우선순위: 심각한 에러부터)
    if (hw_error_status & 0x20) return HW_OVERLOAD;     // Bit 5: 과부하
    if (hw_error_status & 0x10) return HW_ELECTRICAL;   // Bit 4: 전기적 충격
    if (hw_error_status & 0x04) return HW_OVERHEATING;  // Bit 2: 과열
    if (hw_error_status & 0x01) return HW_VOLTAGE;      // Bit 0: 전압
    if (hw_error_status & 0x08) return HW_ENCODER;      // Bit 3: 엔코더
    
    // 다른 비트가 설정된 경우
    if (hw_error_status != 0) {
        return HW_UNKNOWN;  // 알 수 없는 하드웨어 에러
    }
    
    return HW_NONE;  // 실제로는 에러 없음 (Alert 비트 오류?)
}

void DXL_motor::detectAndClassifyErrors(int comm_result, uint8_t error_field) {
    // 이전 에러 상태 초기화 (Alert 비트 제외)
    error_status_.comm_error = DXL_COMM_NONE;
    error_status_.proto_error = PROTO_NONE;
    error_status_.hw_error = HW_NONE;
    
    // 통신 에러 분석
    error_status_.comm_error = analyzeCommResult(comm_result);
    
    // 프로토콜 에러 분석 (Alert 비트도 함께 처리)
    error_status_.proto_error = analyzeStatusPacketError(error_field);
    
    // 하드웨어 에러 분석 (Alert 비트가 설정된 경우만)
    if (error_status_.alert_bit_set) {
        error_status_.hw_error = readAndAnalyzeHardwareError();
    }
    
    // 개선: 구조체 내부 함수 사용으로 코드 간소화
    error_status_.updateErrorStats(HAL_GetTick());
    error_status_.updateConnectionStatus();
}





