/*
 * DXL_Class.h
 *
 *  Created on: Dec 13, 2023
 *      Author: minim
 */

#ifndef INC_DXL_Class_H_
#define INC_DXL_Class_H_

#include "cmsis_os.h"

#include "cpp_Motor_Class_v2/Motor.h"
#include "cpp_DXL_SDK_v2/src/DynamixelSDK.h"
#include "DXL_Protocol.h"
#include "DXL_ErrorTypes.h"
#include "DXL_ErrorConverter.h"

#include "UART_Class.h"

#include "../cpp_tick/cpp_tick.h"


//#define DXLLIB_VERSION "0.0.1"	//
#define DXLLIB_VERSION "0.0.2"	//버전, Jog 기능 추가


/* 한바퀴 최대 카운트 */
#define DXL_MAX_POSI	4095
/* Home 방향 표기*/
#define DXL_ROT_CW		true
#define DXL_ROT_CCW		false


class DXL_motor : public Motor {
public:
	struct DXL_Setting{
		/* 통신 수신 */
		int32_t homeCnt_;	//DXL 기어 . 기준 home 위치

		/* 계산 */
		int32_t rangeCnt_;	//동작 각도 Cnt 환산
	};

    DXL_motor(uint8_t gID, uint8_t sID, MotorType motorType, Serial *serial);

    virtual ~DXL_motor() {
    }

    void getDriverVersion(char *major, char *minor, char *patch){
    	int majorNum, minorNum, patchNum;
		sscanf(DXLLIB_VERSION, "%d.%d.%d", &majorNum, &minorNum, &patchNum);

		sprintf(major, "%d", majorNum);
		sprintf(minor, "%d", minorNum);
		sprintf(patch, "%d", patchNum);
    }

    /* Motor Class 상속 */
    /* input 필수 기능 */
    /* init */
    void setSettingData_op(uint8_t gID, uint8_t sID, uint32_t data_1, uint32_t data_2) override;

    /* control */
    void setPosition(int32_t targetPosition) override;		//MRS motion count 동작

    void timeCheckPosition();

    void setRawPosition(int32_t targetPosition) override;	//motor raw count 동작

    void setJogMove(int jogCounter) override;				//motor jog move 동작

    /* output 필수 기능*/
    uint16_t getCurrentPosition() const override ; 	//const 상태변경x 읽기전용
    int32_t getDefaultPosi() const override ;

    /* 공통 funtion */
    void init() override;		//Motor Active
    InitResult improvedInit();  // Phase 2: 개선된 초기화 (에러 분류 포함)

    void multiTurnInit();

    /* Phase 1: 에러 분석 함수들 */
    DXL_CommError analyzeCommResult(int dxl_comm_result);
    ProtocolError analyzeStatusPacketError(uint8_t error_field);
    HardwareError readAndAnalyzeHardwareError();
    void detectAndClassifyErrors(int comm_result, uint8_t error_field);
    
    /* 에러 상태 관리 */
    const DXL_ErrorStatus& getErrorStatus() const { return error_status_; }
    void resetErrorStatus() { error_status_.reset(); }
    bool hasError() const { return error_status_.hasError(); }
    bool isCriticalError() const { return error_status_.isCriticalError(); }
    
    /* 상위 전달용 에러 코드 변환 */
    const char* getErrorCode() const { 
        return DXL_ErrorConverter::convertToErrorCode(error_status_); 
    }

private:
    /* 속성 */
	bool f_assign;	//true : 할당됨, false : 할당안됨

	Serial *serial_;
	float protocol_version_;
	dynamixel::PortHandler *portHandler_;
	dynamixel::PacketHandler *packetHandler_;

	DXL_Setting dxl_setting_;
	DXL_ErrorStatus error_status_;  // Phase 1: 에러 상태 관리

	Tick com_limit;

};
#endif /* INC_DXL_Class_H_ */
