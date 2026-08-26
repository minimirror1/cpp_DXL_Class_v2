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

#include "UART_Class.h"

#include "../cpp_tick/cpp_tick.h"


//#define DXLLIB_VERSION "0.0.1"	//
#define DXLLIB_VERSION "0.0.2"	//버전, Jog 기능 추가


/* 한바퀴 최대 카운트 (모터 하드웨어 스펙) */
#define DXL_MAX_POSI	4095
/* 상위 위치 명령 스케일 0 ~ MRS_CMD_MAX (프로토콜 스펙)
 * DXL_MAX_POSI 와 현재 값이 같지만 의미가 다르므로 분리해서 쓴다.
 * 한쪽만 변경될 경우 조용히 어긋나는 것을 막기 위함. */
#define MRS_CMD_MAX		4095
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
    
    /* Hardware Error Status 관련 */
    uint8_t getHardwareErrorStatus() const;             // Hardware Error Status 읽기

    /* 공통 funtion */
    void init() override;		//Motor Active

    void multiTurnInit();

private:
    /* 속성 */
	bool f_assign;	//true : 할당됨, false : 할당안됨

	Serial *serial_;
	float protocol_version_;
	dynamixel::PortHandler *portHandler_;
	dynamixel::PacketHandler *packetHandler_;

	DXL_Setting dxl_setting_;

	Tick com_limit;
	
	// 통신 실패 카운터 (const 함수에서 수정 가능하도록 mutable)
	mutable uint8_t commFailureCount;
};
#endif /* INC_DXL_Class_H_ */
