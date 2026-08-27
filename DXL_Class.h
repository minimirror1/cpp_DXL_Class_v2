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
/* Case 3(3점 분할)의 분할 기준이 되는 명령값.
 * 명령 범위가 0~4095 라 정확한 중앙은 2047.5 이지만 상위와 2048 로 합의하였다.
 * 그 결과 앞 구간은 2048 스텝, 뒤 구간은 2047 스텝으로 1스텝 비대칭이다. */
#define MRS_CMD_MID		2048
/* Home 방향 표기*/
#define DXL_ROT_CW		true
#define DXL_ROT_CCW		false

/* 초기화 매핑 Case (op 패킷의 case_num)
 *
 * 설정값은 DATA1 + DATA_OP 두 패킷의 조합으로 완성된다.
 * DATA_OP 8바이트는 2바이트씩 4분할되며 case_num 에 따라 슬롯 의미가 달라진다.
 * (프로토콜 구조체 : prtc_data_ctl_init_driver_data_op_dxl2_t)
 *
 *  case_num | min      | ref_position | max        | DATA1 angle/direction
 *  ---------+----------+--------------+------------+----------------------
 *     1     | -        | 시작 엔코더값 | -          | 사용
 *     2     | 모션 0   | -            | 모션 4095  | 미사용
 *     3     | 모션 0   | 중앙         | 모션 4095  | 미사용
 *     4     | -        | 중앙 엔코더값 | -          | 사용
 *
 * Case 2, 3 은 min/max 의 대소로 방향이 결정되므로 direction 을 참조하지 않는다. */
#define DXL_MAP_CASE_START_ANGLE		1	//시작점·각도 매핑
#define DXL_MAP_CASE_TWO_POINT			2	//양끝점 매핑
#define DXL_MAP_CASE_THREE_POINT		3	//3점 분할 매핑
#define DXL_MAP_CASE_CENTER_SYMMETRIC	4	//중앙점·대칭각 매핑

#define DXL_MAP_CASE_MIN	DXL_MAP_CASE_START_ANGLE
#define DXL_MAP_CASE_MAX	DXL_MAP_CASE_CENTER_SYMMETRIC

/* 미지원 case_num 을 받았을 때 ACK 의 case_num 자리에 실어 거부를 알리는 값 */
#define DXL_MAP_CASE_REJECT	0xFFFF


class DXL_motor : public Motor {
public:
	/* 매핑 정보.
	 * 4가지 Case 는 모두 "명령값 -> raw 카운트" 대응점 집합으로 환원되므로,
	 * Case 별 산출식은 setSettingData_op() 에서 한 번만 풀고
	 * 이후 동작은 아래 대응점만 참조한다. */
	struct DXL_Setting{
		uint16_t caseNum_;	//수신한 case_num (0 : 미설정)

		int32_t p0_;		//명령 0        에 대응하는 raw 카운트
		int32_t pMid_;		//명령 MRS_CMD_MID 에 대응하는 raw 카운트 (Case 3 에서만 유효)
		int32_t p4095_;		//명령 MRS_CMD_MAX 에 대응하는 raw 카운트
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
    void setSettingData_op(uint8_t gID, uint8_t sID,
                           uint16_t caseNum, uint16_t minPosi,
                           uint16_t refPosi, uint16_t maxPosi) override;

    /* case_num 이 이 펌웨어가 처리할 수 있는 값인지 확인한다.
     * 설정을 반영하기 전에 프로토콜 경계에서 먼저 걸러내기 위한 용도. */
    static bool isSupportedCase(uint16_t caseNum){
    	return (DXL_MAP_CASE_MIN <= caseNum) && (caseNum <= DXL_MAP_CASE_MAX);
    }

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
    /* 상위 명령값(0 ~ MRS_CMD_MAX)을 모터 raw 카운트로 환산한다.
     * setPosition / timeCheckPosition / getDefaultPosi 가 모두 이 함수를 거치므로
     * Case 별 분기는 여기 한 곳에만 존재한다. */
    int32_t mrsToRaw(int32_t mrsCmd) const;

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
