/*
 * DXL_ErrorTypes.h
 *
 *  Created on: Dec 2024
 *      Author: AI Assistant
 *  
 *  Description: DXL 에러 분류 시스템 - Phase 1 구현
 *               Dynamixel Protocol 2.0 기반 에러 타입 정의
 */

#ifndef CPP_DXL_CLASS_V2_DXL_ERRORTYPES_H_
#define CPP_DXL_CLASS_V2_DXL_ERRORTYPES_H_

#include <stdint.h>

// =============================================================================
// 8. 시스템 상수 정의 (먼저 정의)
// =============================================================================
#define DXL_ERROR_CONSECUTIVE_THRESHOLD     5       // 연속 에러 임계값
#define DXL_HARDWARE_ERROR_STATUS_ADDR      70      // Hardware Error Status 주소
#define DXL_ERROR_TIMEOUT_DISCONNECT_COUNT  5       // 타임아웃 시 연결 해제 판단 횟수
#define DXL_ERROR_MAX_CONSECUTIVE_ERRORS    255     // 최대 연속 에러 횟수

// =============================================================================
// 1. 통신 에러 (Communication Errors) - DXL SDK 결과 기반
// =============================================================================
enum DXL_CommError {
    DXL_COMM_NONE = 0,          // 에러 없음 (COMM_SUCCESS)
    DXL_COMM_BUSY,              // 포트 사용 중 (COMM_PORT_BUSY)
    DXL_COMM_TXFAIL,            // 송신 실패 (COMM_TX_FAIL)
    DXL_COMM_RXFAIL,            // 수신 실패 (COMM_RX_FAIL)
    DXL_COMM_TXERROR,           // 송신 에러 (COMM_TX_ERROR)
    DXL_COMM_TIMEOUT,           // 타임아웃 (COMM_RX_TIMEOUT)
    DXL_COMM_CORRUPT,           // 패킷 손상 (COMM_RX_CORRUPT)
    DXL_COMM_NOT_AVAILABLE,     // 지원 안함 (COMM_NOT_AVAILABLE)
    DXL_COMM_UNKNOWN            // 알 수 없는 에러
};

// =============================================================================
// 2. 하드웨어 에러 (Hardware Errors) - Hardware Error Status(70) 기반
// =============================================================================
enum HardwareError {
    HW_NONE = 0,            // 에러 없음
    HW_OVERLOAD,            // Bit 5: 과부하 에러
    HW_ELECTRICAL,          // Bit 4: 전기적 충격 에러
    HW_ENCODER,             // Bit 3: 모터 엔코더 에러
    HW_OVERHEATING,         // Bit 2: 과열 에러
    HW_VOLTAGE,             // Bit 0: 입력 전압 에러
    HW_READ_ERROR,          // Hardware Error Status 읽기 실패
    HW_UNKNOWN              // 알 수 없는 하드웨어 에러
};

// =============================================================================
// 3. 프로토콜 에러 (Protocol Errors) - Status Packet Error Number 기반
// =============================================================================
enum ProtocolError {
    PROTO_NONE = 0,         // 에러 없음
    PROTO_RESULT_FAIL,      // 0x01: Result Fail
    PROTO_INSTRUCTION,      // 0x02: Instruction Error
    PROTO_CRC,              // 0x03: CRC Error
    PROTO_DATA_RANGE,       // 0x04: Data Range Error
    PROTO_DATA_LENGTH,      // 0x05: Data Length Error
    PROTO_DATA_LIMIT,       // 0x06: Data Limit Error
    PROTO_ACCESS            // 0x07: Access Error
};

// =============================================================================
// 4. 초기화 결과 (Initialization Result)
// =============================================================================
enum InitResult {
    INIT_SUCCESS,           // 초기화 성공 -> Status_PreRun
    INIT_COMM_FAIL,         // 통신 실패 -> Status_InitError (상위에서 예외처리)
    INIT_HW_ERROR,          // 하드웨어 에러 -> Status_InitError
    INIT_PROTO_ERROR,       // 프로토콜 에러 -> Status_InitError
    INIT_TIMEOUT,           // 타임아웃 -> Status_InitError
    INIT_NOT_CONNECTED      // 모터 미연결 -> Status_InitError
};

// =============================================================================
// 5. 종합 에러 상태 구조체
// =============================================================================
struct DXL_ErrorStatus {
    // 에러 분류
    DXL_CommError comm_error;       // 통신 에러
    HardwareError hw_error;         // 하드웨어 에러
    ProtocolError proto_error;      // 프로토콜 에러
    
    // 에러 통계
    uint32_t error_count;           // 총 에러 발생 횟수
    uint32_t last_error_time;       // 마지막 에러 발생 시간 (tick)
    uint8_t consecutive_errors;     // 연속 에러 횟수
    
    // 모터 상태
    bool is_motor_connected;        // 모터 연결 상태
    bool alert_bit_set;             // Alert 비트 설정 여부
    
    // 생성자
    DXL_ErrorStatus() {
        comm_error = DXL_COMM_NONE;
        hw_error = HW_NONE;
        proto_error = PROTO_NONE;
        error_count = 0;
        last_error_time = 0;
        consecutive_errors = 0;
        is_motor_connected = false;
        alert_bit_set = false;
    }
    
    // 에러 상태 초기화
    void reset() {
        comm_error = DXL_COMM_NONE;
        hw_error = HW_NONE;
        proto_error = PROTO_NONE;
        consecutive_errors = 0;
        alert_bit_set = false;
    }
    
    // 에러 발생 여부 확인
    bool hasError() const {
        return (comm_error != DXL_COMM_NONE) || 
               (hw_error != HW_NONE) || 
               (proto_error != PROTO_NONE);
    }
    
    // 심각한 에러 여부 확인 (Status_InitError로 전환해야 하는 경우)
    bool isCriticalError() const {
        return (comm_error == DXL_COMM_TIMEOUT && consecutive_errors > DXL_ERROR_CONSECUTIVE_THRESHOLD) ||
               (hw_error != HW_NONE && hw_error != HW_READ_ERROR) ||
               (proto_error == PROTO_CRC && consecutive_errors > 3) ||
               (consecutive_errors >= DXL_ERROR_MAX_CONSECUTIVE_ERRORS);
    }
    
    // 에러 통계 업데이트
    void updateErrorStats(uint32_t current_time) {
        if (hasError()) {
            error_count++;
            last_error_time = current_time;
            if (consecutive_errors < DXL_ERROR_MAX_CONSECUTIVE_ERRORS) {
                consecutive_errors++;
            }
        } else {
            consecutive_errors = 0;
        }
    }
    
    // 연결 상태 업데이트
    void updateConnectionStatus() {
        if (comm_error == DXL_COMM_TIMEOUT || comm_error == DXL_COMM_RXFAIL) {
            if (consecutive_errors > DXL_ERROR_TIMEOUT_DISCONNECT_COUNT) {
                is_motor_connected = false;
            }
        } else if (comm_error == DXL_COMM_NONE) {
            is_motor_connected = true;
        }
    }
};

// =============================================================================
// 6. 상위 전달용 4글자 에러 코드 매핑
// =============================================================================

// 통신 에러 코드 (4글자)
#define ERROR_CODE_COMM_NONE    "NONE"
#define ERROR_CODE_COMM_BUSY    "BUSY"
#define ERROR_CODE_COMM_TXFAIL  "TXFL"
#define ERROR_CODE_COMM_RXFAIL  "RXFL"
#define ERROR_CODE_COMM_TXERR   "TXER"
#define ERROR_CODE_COMM_TIMEOUT "TOUT"
#define ERROR_CODE_COMM_CORRUPT "CORP"
#define ERROR_CODE_COMM_NAVAIL  "NAVL"
#define ERROR_CODE_COMM_UNKNOWN "UNKN"

// 하드웨어 에러 코드 (4글자)
#define ERROR_CODE_HW_NONE      "NONE"
#define ERROR_CODE_HW_OVERLOAD  "OVLD"
#define ERROR_CODE_HW_ELECTRIC  "ELEC"
#define ERROR_CODE_HW_ENCODER   "ENCR"
#define ERROR_CODE_HW_OVERHEAT  "HEAT"
#define ERROR_CODE_HW_VOLTAGE   "VOLT"
#define ERROR_CODE_HW_READ_ERR  "RDER"
#define ERROR_CODE_HW_UNKNOWN   "HWER"

// 프로토콜 에러 코드 (숫자 그대로 사용)
#define ERROR_CODE_PROTO_NONE   "NONE"
#define ERROR_CODE_PROTO_0x01   "0x01"
#define ERROR_CODE_PROTO_0x02   "0x02"
#define ERROR_CODE_PROTO_0x03   "0x03"
#define ERROR_CODE_PROTO_0x04   "0x04"
#define ERROR_CODE_PROTO_0x05   "0x05"
#define ERROR_CODE_PROTO_0x06   "0x06"
#define ERROR_CODE_PROTO_0x07   "0x07"


// =============================================================================
// 7. DXL SDK 상수 매핑 (참조용)
// =============================================================================
// DynamixelSDK.h에서 정의된 상수들과 매핑
/*
#define COMM_SUCCESS        0       // tx or rx packet communication success
#define COMM_PORT_BUSY      -1000   // Port is busy (in use)
#define COMM_TX_FAIL        -1001   // Failed transmit instruction packet
#define COMM_RX_FAIL        -1002   // Failed get status packet
#define COMM_TX_ERROR       -2000   // Incorrect instruction packet
#define COMM_RX_WAITING     -3000   // Now receiving status packet
#define COMM_RX_TIMEOUT     -3001   // There is no status packet
#define COMM_RX_CORRUPT     -3002   // Incorrect status packet
#define COMM_NOT_AVAILABLE  -9000   // Protocol does not support this function
*/

// Hardware Error Status Bit 매핑
/*
Bit 7: 미사용
Bit 6: 미사용  
Bit 5: Overload Error
Bit 4: Electrical Shock Error
Bit 3: Motor Encoder Error
Bit 2: Overheating Error
Bit 1: 미사용
Bit 0: Input Voltage Error
*/

#endif /* CPP_DXL_CLASS_V2_DXL_ERRORTYPES_H_ */
