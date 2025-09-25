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

// Phase 3: 에러 로그 시스템 상수
#define DXL_ERROR_LOG_MAX_ENTRIES           10      // 최대 에러 로그 항목 수
#define DXL_ERROR_LOG_RETENTION_TIME        30000   // 에러 로그 보관 시간 (30초)

// Phase 3 개선: 에러 우선순위 정의
#define ERROR_PRIORITY_NONE                 0
#define ERROR_PRIORITY_LOW                  1
#define ERROR_PRIORITY_MEDIUM               2
#define ERROR_PRIORITY_HIGH                 3
#define ERROR_PRIORITY_CRITICAL             4

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
// 4. 에러 로그 항목 (Phase 3)
// =============================================================================
struct DXL_ErrorLogEntry {
    uint32_t timestamp;         // 에러 발생 시간 (HAL_GetTick())
    uint8_t motor_id;          // 모터 ID
    DXL_CommError comm_error;  // 통신 에러
    HardwareError hw_error;    // 하드웨어 에러
    ProtocolError proto_error; // 프로토콜 에러
    
    // 초기화
    void init(uint32_t time, uint8_t id, DXL_CommError comm, HardwareError hw, ProtocolError proto) {
        timestamp = time;
        motor_id = id;
        comm_error = comm;
        hw_error = hw;
        proto_error = proto;
    }
    
    // 로그 항목이 유효한지 확인 (보관 시간 내)
    bool isValid(uint32_t current_time) const {
        return (current_time - timestamp) < DXL_ERROR_LOG_RETENTION_TIME;
    }
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
    void updateConnectionStatus(bool connected) {
        is_motor_connected = connected;
        if (!connected) {
            consecutive_errors++;
        }
    }
    
    // Phase 3 개선: 에러 우선순위 기반 업데이트
    void updateCommErrorWithPriority(DXL_CommError new_error) {
        if (new_error != DXL_COMM_NONE) {
            if (getCommErrorPriority(new_error) >= getCommErrorPriority(comm_error)) {
                comm_error = new_error;
            }
        }
    }
    
    void updateHwErrorWithPriority(HardwareError new_error) {
        if (new_error != HW_NONE) {
            if (getHwErrorPriority(new_error) >= getHwErrorPriority(hw_error)) {
                hw_error = new_error;
            }
        }
    }
    
    void updateProtoErrorWithPriority(ProtocolError new_error) {
        if (new_error != PROTO_NONE) {
            if (getProtoErrorPriority(new_error) >= getProtoErrorPriority(proto_error)) {
                proto_error = new_error;
            }
        }
    }

private:
    // 통신 에러 우선순위 반환
    uint8_t getCommErrorPriority(DXL_CommError error) const {
        switch (error) {
            case DXL_COMM_NONE:         return ERROR_PRIORITY_NONE;
            case DXL_COMM_BUSY:         return ERROR_PRIORITY_LOW;
            case DXL_COMM_TXFAIL:       return ERROR_PRIORITY_MEDIUM;
            case DXL_COMM_RXFAIL:       return ERROR_PRIORITY_HIGH;
            case DXL_COMM_TXERROR:      return ERROR_PRIORITY_MEDIUM;
            case DXL_COMM_TIMEOUT:      return ERROR_PRIORITY_HIGH;
            case DXL_COMM_CORRUPT:      return ERROR_PRIORITY_HIGH;
            case DXL_COMM_NOT_AVAILABLE: return ERROR_PRIORITY_CRITICAL;
            case DXL_COMM_UNKNOWN:      return ERROR_PRIORITY_MEDIUM;
            default:                    return ERROR_PRIORITY_LOW;
        }
    }
    
    // 하드웨어 에러 우선순위 반환
    uint8_t getHwErrorPriority(HardwareError error) const {
        switch (error) {
            case HW_NONE:           return ERROR_PRIORITY_NONE;
            case HW_VOLTAGE:        return ERROR_PRIORITY_MEDIUM;
            case HW_OVERHEATING:    return ERROR_PRIORITY_HIGH;
            case HW_ENCODER:        return ERROR_PRIORITY_HIGH;
            case HW_ELECTRICAL:     return ERROR_PRIORITY_CRITICAL;
            case HW_OVERLOAD:       return ERROR_PRIORITY_CRITICAL;
            case HW_READ_ERROR:     return ERROR_PRIORITY_MEDIUM;
            case HW_UNKNOWN:        return ERROR_PRIORITY_MEDIUM;
            default:                return ERROR_PRIORITY_LOW;
        }
    }
    
    // 프로토콜 에러 우선순위 반환
    uint8_t getProtoErrorPriority(ProtocolError error) const {
        switch (error) {
            case PROTO_NONE:            return ERROR_PRIORITY_NONE;
            case PROTO_RESULT_FAIL:     return ERROR_PRIORITY_LOW;
            case PROTO_INSTRUCTION:     return ERROR_PRIORITY_MEDIUM;
            case PROTO_CRC:             return ERROR_PRIORITY_HIGH;
            case PROTO_DATA_RANGE:      return ERROR_PRIORITY_MEDIUM;
            case PROTO_DATA_LENGTH:     return ERROR_PRIORITY_MEDIUM;
            case PROTO_DATA_LIMIT:      return ERROR_PRIORITY_MEDIUM;
            case PROTO_ACCESS:          return ERROR_PRIORITY_HIGH;
            default:                    return ERROR_PRIORITY_LOW;
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

// =============================================================================
// 6. 글로벌 에러 로그 시스템 (Phase 3)
// =============================================================================
class DXL_ErrorLogger {
private:
    static DXL_ErrorLogEntry log_entries_[DXL_ERROR_LOG_MAX_ENTRIES];
    static uint8_t log_index_;
    static uint8_t log_count_;

public:
    // Phase 3 개선: Thread-safe 에러 로그 추가
    static void addErrorLog(uint8_t motor_id, DXL_CommError comm_error, 
                           HardwareError hw_error, ProtocolError proto_error) {
        // STM32에서 간단한 임계 영역 보호 (인터럽트 비활성화)
        __disable_irq();
        
        uint32_t current_time = HAL_GetTick();
        
        // 순환 버퍼 방식으로 로그 저장
        log_entries_[log_index_].init(current_time, motor_id, comm_error, hw_error, proto_error);
        log_index_ = (log_index_ + 1) % DXL_ERROR_LOG_MAX_ENTRIES;
        
        if (log_count_ < DXL_ERROR_LOG_MAX_ENTRIES) {
            log_count_++;
        }
        
        __enable_irq();
    }
    
    // 유효한 에러 로그 개수 반환
    static uint8_t getValidLogCount() {
        uint32_t current_time = HAL_GetTick();
        uint8_t valid_count = 0;
        
        for (uint8_t i = 0; i < log_count_; i++) {
            if (log_entries_[i].isValid(current_time)) {
                valid_count++;
            }
        }
        
        return valid_count;
    }
    
    // Phase 3 개선: 에러 로그 검색 최적화
    static bool getRecentErrorLog(uint8_t motor_id, DXL_ErrorLogEntry* entry) {
        uint32_t current_time = HAL_GetTick();
        
        // 최근 항목부터 역순 검색 (단순화된 인덱스 계산)
        uint8_t search_count = (log_count_ < DXL_ERROR_LOG_MAX_ENTRIES) ? log_count_ : DXL_ERROR_LOG_MAX_ENTRIES;
        
        for (uint8_t i = 0; i < search_count; i++) {
            // 최신 항목부터 역순으로 계산
            uint8_t idx = (log_index_ + DXL_ERROR_LOG_MAX_ENTRIES - 1 - i) % DXL_ERROR_LOG_MAX_ENTRIES;
            
            if (log_entries_[idx].motor_id == motor_id && log_entries_[idx].isValid(current_time)) {
                *entry = log_entries_[idx];
                return true;
            }
        }
        
        return false;
    }
    
    // Phase 3 개선: 모든 유효한 에러 로그 조회 (디버깅용)
    static uint8_t getAllValidLogs(DXL_ErrorLogEntry* entries, uint8_t max_entries) {
        uint32_t current_time = HAL_GetTick();
        uint8_t found_count = 0;
        uint8_t search_count = (log_count_ < DXL_ERROR_LOG_MAX_ENTRIES) ? log_count_ : DXL_ERROR_LOG_MAX_ENTRIES;
        
        for (uint8_t i = 0; i < search_count && found_count < max_entries; i++) {
            uint8_t idx = (log_index_ + DXL_ERROR_LOG_MAX_ENTRIES - 1 - i) % DXL_ERROR_LOG_MAX_ENTRIES;
            
            if (log_entries_[idx].isValid(current_time)) {
                entries[found_count++] = log_entries_[idx];
            }
        }
        
        return found_count;
    }
    
    // Phase 3 개선: Thread-safe 로그 초기화
    static void clearLogs() {
        __disable_irq();
        log_index_ = 0;
        log_count_ = 0;
        __enable_irq();
    }
};

#endif /* CPP_DXL_CLASS_V2_DXL_ERRORTYPES_H_ */
