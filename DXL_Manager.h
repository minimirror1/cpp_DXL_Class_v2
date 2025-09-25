/*
 * DXL_Manager.h
 *
 *  Created on: Dec 14, 2023
 *      Author: minim
 */

#ifndef CPP_DXL_CLASS_V2_DXL_MANAGER_H_
#define CPP_DXL_CLASS_V2_DXL_MANAGER_H_
#include <iostream>
#include "DXL_Class.h"
#include "Motor.h"
#include "UART_Class.h"

class DXL_Manager {
public:
    DXL_Manager() = default;

    // DXL_Class 객체를 추가하는 함수
    void addDXLObject(uint8_t gID, uint8_t sID, Motor::MotorType motorType, Serial *serial) {
        // 동적 할당하여 객체 생성
        DXL_motor *dxlObject = new DXL_motor(gID, sID, motorType, serial);
        dxlObjects[sID] = dxlObject;

//        // 추가된 객체를 배열에 할당
//        if (currentSize <= maxSize) {
//            dxlObjects[currentSize++] = dxlObject;
//        } else {
//            // 배열이 가득 찬 경우 예외 처리
//            delete dxlObject; // 동적으로 할당된 메모리 해제
//            //std::cout << "Array is full!" << std::endl;
//        }
    }

    /* All */
    // 모든 DXL_Class 객체의 초기화 함수 호출. 연결된 모터를 알 수 있다.
    void initializeAll() {
        for (size_t i = 1; i < maxSize; i++) {
            dxlObjects[i]->init();
        }
    }
    
    // Phase 2: 개선된 초기화 함수 (에러 정보 포함)
    void improvedInitializeAll() {
        for (size_t i = 1; i < maxSize; i++) {
            InitResult result = dxlObjects[i]->improvedInit();
            // 결과는 각 모터의 error_status_에 저장됨
            // 필요시 로그나 상위 시스템에 전달 가능
        }
    }
    void initializeAll_MultiTrun() {
           for (size_t i = 1; i < 3; i++) {
               dxlObjects[i]->multiTurnInit();
           }
       }


    // 모든 DXL_Class 객체의 위치 동기화 함수 호출
    void syncAllPositions() {
        for (size_t i = 1; i < maxSize; i++) {
            dxlObjects[i]->defaultPosi_Ready();
            dxlObjects[i]->defaultPosi_Move();
        }
    }

    // 모든 DXL_Class 객체의 현재 위치를 출력하는 함수
    void printAllPositions() const {
        for (size_t i = 1; i < maxSize; i++) {
            //std::cout << "Current Position: " << dxlObjects[i]->getPosition() << std::endl;
        }
    }

    // 특정 DXL_Class 객체의 위치 설정 함수 호출
    void setPosition(uint8_t sid, uint16_t targetPosition ) {
    	if (0 < sid && sid < maxSize) {
            dxlObjects[sid]->setPosition(targetPosition);
        } else {
            //std::cout << "Invalid index!" << std::endl;
        }
    }
    void setRawPosition(uint8_t sid, int32_t rawCount){
    	if (0 < sid && sid < maxSize) {
			dxlObjects[sid]->setRawPosition(rawCount);
		} else {
			//std::cout << "Invalid index!" << std::endl;
		}
    }

    void setJogMove(uint8_t sid, int32_t jogCounter ) {
		if (0 < sid && sid < maxSize) {
			dxlObjects[sid]->setJogMove(jogCounter);
		} else {
			//std::cout << "Invalid index!" << std::endl;
		}
	}

    void allMotorProcess(void){
    	for (size_t i = 1; i < maxSize; i++) {
			dxlObjects[i]->process();
		}
    }

    void allTimeCheckPosi(void){
    	for (size_t i = 1; i < maxSize; i++) {
			dxlObjects[i]->timeCheckPosition();
		}
    }


    /* index */
    Motor::Status getStatus(uint8_t gid, uint8_t sid){
    	if (0 < gid && sid < maxSize) {
			return dxlObjects[sid]->getStatus(gid, sid);
		}
    	return Motor::Status_None;
    }

    void setSettingInfo(uint8_t gid, uint8_t sid, uint8_t dir, uint16_t angle, uint16_t initPosi, uint16_t reducer_ratio){
    	if (0 < sid && sid < maxSize) { //sid = index
			dxlObjects[sid]->setSettingInfo(gid, sid, dir, angle, initPosi, reducer_ratio);
		}
    }
    void setSettingData_op(uint8_t gid, uint8_t sid, uint32_t data_1, uint32_t data_2){
    	if (0 < sid && sid < maxSize) { //sid = index
			dxlObjects[sid]->setSettingData_op(gid, sid, data_1, data_2);
		}
    }

    void setDefaultPosi_Ready(uint8_t gid, uint8_t sid){
    	if (0 < sid && sid < maxSize) { //sid = index
			dxlObjects[sid]->defaultPosi_Ready();
		}
    }
    
    /* Phase 2: 에러 상태 관리 함수들 */
    // 특정 모터의 에러 상태 확인
    const DXL_ErrorStatus& getMotorErrorStatus(uint8_t sid) const {
        if (0 < sid && sid < maxSize) {
            return dxlObjects[sid]->getErrorStatus();
        }
        // 잘못된 인덱스의 경우 기본 에러 상태 반환
        static DXL_ErrorStatus default_error;
        return default_error;
    }
    
    // 특정 모터의 4글자 에러 코드 확인
    const char* getMotorErrorCode(uint8_t sid) const {
        if (0 < sid && sid < maxSize) {
            return dxlObjects[sid]->getErrorCode();
        }
        return "INVD"; // Invalid ID
    }
    
    // 모든 모터의 에러 상태 확인
    bool hasAnyError() const {
        for (size_t i = 1; i < maxSize; i++) {
            if (dxlObjects[i]->hasError()) {
                return true;
            }
        }
        return false;
    }
    
    // 심각한 에러가 있는 모터 개수 확인
    uint8_t getCriticalErrorCount() const {
        uint8_t count = 0;
        for (size_t i = 1; i < maxSize; i++) {
            if (dxlObjects[i]->isCriticalError()) {
                count++;
            }
        }
        return count;
    }
    
    // 연결되지 않은 모터 개수 확인
    uint8_t getDisconnectedMotorCount() const {
        uint8_t count = 0;
        for (size_t i = 1; i < maxSize; i++) {
            const DXL_ErrorStatus& status = dxlObjects[i]->getErrorStatus();
            if (!status.is_motor_connected) {
                count++;
            }
        }
        return count;
    }
    
    // 모든 모터의 에러 상태 초기화
    void resetAllErrorStatus() {
        for (size_t i = 1; i < maxSize; i++) {
            dxlObjects[i]->resetErrorStatus();
        }
    }
    
    /* Phase 3: 실시간 모니터링 함수들 */
    // 모든 연결된 모터에 대해 실시간 에러 체크 수행
    void performRuntimeErrorCheckAll() {
        for (size_t i = 1; i < maxSize; i++) {
            dxlObjects[i]->performRuntimeErrorCheck();
        }
    }
    
    // 새로운 에러가 발생한 모터 ID 목록 반환
    void getMotorsWithNewErrors(uint8_t* error_motor_ids, uint8_t* count) {
        *count = 0;
        for (size_t i = 1; i < maxSize && *count < 30; i++) {
            if (dxlObjects[i]->hasError()) {
                const DXL_ErrorStatus& status = dxlObjects[i]->getErrorStatus();
                // 최근 에러가 있는지 확인 (예: 최근 1초 이내)
                uint32_t current_time = HAL_GetTick();
                if (status.last_error_time > 0 && 
                    (current_time - status.last_error_time) < 1000) {
                    error_motor_ids[(*count)++] = i;
                }
            }
        }
    }
    
    // 에러 발생 모터의 간단한 정보 출력 (디버깅용)
    void printErrorSummary() const {
        uint8_t error_count = 0;
        uint8_t disconnected_count = 0;
        uint8_t critical_count = 0;
        
        for (size_t i = 1; i < maxSize; i++) {
            const DXL_ErrorStatus& status = dxlObjects[i]->getErrorStatus();
            if (status.hasError()) {
                error_count++;
                if (status.isCriticalError()) {
                    critical_count++;
                }
            }
            if (!status.is_motor_connected) {
                disconnected_count++;
            }
        }
        
        // STM32에서는 printf 대신 다른 방법 사용 가능
        // printf("Error Summary - Total: %d, Critical: %d, Disconnected: %d\n", error_count, critical_count, disconnected_count);
    }
    
    // Phase 3 개선: 글로벌 에러 로그 조회 기능
    uint8_t getSystemErrorLogs(DXL_ErrorLogEntry* entries, uint8_t max_entries) const {
        return DXL_ErrorLogger::getAllValidLogs(entries, max_entries);
    }
    
    // 시스템 전체 에러 로그 초기화
    void clearSystemErrorLogs() {
        DXL_ErrorLogger::clearLogs();
    }


    // 소멸자에서 동적으로 할당한 객체들의 메모리를 해제
    ~DXL_Manager() {
        for (size_t i = 1; i < maxSize; i++) {
            delete dxlObjects[i];
        }
    }

private:
    static constexpr size_t maxSize = 31; // 최대 크기 지정
    DXL_motor *dxlObjects[maxSize];
};


#endif /* CPP_DXL_CLASS_V2_DXL_MANAGER_H_ */
