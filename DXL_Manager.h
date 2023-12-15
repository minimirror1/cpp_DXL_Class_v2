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

    void allMotorProcess(void){
    	for (size_t i = 1; i < maxSize; i++) {
			dxlObjects[i]->process();
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
