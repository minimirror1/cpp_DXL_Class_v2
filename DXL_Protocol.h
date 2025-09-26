/*
 * DXL_Protocol.h
 *
 *  Created on: Dec 13, 2023
 *      Author: minim
 */

#ifndef CPP_DXL_CLASS_V2_DXL_PROTOCOL_H_
#define CPP_DXL_CLASS_V2_DXL_PROTOCOL_H_


// Protocol version
#define PROTOCOL_VERSION1               1.0                 // See which protocol version is used in the Dynamixel
#define PROTOCOL_VERSION2               2.0
// Default setting
#define DXL2_ID                         2                   // Dynamixel#2 ID: 2
#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller

#define ADDR_PRO_DRIVE_MODE				10
#define ADDR_OPERATING_MODE				11
#define ADDR_STATUS_RETURN_LV			68
#define ADDR_PRO_ACCELE					108
#define ADDR_PRO_VELOCITY				112

#define ADDR_PRO_TORQUE_ENABLE          64
#define ADDR_PRO_LED					65
#define ADDR_PRO_HARDWARE_ERROR_STATUS  70
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque

// Hardware Error Status Bits (Address 70)
#define HW_ERROR_INPUT_VOLTAGE          0x01    // Bit 0: Input Voltage Error - 인가된 전압이 설정된 동작전압 범위를 벗어난 경우
#define HW_ERROR_OVERHEATING            0x04    // Bit 2: Overheating Error - 설정된 온도를 벗어난 경우 (기본값)
#define HW_ERROR_MOTOR_ENCODER          0x08    // Bit 3: Motor Encoder Error - 모터의 엔코더가 동작하지 않는 경우
#define HW_ERROR_ELECTRICAL_SHOCK       0x10    // Bit 4: Electrical Shock Error - 전기적으로 회로가 충격을 받았거나, 입력 전력이 부족한 경우 (기본값)
#define HW_ERROR_OVERLOAD               0x20    // Bit 5: Overload Error - 최대 출력으로 제어할 수 없는 하중이 지속적으로 발생한 경우 (기본값)
#define HW_ERROR_COMM_FAILURE           0x40    // Bit 6: Communication Failure - 통신 실패
#define HW_ERROR_NOT_ASSIGNED           0x80    // Bit 7: Motor Not Assigned - 할당되지 않은 모터

// Hardware Error Check Macros
#define IS_HW_ERROR_INPUT_VOLTAGE(status)       ((status) & HW_ERROR_INPUT_VOLTAGE)
#define IS_HW_ERROR_OVERHEATING(status)         ((status) & HW_ERROR_OVERHEATING)
#define IS_HW_ERROR_MOTOR_ENCODER(status)       ((status) & HW_ERROR_MOTOR_ENCODER)
#define IS_HW_ERROR_ELECTRICAL_SHOCK(status)    ((status) & HW_ERROR_ELECTRICAL_SHOCK)
#define IS_HW_ERROR_OVERLOAD(status)            ((status) & HW_ERROR_OVERLOAD)
#define IS_HW_ERROR_COMM_FAILURE(status)        ((status) & HW_ERROR_COMM_FAILURE)
#define IS_HW_ERROR_NOT_ASSIGNED(status)        ((status) & HW_ERROR_NOT_ASSIGNED)
#define HAS_ANY_HW_ERROR(status)                ((status) != 0)

// Multiple Error Check Macros
#define GET_HW_ERROR_COUNT(status)              (__builtin_popcount(status & 0x3D)) // Bit 0,2,3,4,5 하드웨어 에러만 카운트
#define GET_SYSTEM_ERROR_COUNT(status)          (__builtin_popcount(status & 0xC0)) // Bit 6,7 시스템 에러만 카운트
#define HAS_CRITICAL_ERRORS(status)             ((status) & (HW_ERROR_ELECTRICAL_SHOCK | HW_ERROR_MOTOR_ENCODER))
#define HAS_THERMAL_OVERLOAD_ERRORS(status)     ((status) & (HW_ERROR_OVERHEATING | HW_ERROR_OVERLOAD))
#define HAS_SYSTEM_ERRORS(status)               ((status) & (HW_ERROR_COMM_FAILURE | HW_ERROR_NOT_ASSIGNED))


#endif /* CPP_DXL_CLASS_V2_DXL_PROTOCOL_H_ */
