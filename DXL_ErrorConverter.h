/*
 * DXL_ErrorConverter.h
 *
 *  Created on: Dec 2024
 *      Author: AI Assistant
 *  
 *  Description: DXL 에러를 상위 전달용 4글자 코드로 변환하는 유틸리티
 */

#ifndef CPP_DXL_CLASS_V2_DXL_ERRORCONVERTER_H_
#define CPP_DXL_CLASS_V2_DXL_ERRORCONVERTER_H_

#include "DXL_ErrorTypes.h"

class DXL_ErrorConverter {
public:
    // DXL 에러 상태를 4글자 코드로 변환
    static const char* convertToErrorCode(const DXL_ErrorStatus& error_status) {
        // 우선순위: 하드웨어 > 프로토콜 > 통신 에러
        
        // 1. 하드웨어 에러 우선 (가장 심각)
        if (error_status.hw_error != HW_NONE) {
            return convertHardwareError(error_status.hw_error);
        }
        
        // 2. 프로토콜 에러
        if (error_status.proto_error != PROTO_NONE) {
            return convertProtocolError(error_status.proto_error);
        }
        
        // 3. 통신 에러
        if (error_status.comm_error != DXL_COMM_NONE) {
            return convertCommError(error_status.comm_error);
        }
        
        // 에러 없음
        return ERROR_CODE_COMM_NONE;
    }
    
private:
    // 통신 에러 변환
    static const char* convertCommError(DXL_CommError comm_error) {
        switch (comm_error) {
            case DXL_COMM_NONE:         return ERROR_CODE_COMM_NONE;
            case DXL_COMM_BUSY:         return ERROR_CODE_COMM_BUSY;
            case DXL_COMM_TXFAIL:       return ERROR_CODE_COMM_TXFAIL;
            case DXL_COMM_RXFAIL:       return ERROR_CODE_COMM_RXFAIL;
            case DXL_COMM_TXERROR:      return ERROR_CODE_COMM_TXERR;
            case DXL_COMM_TIMEOUT:      return ERROR_CODE_COMM_TIMEOUT;
            case DXL_COMM_CORRUPT:      return ERROR_CODE_COMM_CORRUPT;
            case DXL_COMM_NOT_AVAILABLE: return ERROR_CODE_COMM_NAVAIL;
            case DXL_COMM_UNKNOWN:      return ERROR_CODE_COMM_UNKNOWN;
            default:                    return ERROR_CODE_COMM_UNKNOWN;
        }
    }
    
    // 하드웨어 에러 변환
    static const char* convertHardwareError(HardwareError hw_error) {
        switch (hw_error) {
            case HW_NONE:           return ERROR_CODE_HW_NONE;
            case HW_OVERLOAD:       return ERROR_CODE_HW_OVERLOAD;
            case HW_ELECTRICAL:     return ERROR_CODE_HW_ELECTRIC;
            case HW_ENCODER:        return ERROR_CODE_HW_ENCODER;
            case HW_OVERHEATING:    return ERROR_CODE_HW_OVERHEAT;
            case HW_VOLTAGE:        return ERROR_CODE_HW_VOLTAGE;
            case HW_READ_ERROR:     return ERROR_CODE_HW_READ_ERR;
            case HW_UNKNOWN:        return ERROR_CODE_HW_UNKNOWN;
            default:                return ERROR_CODE_HW_UNKNOWN;
        }
    }
    
    // 프로토콜 에러 변환
    static const char* convertProtocolError(ProtocolError proto_error) {
        switch (proto_error) {
            case PROTO_NONE:        return ERROR_CODE_PROTO_NONE;
            case PROTO_RESULT_FAIL: return ERROR_CODE_PROTO_0x01;
            case PROTO_INSTRUCTION: return ERROR_CODE_PROTO_0x02;
            case PROTO_CRC:         return ERROR_CODE_PROTO_0x03;
            case PROTO_DATA_RANGE:  return ERROR_CODE_PROTO_0x04;
            case PROTO_DATA_LENGTH: return ERROR_CODE_PROTO_0x05;
            case PROTO_DATA_LIMIT:  return ERROR_CODE_PROTO_0x06;
            case PROTO_ACCESS:      return ERROR_CODE_PROTO_0x07;
            default:                return ERROR_CODE_PROTO_NONE;
        }
    }
};

#endif /* CPP_DXL_CLASS_V2_DXL_ERRORCONVERTER_H_ */
