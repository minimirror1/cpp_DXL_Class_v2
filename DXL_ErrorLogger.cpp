/*
 * DXL_ErrorLogger.cpp
 *
 * Phase 3: 에러 로그 시스템 구현
 * 
 * Static 멤버 변수 정의
 */

#include "DXL_ErrorTypes.h"

// Static 멤버 변수 정의
DXL_ErrorLogEntry DXL_ErrorLogger::log_entries_[DXL_ERROR_LOG_MAX_ENTRIES];
uint8_t DXL_ErrorLogger::log_index_ = 0;
uint8_t DXL_ErrorLogger::log_count_ = 0;
