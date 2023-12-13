#include "cpp_Motor_Class_v2/Motor.h"

class DXL_motor : public Motor {
public:
    DXL_motor(uint8_t gID, uint8_t sID, MotorType motorType) :
    	Motor(gID, sID, motorType){
        // Set IDs and other parameters for DXL_motor
    }

    /* Motor Class 상속 */
    /* input 필수 기능 */
    //init
    void setSettingInfo(uint8_t dir, uint16_t angle, uint16_t initPosi, uint16_t reducer_ratio);

    //control
    void setPosition(uint16_t targetPosition);

    /* output 필수 기능*/
    uint16_t getPosition() const;//const 상태변경x 읽기전용


private:

};
