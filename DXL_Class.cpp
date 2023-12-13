#include "DXL_Class.h"




/* Motor Class 상속 */
/* input 필수 기능 */
//init
void DXL_motor::setSettingInfo(uint8_t dir, uint16_t angle, uint16_t initPosi, uint16_t reducer_ratio)
{

}

//control
void DXL_motor::setPosition(uint16_t targetPosition)
{

}


/* output 필수 기능*/
uint16_t DXL_motor::getPosition() const
{
    return 0;
}
