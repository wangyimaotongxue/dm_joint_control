#include "damiao.h"
#include "unistd.h"
#include <cmath>


damiao::Motor M1(damiao::DMH3510,0x01, 0x00);
damiao::Motor M2(damiao::DM4310,0x02, 0x00);
std::shared_ptr<SerialPort> serial;
damiao::Motor_Control dm(serial);