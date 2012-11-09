#include <iostream>
#include <Model/HumanoidBody.h>

std::ostream& operator<<(std::ostream &ost, motion_generator::HumanoidBodyPtr robot);
std::istream& operator>>(std::istream &ist, motion_generator::HumanoidBodyPtr robot);
void setHalfConf(motion_generator::HumanoidBodyPtr robot);
