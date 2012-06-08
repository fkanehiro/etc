#include "childB.h"
#include "childA.h"
#include <iostream>

void childB::purevirtual(parent *i_parent)
{
    std::cout << "childB::purevirtual" << std::endl;

    childA *a = dynamic_cast<childA *>(i_parent);
    if (a){
        std::cerr << "dynamic_cast succeeded" << std::endl;
    }else{
        std::cerr << "dynamic_cast failed" << std::endl;
    }
}

void childB::print()
{
    std::cout << "childB::print() : " << *m_ptr << std::endl;
}
