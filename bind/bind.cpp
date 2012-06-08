#include <iostream>
#include <boost/function.hpp>
#include <boost/bind.hpp>

class Boo
{
public:
    void f(int i, int j){
        std::cout<<"Sum is "<<(i+j)<<"."<<std::endl; 
    }
};

void f2(int i, int j)
{
    std::cout<<"Sum is "<<(i+j)<<"."<<std::endl; 
}

int main(){
    Boo boo; 
    boost::function2<void, int, int> func, func2, func3;
    func= boost::bind(&Boo::f, &boo, _1, _2);
    func(13, 7);
    func2= boost::bind(f2, _1, _2);
    func2(13, 7);
    func3 = &f2;
    func3(1,2);

    return 0;
}
