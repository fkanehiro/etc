#ifndef __PORT_H__
#define __PORT_H__
#include <iostream>

class shared_data_base
{
public:
    virtual ~shared_data_base() {}
    virtual void share(shared_data_base *i_base)=0;
};

template<class T> 
class shared_data : public shared_data_base
{
public:
    shared_data(T &i_data) : m_ptr(&i_data) {}
    void share(shared_data_base *i_base){
        shared_data<T *> *dst = dynamic_cast<shared_data<T *> *>(i_base);
        if (dst){
            *(dst->m_ptr) = m_ptr;
        }else{
        }
    }
    T *m_ptr;
};

#endif
