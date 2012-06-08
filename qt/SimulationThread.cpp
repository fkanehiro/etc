#include <iostream>
#include <QMutexLocker>
#include "SimulationThread.h"
#include "Simulator.h"

SimulationThread::SimulationThread(Simulator* simulator, QObject* parent)
    : QThread(parent),
      simulator_(simulator),
      abort_(false),
      pause_(false)
{
    if(parent == NULL)
    {
        moveToThread(this);
    }
}


SimulationThread::~SimulationThread()
{
    abortThread();
    if( isRunning() ) quit();
}

void SimulationThread::abortThread()
{
    QMutexLocker lock(&mutex_);
    abort_ = true;
    if(pause_) pause();
    pauseCondition_.wakeAll();
}

bool SimulationThread::pause()
{
    bool ret = false;
    if(isRunning())
    {
        if(pause_){
            pause_ = false;
            pauseCondition_.wakeOne();
        } else {
            pause_ = true;
        }
        ret = pause_;
    }
    return ret;
}

bool SimulationThread::startThread()
{
    bool ret = true;

    if(isRunning())
    {
        ret = false;
    } else {
        abort_ = false;
        pause_ = false;
        start(QThread::HighestPriority);
    }
    return ret;
}

void SimulationThread::run()
{
    while( !abort_ ){
        bool ret = true;
        try{
            for(unsigned int i = 0; i < 10; ++i )
            {
                if(abort_ || pause_)
                    break;
                if ( (ret = !simulator_->oneStep()) )
                    break;
            }
        }catch(std::exception& ex){
            std::cerr << ex.what() << std::endl;
            std::cerr << "Catch std::exception in " << __FILE__ << ":" << __LINE__ << std::endl;
            abort_ = true;
        } catch (CORBA::SystemException& ex){
            std::cerr << "Catch CORBA::SystemException in " << __FILE__ << ":" << __LINE__ << std::endl;
            abort_ = true;
        }catch(...){
            std::cerr << "Catch error something in " << __FILE__ << ":" << __LINE__ << std::endl;
            abort_ = true;
        }

        if( ret )
        {
            abort_ = true;
        } else if(pause_)
        {
            mutex_.lock();
            pauseCondition_.wait(&mutex_);
            mutex_.unlock();
        }
    }

}
