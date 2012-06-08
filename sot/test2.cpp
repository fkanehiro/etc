#include <iostream>
#include <sstream>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/linear-algebra.h>
#include <sot/core/device.hh>
#include <sot/core/pool.hh>
#include <sot/core/sot.hh>
#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/matrix-rotation.hh>
#include <dynamic-graph/corba/interpreter.hh>
#include <sot/core/robot-simu.hh>
#include <sot-dynamic/dynamic-hrp2.h>

#define SOT_OPENHRP_OUTPUT_FILE "/tmp/sot.out"

using namespace dynamicgraph;
using namespace dynamicgraph::sot;

dynamicgraph::corba::Interpreter interpreter_;
Entity* entity_;
Entity* solver_;

static void runPython(std::ostream& file, const std::string& command,
		      corba::Interpreter& interpreter) {
  file << ">>> " << command << std::endl;
  std::string value = interpreter.python(command);
  if (value != "None")
    file << value;
}

void setup(double *initial, double *target)
{
    std::stringstream sstr;
    sstr << "random_posture.setup((";
    for (int i=0; i<36; i++){
        sstr << initial[i];
        if (i!=35) sstr << ",";
    }
    sstr <<"),[";
    for (int i=0; i<20; i++){
        sstr << target[i];
        if (i!=19) sstr << ",";
    }
    sstr <<"])";
    runPython(std::cout, sstr.str(), interpreter_);
}

int main(int argc, char *argv[])
{
  std::ostream &aof = std::cout;
  runPython(aof, "import sys, os", interpreter_);
  runPython(aof, "pythonpath = ''", interpreter_);
  runPython(aof,
	    "with open('./python-path') as f:\n"
	    "  pythonpath = f.readline().rstrip('\\n ')", interpreter_);
  runPython(aof, "path = []", interpreter_);
  runPython(aof,
	    "for p in pythonpath.split(':'):\n"
	    "  if p not in sys.path:\n"
	    "    path.append(p)", interpreter_);
  runPython(aof, "path.extend(sys.path)", interpreter_);
  runPython(aof, "sys.path = path", interpreter_);
  runPython(aof, "import random_posture",  interpreter_);

  Entity *entity;
  if (!g_pool.existEntity("robot_device", entity)){
    std::cerr << "can't find robot_device" << std::endl;
    return 1;
  }
  Device* robot_device = dynamic_cast<Device *>(entity);
  if (!robot_device){
      std::cerr << "dynamic_cast failed(1)" << std::endl;
      return 1;
  }

  SignalBase<int> *signal;

  signal = &entity->getSignal("state");
  Signal< ml::Vector,int > *state;
  state = dynamic_cast< Signal< ml::Vector, int>* >(signal);
  if (!state){
    std::cerr << "dynamic_cast failed(5)" << std::endl;
    return 1;
  }

  signal = &entity->getSignal("control");
  Signal< ml::Vector,int > *control;
  control = dynamic_cast< Signal< ml::Vector, int>* >(signal);
  if (!state){
    std::cerr << "dynamic_cast failed(6)" << std::endl;
    return 1;
  }

  ml::Vector s, c;

  double halfSitting[] = {
      0.0, 0.0, 0.648702, 0.0, 0.0, 0.0,
      0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0,
      0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0,
      0.0, 0.0,
      0.0, 0.0,
      0.261799, -0.17453, 0.0, -0.5236, 0.0, 0.0, 0.1,
      0.261799,  0.17453, 0.0, -0.5236, 0.0, 0.0, 0.1
  };
  double target[] = {
      0.0, 0.5,
      0.0, 0.0,
      0.0, 0.0,
      0.261799, -1.17453, 0.0, -1.5236, 0.0, 0.0, 0.1,
      0.261799,  1.17453, 0.0, -1.5236, 0.0, 0.0, 0.1
  };
  setup(halfSitting, target);
  do{
      robot_device->increment(0.05);
      c = control->accessCopy();
      //std::cout << c.norm() << std::endl;
  }while(c.norm() > 0.02);
  runPython(aof, "random_posture.cleanup()", interpreter_);

  s = state->accessCopy();
  std::cout << "final posture:" << s << std::endl;

  return 0;
}
