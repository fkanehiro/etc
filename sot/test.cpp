#include <iostream>
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
  runPython(aof, "from dynamic_graph.sot.dynamics.tools import *",
	    interpreter_);
  runPython(aof, "robot = Hrp2Laas(\"robot\")",
	    interpreter_);
  runPython(aof, "from dynamic_graph.sot.dynamics.solver import Solver",
	    interpreter_);
  runPython(aof, "solver = Solver(robot)",
	    interpreter_);
#if 0
  runPython(aof, "reach(robot, 'left-wrist', 0.25, 0, 0.1)",
	    interpreter_);
#endif
  runPython(aof, "solver.sot.push(robot.tasks['right-ankle'].name)",
	    interpreter_);
  runPython(aof, "solver.sot.push(robot.tasks['left-ankle'].name)",
	    interpreter_);
  runPython(aof, "solver.sot.push(robot.tasks['right-wrist'].name)",
	    interpreter_);
  runPython(aof, "solver.sot.push(robot.tasks['left-wrist'].name)",
	    interpreter_);
  //interpreter_.startCorbaServer("openhrp", "", "stackOfTasks", "");

#if 0
  std::vector<std::string> klasses;
  std::cout << "factory:" << std::endl;
  dynamicgraph::g_factory.listEntities(klasses);
  for (unsigned int i=0; i<klasses.size(); i++){
    std::cout << i << ":" << klasses[i] << std::endl;
  }

  Entity *sot = g_factory.newEntity("SOT", "solver");
#endif
  Entity *entity;
  if (!g_pool.existEntity("robot_device", entity)){
    std::cerr << "can't find robot_device" << std::endl;
    return 1;
  }
  RobotSimu* robot_device = dynamic_cast<RobotSimu *>(entity);
  SignalBase<int> *signal;

#if 1
  Entity *robot_dynamic;
  if (!g_pool.existEntity("robot_dynamic", robot_dynamic)){
    std::cerr << "can't find robot_dynamic" << std::endl;
    return 1;
  }

  // reach(robot, 'left-wrist', 0.25, 0, 0.1)
  double tx = 0.25, ty = 0, tz = 0.1;
  signal = &robot_dynamic->getSignal("left-wrist");
  SignalTimeDependent< MatrixHomogeneous,int >* sigmh;
  sigmh = dynamic_cast< SignalTimeDependent< MatrixHomogeneous, int>* >(signal);
  if (!sigmh){
    std::cerr << "dynamic_cast failed(1)" << std::endl;
    return 1;
  }
  MatrixHomogeneous sdes = sigmh->accessCopy();
  sdes(0,3) += tx;
  sdes(1,3) += ty;
  sdes(2,3) += tz;

  Entity *feature;
  if (!g_pool.existEntity("robot_feature_left-wrist_ref", feature)){
    std::cerr << "can't find robot_feature_left-wrist_ref" << std::endl;
    return 1;
  }
  signal = &feature->getSignal("position");
  SignalPtr<MatrixHomogeneous,int>* reference;
  reference = dynamic_cast< SignalPtr< MatrixHomogeneous, int>* >(signal);
  if (!reference){
    std::cerr << "dynamic_cast failed(2)" << std::endl;
    return 1;
  }
  reference->setConstant(sdes);
  std::cout << "reference:" << std::endl;
  reference->get(std::cout);

  if (!g_pool.existEntity("robot_feature_left-wrist", feature)){
    std::cerr << "can't find robot_feature_left-wrist" << std::endl;
    return 1;
  }
  signal = &feature->getSignal("selec");
  SignalPtr<Flags,int>* selec;
  selec = dynamic_cast< SignalPtr< Flags, int>* >(signal);
  if (!selec){
    std::cerr << "dynamic_cast failed(3)" << std::endl;
    return 1;
  }
  Flags flags;
  char c = 0x07;
  flags.add(c);
  selec->setConstant(flags);
  std::cout << "selec:" << std::endl;
  selec->get(std::cout);

  if (!g_pool.existEntity("robot_task_left-wrist", feature)){
    std::cerr << "can't find robot_task_left-wrist" << std::endl;
    return 1;
  }
  signal = &feature->getSignal("controlGain");
  SignalPtr< double,int > *controlGain;
  controlGain = dynamic_cast< SignalPtr< double, int>* >(signal);
  if (!controlGain){
    std::cerr << "dynamic_cast failed(4)" << std::endl;
    return 1;
  }
  controlGain->setConstant(1.0);
  std::cout << "controlGain:" << std::endl;
  controlGain->get(std::cout);
#endif

  signal = &robot_device->getSignal("state");
  Signal< ml::Vector,int > *state;
  state = dynamic_cast< Signal< ml::Vector, int>* >(signal);
  if (!state){
    std::cerr << "dynamic_cast failed(5)" << std::endl;
    return 1;
  }
  ml::Vector v = state->accessCopy();
  std::cout << v << std::endl;

  /*
  sdes = toList(robot.dynamic.signal(op).value)
  sdes[0][3] += tx
  sdes[1][3] += ty
  sdes[2][3] += tz
  robot.features[op].reference.value = toTuple(sdes)
  robot.features[op]._feature.signal('selec').value = '000111'
  robot.tasks[op].signal('controlGain').value = 1.
  */

  for (int i=0; i<500; i++){
    robot_device->increment(0.005);
  }
  v = state->accessCopy();
  std::cout << v << std::endl;

  return 0;
}
