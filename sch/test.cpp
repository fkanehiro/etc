#include <sys/time.h>
#include <sch/CD/CD_Pair.h>
#include <sch/STP-BV/STP_BV_P.h>

using namespace sch;

int main(int argc, char *argv[])
{
  if (argc < 3){
    std::cerr << "Usage:" << argv[0] << "[sch file1] [sch file2]" << std::endl;
    return 1;
  }
  STP_BV_P obj1, obj2;
  obj1.constructFromFile(argv[1]);
  obj2.constructFromFile(argv[2]);

  CD_Pair pair(&obj1, &obj2);
  
  obj1.setPosition(0,0,0);

  struct timeval tm1, tm2;
  Point3 p1,p2;
  for (int i=0; i<20; i++){
    obj1.setPosition(0.5+i*0.05,0,0);
    gettimeofday(&tm1, NULL);
    Scalar d = sqrt(pair.getDistance());
    pair.getClosestPoints(p1,p2);
    gettimeofday(&tm2, NULL);
    std::cout << "tm = " << ((tm2.tv_sec - tm1.tv_sec)*1e3 + (tm2.tv_usec - tm1.tv_usec)/1e3) << "[ms], d = " << d << std::endl;
  }

  return 0;
}
