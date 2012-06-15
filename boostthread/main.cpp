#include <boost/thread.hpp>
#include <boost/bind.hpp>

class Object
{
public:
  int hello(){
    loop_flag = true; 
    int cnt = 0;
    while(loop_flag){
      std::cout << "hello" << std::endl;
      sleep(1);
      cnt++;
    }
    return cnt;
  }
  void stop() {
    loop_flag = false;
  }
  bool loop_flag;
};

int main(void)
{
  Object obj;
  boost::packaged_task<int> pt(boost::bind(&Object::hello, &obj));
  boost::unique_future<int> f = pt.get_future();

  std::cout << "To stop the thread, hit return key" << std::endl;
  boost::thread thr(boost::ref(pt));
  getchar();
  obj.stop();
  std::cout << "ret = " << f.get() << std::endl;

  return 0;
}
