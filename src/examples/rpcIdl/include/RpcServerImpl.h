#include <IRpcServer.h>


class RpcServerImpl : public IRpcServer {
  int answer;
  bool running;

public:
  RpcServerImpl();

  virtual int32_t get_answer();
  virtual bool set_answer(const int32_t rightAnswer);
  virtual int32_t add_int(const int32_t x);
  virtual bool start();
  virtual bool stop();
  virtual bool is_running();
};



