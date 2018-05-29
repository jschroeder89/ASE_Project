#include "userthread.hpp"

#include <global.hpp>

using namespace amiro;

extern Global global;

UserThread::UserThread() :
  chibios_rt::BaseStaticThread<USER_THREAD_STACK_SIZE>()
{
}

UserThread::~UserThread()
{
}

msg_t
UserThread::main()
{
  while (!this->shouldTerminate())
  {
    /*
     * INSERT CUSTOM CODE HERE
     */

	//chprintf((BaseSequentialStream*)&global.sercanmux1, "Hello Light Ring!\n");
    this->sleep(MS2ST(1000));
  }

  return RDY_OK;
}

