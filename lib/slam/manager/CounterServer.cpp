#include <slam/manager/CounterServer.h>

namespace slam {

CounterServer *CounterServer::m_instance_ptr = nullptr;
int CounterServer::m_cnt = 0;

} // namespace slam
