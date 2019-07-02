#include <slam/manager/ParamServer.h>

namespace slam {

ParamServer *ParamServer::m_instance_ptr = nullptr;
std::map<std::string, double> ParamServer::m_params;

} // namespace slam
