#include <slam/manager/ParamServer.h>
#include <string>

namespace slam {

ParamServer *ParamServer::m_instance_ptr = nullptr;
std::map<std::string, double> ParamServer::m_params;

void ParamServer::Create() {
  if (m_instance_ptr == nullptr) {
    m_instance_ptr = new ParamServer;
  }
}

double ParamServer::Get(const std::string &key) {
  const auto iter = m_params.find(key);
  if (iter != m_params.end()) {
    return m_params[key];
  } else {
    std::cout << key << " did not found" << std::endl;
    return 0;
  }
}

void ParamServer::Set(const std::string &key, double val) {
  m_params[key] = val;
}

} // namespace slam
