#ifndef PARAM_SERVER_H
#define PARAM_SERVER_H

#include <map>

namespace slam {

class ParamServer {
public:
  static void Create() {
    if (m_instance_ptr == nullptr)
      m_instance_ptr = new ParamServer;
  }

  static double Get(const std::string &key) { return m_params[key]; }

  static void Set(const std::string &key, double val) { m_params[key] = val; }

private:
  ParamServer(){};
  static ParamServer *m_instance_ptr;
  static std::map<std::string, double> m_params;
};

} // namespace slam
#endif /* PARAM_SERVER_H */
