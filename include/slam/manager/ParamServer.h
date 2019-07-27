#ifndef PARAM_SERVER_H
#define PARAM_SERVER_H

#include <iostream>
#include <map>

namespace slam {

class ParamServer {
public:
  static void Create();
  static double Get(const std::string &key);
  static void Set(const std::string &key, double val);

private:
  ParamServer(){};
  static ParamServer *m_instance_ptr;
  static std::map<std::string, double> m_params;
};

} // namespace slam
#endif /* PARAM_SERVER_H */
