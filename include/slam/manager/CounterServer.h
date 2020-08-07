#ifndef COUNTER_SERVER_H
#define COUNTER_SERVER_H

namespace slam {

class CounterServer {
private:
  CounterServer() {}
  static CounterServer *m_instance_ptr;
  static int m_cnt;

public:
  static void Create() {
    if (m_instance_ptr == nullptr) {
      m_instance_ptr = new CounterServer;
      m_cnt = 0;
    }
  }

  static int Get() { return m_cnt; }

  static void Increment() { m_cnt++; }
};

} // namespace slam
#endif /* COUNTER_SERVER_H */
