#ifndef COUNTER_SERVER_H
#define COUNTER_SERVER_H

namespace slam {

class CounterServer {
public:
  static void Create() {
    if (m_instance_ptr == nullptr)
      m_instance_ptr = new CounterServer;
  }

  static int Get() { return m_cnt; }

  static void Increment() { m_cnt++; }

private:
  CounterServer() {}
  static CounterServer *m_instance_ptr;
  static int m_cnt;
};

} // namespace slam
#endif /* COUNTER_SERVER_H */
