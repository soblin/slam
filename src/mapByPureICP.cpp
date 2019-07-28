#include <iostream>
#include <stdlib.h>
#include <unistd.h>

#include <slam/geometry/Scan2D.h>
#include <slam/io/SensorDataReader.h>
#include <slam/manager/SlamLauncher.h>

int main(int argc, char **argv) {
  bool scanCheck = false;    // only display the scan if true
  bool odometryOnly = false; // use raw odometry if true
  char *filename;
  std::string customize;

  if (argc < 2) {
    std::cerr << "Usage: ./SLAM <relative-path-to-filename>"
                 "custom[A-G]"
              << std::endl;
    return 1;
  }

  // process the arguments
  int idx = 2;
  if (argc < idx) {
    std::cerr << "Too few arguments" << std::endl;
    return 1;
  }
  filename = argv[idx - 1];

  if (argc == idx + 1) {
    customize = std::string(argv[idx]);
  }

  char buf[256] = {};

  readlink("/proc/self/exe", buf, sizeof(buf));
  std::string str(buf);
  int pos = str.find_last_of('/');
  std::string curr_dir = str.substr(0, pos);
  std::string filepath = curr_dir + '/' + filename;

  slam::SlamLauncher sl;
  sl.SetOdometryOnly(false);
  if (!sl.SetFilename(filepath)) {
    std::cout << "failed to open " << filepath << std::endl;
  }
  sl.CustomizeFrameWork(customize);
  sl.Initialize();
  sl.Run();
  std::cout << "finished completely" << std::endl;
}
