#include <iostream>
#include <stdlib.h>
#include <unistd.h>

#include <slam/geometry/Scan2D.h>
#include <slam/io/SensorDataReader.h>
#include <slam/io/SlamLauncher.h>

int main(int argc, char **argv) {
  bool scanCheck = false;    // only display the scan if true
  bool odometryOnly = false; // use raw odometry if true
  char *filename;
  int startScanIndex = 0;

  if (argc < 2) {
    std::cerr << "Usage: ./SLAM (-{so}) <relative-path-to-filename> "
                 "(<startScanIndex>)"
              << std::endl;
    return 1;
  }

  // process the arguments
  int idx = 2;
  if (argv[1][0] == '-') {
    for (int i = 1;; ++i) {
      char option = argv[1][i];
      if (!option) {
        break;
      } else if (option == 's') {
        scanCheck = true;
      } else if (option == 'o') {
        odometryOnly = true;
      }
    }
    if (argc == 2) {
      std::cerr << "No file specified" << std::endl;
      return 1;
    }
    idx++;
  }
  if (argc < idx) {
    std::cerr << "Too few arguments" << std::endl;
    return 1;
  }
  filename = argv[idx - 1];
  if (argc == idx + 1) {
    startScanIndex = atoi(argv[idx]);
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
  sl.CustomizeFrameWork();
  sl.Run();
  std::cout << "finished completely" << std::endl;
}
