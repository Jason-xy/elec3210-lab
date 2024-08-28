/**
** Created by Zhijian QIAO.
** UAV Group, Hong Kong University of Science and Technology
** email: zqiaoac@connect.ust.hk
**/

#ifndef SRC_UTILS_H
#define SRC_UTILS_H

#include <chrono>

class TicToc {
 public:
  TicToc() { tic(); }

  void tic() { start = std::chrono::system_clock::now(); }

  double toc() {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    duration = elapsed_seconds.count();
    return duration;
  }

  double duration_ms() { return duration * 1000; }

 private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
  double duration;
};

#endif  // SRC_UTILS_H
