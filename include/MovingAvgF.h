#pragma once
#include <stddef.h> // size_t

class MovingAvgF {
public:
  static constexpr size_t MAX_N = 64;

  explicit MovingAvgF(size_t window = 1) { setWindow(window); }

  // Optional: keep same "begin()" style as movingAvg lib
  void begin() { reset(); }

  // Change window size (clamped to 1..MAX_N)
  void setWindow(size_t window) {
    if (window < 1) window = 1;
    if (window > MAX_N) window = MAX_N;
    n = window;
    reset();
  }

  void reset() {
    idx = 0;
    count = 0;
    sum = 0.0f;
    for (size_t i = 0; i < n; i++) buf[i] = 0.0f;
  }

  float reading(float x) {
    if (count < n) {
      buf[idx] = x;
      sum += x;
      idx = (idx + 1) % n;
      count++;
      return sum / (float)count;  // ramp-up behavior
    } else {
      sum -= buf[idx];
      buf[idx] = x;
      sum += x;
      idx = (idx + 1) % n;
      return sum / (float)n;
    }
  }

  size_t window()  const { return n; }
  size_t samples() const { return count; }

private:
  float  buf[MAX_N];
  size_t n = 1;
  size_t idx = 0;
  size_t count = 0;
  float  sum = 0.0f;
};
