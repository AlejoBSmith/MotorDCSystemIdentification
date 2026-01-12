#pragma once
#include <stddef.h> // size_t

class MovingMedianF {
public:
  static constexpr size_t MAX_N = 64;

  explicit MovingMedianF(size_t window = 1) { setWindow(window); }

  void begin() { reset(); }

  void setWindow(size_t window) {
    if (window < 1) window = 1;
    if (window > MAX_N) window = MAX_N;
    n = window;
    reset();
  }

  void reset() {
    idx = 0;
    count = 0;
    for (size_t i = 0; i < n; i++) buf[i] = 0.0f;
  }

  float reading(float x) {
    // Insert in ring buffer
    buf[idx] = x;
    idx = (idx + 1) % n;
    if (count < n) count++;

    // Copy current samples to temp and sort
    const size_t m = count;
    for (size_t i = 0; i < m; i++) tmp[i] = buf[i];

    insertionSort(tmp, m);

    // Median
    if (m & 1) {
      return tmp[m / 2];
    } else {
      // even: average of the two middle values
      return 0.5f * (tmp[m / 2 - 1] + tmp[m / 2]);
    }
  }

  size_t window()  const { return n; }
  size_t samples() const { return count; }

private:
  static void insertionSort(float* a, size_t m) {
    for (size_t i = 1; i < m; i++) {
      float key = a[i];
      size_t j = i;
      while (j > 0 && a[j - 1] > key) {
        a[j] = a[j - 1];
        j--;
      }
      a[j] = key;
    }
  }

  float  buf[MAX_N];
  float  tmp[MAX_N];   // temp workspace for sorting (keeps API simple)
  size_t n = 1;
  size_t idx = 0;
  size_t count = 0;
};
