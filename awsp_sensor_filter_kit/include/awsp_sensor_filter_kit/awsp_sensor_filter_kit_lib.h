#include <vector>
#include <time.h>
#include <chrono>

#define ALPHA_WEIGHT 0.5

typedef unsigned int uint;
typedef unsigned long ulong_t;
typedef unsigned long long uulong_t;

enum feature_method {SMA = 1, EMA = 2, KALMAN = 3};
enum {ACCEL_X = 0, ACCEL_Y = 1, ACCEL_Z = 2, GYRO_X = 3, \
  GYRO_Y = 4, GYRO_Z = 5};

typedef struct WindowContainer {
  std::vector<float> window;
} WindowContainer;

class FilterKit
{
  public:
    FilterKit(uint sensor_num, const uint window_size);
    ~FilterKit();

    void window(float sensor_readings[], uint sensors[], uint method);
    std::vector<double> get_features();

  private:

    // Simple Moving Average (SMA)
    void sma_(std::vector<float> current_window);

    // Exponential moving average (EMA)
    void ema_(std::vector<float> current_window);
    float previous_ema_;

    uint window_size_;
    uint sensor_num_;
    std::vector<WindowContainer> windows_holder_;
    std::vector<double> features_;
};