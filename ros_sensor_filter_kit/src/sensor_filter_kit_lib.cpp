#include "sensor_filter_kit/sensor_filter_kit_lib.h"

// ******************************** CONSTRUCTORS-DESTRUCTORS *******************************


FilterKit::FilterKit(uint sensor_num, const uint window_size)
{
  window_size_ = window_size;

  sensor_num_ = sensor_num;

  WindowContainer window_filler;

  for(int i = 0; i < sensor_num; i++)
  {
    windows_holder_.push_back(window_filler);
  }

  previous_ema_ = 0;
}

FilterKit::~FilterKit() {}

// **************************************** PUBLIC *****************************************

void FilterKit::window(float sensor_readings[], uint sensors[], uint method)
{
  std::vector<float>::iterator window_it;
  features_.clear();

  for(int i = 0; i < windows_holder_.size(); i++)
  {
    window_it = windows_holder_[i].window.begin();
    windows_holder_[i].window.insert(window_it, sensor_readings[sensors[i]]);
  }

  if (windows_holder_[0].window.size() == window_size_)
  {
    for(int i = 0; i < windows_holder_.size(); i++)
    {
      windows_holder_[i].window.pop_back();

      switch (method)
      {
        case SMA:
          sma_(windows_holder_[i].window);
          break;
        case EMA:
          sma_(windows_holder_[i].window);
          break;
        case KALMAN:
          sma_(windows_holder_[i].window);
          break;
      }
    }
  }
}

std::vector<double> FilterKit::get_features()
{
  std::vector<double> temp;

  if (features_.size() != 0)
  {
    return features_;
  }
  else
  {
    for (int i = 0; i < sensor_num_; ++i)
    {
      temp.push_back(0);
    }

    return temp;
  } 
}

// **************************************** PRIVATE ****************************************

void FilterKit::sma_(std::vector<float> current_window)
{
  float temp_window_value = 0;

  for(int i = 0; i < current_window.size(); i++)
  {
    temp_window_value += current_window[i];
  }

  temp_window_value = temp_window_value / current_window.size();

  features_.push_back(temp_window_value);
}

/*


*/

void FilterKit::ema_(std::vector<float> current_window)
{

  float new_ema = 0;
  new_ema = ALPHA_WEIGHT * current_window.front() + (1 - ALPHA_WEIGHT) * previous_ema_;

  features_.push_back(new_ema);
  previous_ema_ = new_ema;
}