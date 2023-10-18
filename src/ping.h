#ifndef MARINE_TOOLS_PING_H
#define MARINE_TOOLS_PING_H

#include "marine_acoustic_msgs/RawSonarImage.h"

namespace marine_tools
{

class Ping
{
public:
  Ping(const marine_acoustic_msgs::RawSonarImage& message, float bin_size = 0.0);

  ros::Time timestamp() const;
  
  /// Returns the binned samples
  const std::vector<float>& values() const;

  /// @brief Get the samples relative to a quadratic approximation of the background level.
  /// @return Vector of binned samples relative to the approximated background level.
  const std::vector<float>& valuesReBackground() const;

  float binSize() const;

private:
  ros::Time timestamp_;

  /// size in meters of sample bins
  float bin_size_;

  std::vector<float> values_;
  std::vector<float> values_re_background_;
};

} // namepsace layer_tracker

#endif
