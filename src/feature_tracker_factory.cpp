#include "feature_tracker_factory.hpp"

std::unique_ptr<IFeatureTracker> FeatureTrackerFactory::create(const TrackerConfig& config)
{
   if (config.feature_type == FeatureType::SIFT)
   {
      return std::make_unique<FeatureTrackerSIFT>(config);
   }
   else
   {
      return std::make_unique<FeatureTrackerORB>(config);
   }
}
