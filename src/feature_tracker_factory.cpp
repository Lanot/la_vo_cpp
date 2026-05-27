#include "feature_tracker_factory.hpp"

std::shared_ptr<IFeatureTracker> FeatureTrackerFactory::create(const TrackerConfig& config)
{
   if (config.feature_type == FeatureType::SIFT)
   {
      return std::make_shared<FeatureTrackerSIFT>(config);
   }
   else
   {
      return std::make_shared<FeatureTrackerORB>(config);
   }
}
