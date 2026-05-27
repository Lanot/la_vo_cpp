#pragma once

#include "ifeature_tracker.hpp"
#include "feature_tracker_orb.hpp"
#include "feature_tracker_sift.hpp"

class FeatureTrackerFactory
{
public:
    static std::shared_ptr<IFeatureTracker> create(const TrackerConfig& config);
};
