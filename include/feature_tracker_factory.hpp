#pragma once

#include "config.hpp"
#include "ifeature_tracker.hpp"

#include "feature_tracker_orb.hpp"
#include "feature_tracker_sift.hpp"
#include "feature_tracker_super_point.hpp"

class FeatureTrackerFactory
{
public:
    static std::unique_ptr<IFeatureTracker> create(const TrackerConfig& config);
};
