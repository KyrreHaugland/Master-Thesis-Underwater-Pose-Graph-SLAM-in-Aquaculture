// gtsam-examples matlab wrapper declarations

// gtsam deceleration
class gtsam::Point2;
class gtsam::Point3;
class gtsam::Pose3;

class gtsam::GaussianFactorGraph;
class gtsam::Values;
virtual class gtsam::noiseModel::Base;
virtual class gtsam::NonlinearFactor;
virtual class gtsam::NonlinearFactorGraph;
virtual class gtsam::NoiseModelFactor;


namespace gtsam_absolute_factors {

// GPS factor for Pose2
#include <cpp/DepthFactor.h>
#include <cpp/TiltFactor.h>
#include <cpp/USBLFactor.h>


virtual class DepthFactor : gtsam::NoiseModelFactor {
  DepthFactor(size_t poseKey, const gtsam::Point3& m, gtsam::noiseModel::Base* model);
};

virtual class TiltFactor : gtsam::NoiseModelFactor {
  TiltFactor(size_t poseKey, const gtsam::Point3& m, gtsam::noiseModel::Base* model);
};

virtual class USBLFactor : gtsam::NoiseModelFactor {
  USBLFactor(size_t poseKey, const gtsam::Point2& m, gtsam::noiseModel::Base* model);
};

} // namespace gtsamexamples
