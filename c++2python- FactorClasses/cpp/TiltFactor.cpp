/**
 * @file GPSPose2Factor.h
 * @brief 2D 'GPS' like factor for Pose2
 * @date Nov 8, 2016
 * @author Jing Dong
 */

/**
 * A simple 2D 'GPS' like factor
 * The factor contains a X-Y position measurement (mx, my) for a Pose2, but no rotation information
 * The error vector will be [x-mx, y-my]'
 */

#include "TiltFactor.h"

// you can custom namespace (if needed for your project)
namespace gtsam_absolute_factors {

TiltFactor::TiltFactor(gtsam::Key poseKey, const gtsam::Point3 m, gtsam::SharedNoiseModel model) :
  gtsam::NoiseModelFactor1<gtsam::Pose3>(model, poseKey), roll_(m.x()), pitch_(m.y()), yaw_(m.z()) {}

gtsam::Vector TiltFactor::evaluateError(const gtsam::Pose3& p, boost::optional<gtsam::Matrix&> H /*= boost::none*/) const {
  
  // note that use boost optional like a pointer
  // only calculate jacobian matrix when non-null pointer exists
  if (H) *H = (gtsam::Matrix36() << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,  
                                    0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 1.0, 0.0, 0.0, 0.0).finished();
    
  
  // return error vector
  return (gtsam::Vector3() << p.rotation().roll() - roll_, p.rotation().pitch() - pitch_, p.rotation().yaw() - yaw_).finished();
}




} // namespace gtsamexamples
