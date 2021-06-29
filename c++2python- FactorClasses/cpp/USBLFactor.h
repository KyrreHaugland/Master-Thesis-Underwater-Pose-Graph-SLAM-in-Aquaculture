

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactor.h>


//#include <gtsam/geometry/Pose3.h>
//#include <gtsam/linear/NoiseModel.h>

#include <vector>


namespace gtsam_absolute_factors {


class USBLFactor: public gtsam::NoiseModelFactor1<gtsam::Pose3> {

private:
  // measurement information
  double mx_, my_;

public:

  /**
   * Constructor
   * @param poseKey    associated pose varible key
   * @param model      noise model for GPS snesor, in X-Y
   * @param m          Point2 measurement
   */
  USBLFactor(gtsam::Key poseKey, const gtsam::Point2 m, gtsam::SharedNoiseModel model);

  // error function
  // @param p    the pose in Pose2
  // @param H    the optional Jacobian matrix, which use boost optional and has default null pointer
  gtsam::Vector evaluateError(const gtsam::Pose3& p, boost::optional<gtsam::Matrix&> H = boost::none) const; 

};

} // namespace gtsamexamples
