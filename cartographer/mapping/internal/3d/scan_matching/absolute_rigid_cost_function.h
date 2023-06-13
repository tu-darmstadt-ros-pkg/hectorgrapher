#ifndef CARTOGRAPHER_MAPPING_3D_ABSOLUTE_RIGID_COST_FUNCTION_H_
#define CARTOGRAPHER_MAPPING_3D_ABSOLUTE_RIGID_COST_FUNCTION_H_

#include "Eigen/Core"
#include "Eigen/Geometry"

namespace cartographer {
namespace mapping {

class AbsoluteRigidCostFunction {
 public:
  AbsoluteRigidCostFunction(const double translation_scaling_factor,
                            const double rotation_scaling_factor,
                            const transform::Rigid3d& reference)
      : translation_scaling_factor_(translation_scaling_factor),
        rotation_scaling_factor_(rotation_scaling_factor),
        reference_(reference) {}

  AbsoluteRigidCostFunction(const AbsoluteRigidCostFunction&) = delete;
  AbsoluteRigidCostFunction& operator=(const AbsoluteRigidCostFunction&) =
      delete;

  template <typename T>
  bool operator()(const T* const estimate_translation,
                  const T* const estimate_rotation, T* residual) const {
    const transform::Rigid3<T> estimate(
        Eigen::Map<const Eigen::Matrix<T, 3, 1>>(estimate_translation),
        Eigen::Quaternion<T>(estimate_rotation[0], estimate_rotation[1],
                             estimate_rotation[2], estimate_rotation[3]));
    const transform::Rigid3<T> error =
        reference_.inverse().cast<T>() * estimate;
    residual[0] = translation_scaling_factor_ * error.translation().x();
    residual[1] = translation_scaling_factor_ * error.translation().y();
    residual[2] = translation_scaling_factor_ * error.translation().z();
    //    residual[3] = rotation_scaling_factor_ * transform::GetYaw(error);
    residual[3] = rotation_scaling_factor_ * transform::GetRoll(error);
    residual[4] = rotation_scaling_factor_ * transform::GetPitch(error);
    residual[5] = rotation_scaling_factor_ * transform::GetYaw(error);
    return true;
  }

 private:
  const double translation_scaling_factor_;
  const double rotation_scaling_factor_;
  const transform::Rigid3d reference_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_ABSOLUTE_RIGID_COST_FUNCTION_H_