#ifndef SC_PGO__LOOP_CONSTRAINT_HPP_
#define SC_PGO__LOOP_CONSTRAINT_HPP_

#include <gtsam/geometry/Pose3.h>

namespace sc_pgo
{

// ICP aligns a source cloud already expressed in the global frame to a target
// cloud in that same frame.  Its result is therefore a *global correction*,
// not a previous-to-current BetweenFactor measurement by itself.
inline gtsam::Pose3 correctedRelativePose(
  const gtsam::Pose3 & previous_global_pose,
  const gtsam::Pose3 & current_global_pose,
  const gtsam::Pose3 & global_icp_correction)
{
  const gtsam::Pose3 corrected_current =
    global_icp_correction.compose(current_global_pose);
  return previous_global_pose.between(corrected_current);
}

}  // namespace sc_pgo

#endif  // SC_PGO__LOOP_CONSTRAINT_HPP_
