#ifndef ACTIVE_3D_PLANNING_VOXBLOX_INITIALIZATION_VOXBLOX_PACKAGE_H_
#define ACTIVE_3D_PLANNING_VOXBLOX_INITIALIZATION_VOXBLOX_PACKAGE_H_

namespace active_3d_planning {
namespace initialize {
// force the linker to include this lib
// TODO(schmluk): theres probably a better way to do this...
void voxblox_package();

}  // namespace initialize
}  // namespace active_3d_planning

#endif  // ACTIVE_3D_PLANNING_VOXBLOX_INITIALIZATION_VOXBLOX_PACKAGE_H_
