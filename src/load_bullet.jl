const bullet_lib_path = joinpath(ENV["BULLET3"], "src");
const bullet_objf_path = joinpath(ENV["BULLET3"], "bullet-build/src");
const bullet_include_paths = [bullet_lib_path, bullet_objf_path, dirname(@__FILE__())]  # TODO(tzahroof): add the header and cpp(? maybe .o? Maybe look into this) files

for p in bullet_include_paths
  addHeaderDir(p, kind=C_System)
end

# add *.so files from /usr/local/lib 
include_files = ["libBulletCollision.so","libLinearMath.so","libBulletDynamics.so"];
so_hdr = "/usr/lib/x86_64-linux-gnu"
for fn in include_files
  Libdl.dlopen(joinpath(so_hdr,fn), Libdl.RTLD_GLOBAL);
end

cxx"""
#include <iostream>
#include <sstream>
#include "btBulletCollisionCommon.h"
#include "BulletCollision/CollisionShapes/btConvex2dShape.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btPointCollector.h"
#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "distanceComputation.h"
#include "LinearMath/btConvexHull.h"

#ifndef BT_BINARY_COLLISION_CALLBACKS
#define BT_BINARY_COLLISION_CALLBACKS

struct BinaryCollisionCallback : public btCollisionWorld::ContactResultCallback
{
  bool is_collision;

  BinaryCollisionCallback() : btCollisionWorld::ContactResultCallback(), is_collision(false) {}

  virtual btScalar addSingleResult(btManifoldPoint& cp,
    const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0,
    const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1) {
    is_collision = true;
    return 0;
  }
};

struct BinarySweptConvexCollisionCallback : public btCollisionWorld::ConvexResultCallback
{
  bool is_collision;

  BinarySweptConvexCollisionCallback() : btCollisionWorld::ConvexResultCallback(), is_collision(false) {}

  virtual btScalar addSingleResult(btCollisionWorld::LocalConvexResult& convexResult, bool normalInWorldSpace) {
    is_collision = true;
    return 0;
  }
};
#endif // BT_BINARY_COLLISION_CALLBACKS
"""
