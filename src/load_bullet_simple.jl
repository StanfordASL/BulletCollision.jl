using Cxx
const bullet_lib_path = joinpath(ENV["BULLET3"], "src");
const bullet_objf_path = joinpath(ENV["BULLET3"], "build_cmake/src");
const bullet_include_paths = [bullet_lib_path, bullet_objf_path, dirname(@__FILE__())]

for p in bullet_include_paths
  addHeaderDir(p, kind=C_System)
end

include_files = ["BulletCollision/libBulletCollision.so", "LinearMath/libLinearMath.so", "BulletDynamics/libBulletDynamics.so"];
for fn in include_files
  Libdl.dlopen(joinpath(bullet_objf_path, fn), Libdl.RTLD_GLOBAL)
end

cxx"""
#include <iostream>
#include <sstream>
#include <fstream>
#include "btBulletCollisionCommon.h"
#include "BulletCollision/CollisionShapes/btConvex2dShape.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btPointCollector.h"
#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "LinearMath/btConvexHull.h"
#include <btBulletDynamicsCommon.h>

"""
