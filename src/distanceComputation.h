void compute_distance(const btCollisionShape* cs1, const btTransform& tr1,
                      const btCollisionShape* cs2, const btTransform& tr2,
                      btScalar* result, btScalar max_d2) {
  if (cs1->isConvex() && cs2->isConvex()) {
    btVoronoiSimplexSolver sGjkSimplexSolver;
    btGjkEpaPenetrationDepthSolver epa;
    sGjkSimplexSolver.setEqualVertexThreshold(0.f);
    btGjkPairDetector convexConvex((btConvexShape*)(cs1),
                                   (btConvexShape*)(cs2),
                                   &sGjkSimplexSolver, &epa);
    btGjkPairDetector::ClosestPointInput input;
    btPointCollector output;
    input.m_transformA = tr1;
    input.m_transformB = tr2;
    input.m_maximumDistanceSquared = max_d2;

    convexConvex.getClosestPoints(input, output, 0);
    if (output.m_hasResult) {
      result[0] = output.m_distance;    // negative means penetration
      // m_pointInWorld is point on shape cs1
      // m_normalOnBInWorld is n-hat from cs1 to cs2

      result[1] = output.m_pointInWorld.x() + output.m_distance*output.m_normalOnBInWorld.x();
      result[2] = output.m_pointInWorld.y() + output.m_distance*output.m_normalOnBInWorld.y();
      result[3] = output.m_pointInWorld.z() + output.m_distance*output.m_normalOnBInWorld.z();
      result[4] = output.m_pointInWorld.x();
      result[5] = output.m_pointInWorld.y();
      result[6] = output.m_pointInWorld.z();
    }
  } else if (cs1->isCompound()) {
    btCompoundShape* com1 = (btCompoundShape*)cs1;
    for (int i = 0; i < com1->getNumChildShapes(); i++) {
      compute_distance(com1->getChildShape(i), tr1 * com1->getChildTransform(i), cs2, tr2, result, max_d2);
    }
  } else if (cs2->isCompound()) {
    btCompoundShape* com2 = (btCompoundShape*)cs2;
    for (int i = 0; i < com2->getNumChildShapes(); i++) {
      compute_distance(cs1, tr1, com2->getChildShape(i), tr2 * com2->getChildTransform(i), result, max_d2);
    }
  } else {
      result[0] = -1;
  }
}

// void compute_weighted_distance(const btCollisionShape* cs1, const btTransform& tr1,
//                                const btCollisionShape* cs2, const btTransform& tr2,
//                                const btMatrix3x3& U, btScalar* result, btScalar max_d2)
// {
//   if (cs1->isConvex() && cs2->isConvex()) {
//     btVoronoiSimplexSolver sGjkSimplexSolver;
//     sGjkSimplexSolver.setEqualVertexThreshold(0.f);
//     btWeightedGjkPairDetector convexConvex((btConvexShape*)(cs1),
//                                            (btConvexShape*)(cs2),
//                                            &sGjkSimplexSolver, U);
//     btGjkPairDetector::ClosestPointInput input;
//     btPointCollector output;
//     input.m_transformA = tr1;
//     input.m_transformB = tr2;
//     input.m_maximumDistanceSquared = max_d2;
// 
//     convexConvex.getClosestPoints(input, output, 0);
//     if (output.m_hasResult) {
//       if (output.m_distance < result[0]) {
//         btVector3 Unormal = U * output.m_normalOnBInWorld;
//         btScalar lambda = output.m_distance / Unormal.length();
//         result[0] = output.m_distance;
//         result[1] = output.m_pointInWorld.x() + lambda*output.m_normalOnBInWorld.x();
//         result[2] = output.m_pointInWorld.y() + lambda*output.m_normalOnBInWorld.y();
//         result[3] = output.m_pointInWorld.z() + lambda*output.m_normalOnBInWorld.z();
//         result[4] = output.m_pointInWorld.x();
//         result[5] = output.m_pointInWorld.y();
//         result[6] = output.m_pointInWorld.z();
//       }
//     }
//   } else if (cs1->isCompound()) {
//     btCompoundShape* com1 = (btCompoundShape*)cs1;
//     for (int i = 0; i < com1->getNumChildShapes(); i++) {
//       compute_weighted_distance(com1->getChildShape(i), tr1 * com1->getChildTransform(i), cs2, tr2, U, result, max_d2);
//     }
//   } else if (cs2->isCompound()) {
//       btCompoundShape* com2 = (btCompoundShape*)cs2;
//       for (int i = 0; i < com2->getNumChildShapes(); i++) {
//         compute_weighted_distance(cs1, tr1, com2->getChildShape(i), tr2 * com2->getChildTransform(i), U, result, max_d2);
//       }
//   } else {
//       result[0] = -1;
//   }
// }
