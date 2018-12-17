__precompile__(false)
module BulletCollision

using Cxx
using GeometryTypes
using StaticArrays
using Libdl
using LinearAlgebra

include("load_bullet.jl")

### type definitions
const BulletCollisionObjectPtr  = @cxxt_str "btCollisionObject*"
const BulletCollisionShapePtr   = @cxxt_str "btCollisionShape*"
const BulletConvexShapePtr      = @cxxt_str "btConvexShape*"
const BulletCompoundShapePtr    = @cxxt_str "btCompoundShape*"
const BulletCollisionWorldPtr   = @cxxt_str "btCollisionWorld*"
const BulletTransformPtr        = @cxxt_str "btTransform*"
const BulletTransformValue      = @cxxt_str "btTransform"
const BulletScalar              = @cxxt_str "btScalar"
const BulletMultiBodyPtr        = @cxxt_str "btMultiBody*"
const BulletInf = icxx"""BT_LARGE_FLOAT;""";

set_margin(obj::BulletCollisionObjectPtr, dx = 0.) = set_margin(get_collision_shape(obj), dx)

function set_margin(cs::BulletCollisionShapePtr, dx = 0.)
  if icxx"""$cs->isConvex();"""
    icxx"""$cs->setMargin($dx);"""
  elseif icxx"""$cs->isCompound();"""
    cs = icxx"""btCompoundShape* cm = (btCompoundShape*)$cs; cm;"""
    num_children = icxx"""$cs->getNumChildShapes();"""
    for i in 0:num_children-1
      set_margin(icxx"""$cs->getChildShape($i);""", dx)
    end
  end
end

get_collision_shape(obj::BulletCollisionObjectPtr) = icxx"""$obj->getCollisionShape();"""
get_world_transform(obj::BulletCollisionObjectPtr) = icxx"""btTransform tr = $obj->getWorldTransform(); tr;"""
identity_transform() = icxx"""btTransform tr; tr.setIdentity(); tr;"""

### primitive shapes
function box(lo::AbstractVector, hi::AbstractVector)
  halfextents = (hi - lo)/2
  mid = (hi + lo)/2
  icxx"""
  btCollisionObject* box = new btCollisionObject();
  btBoxShape* box_shape = new btBoxShape(btVector3($(halfextents[1]), $(halfextents[2]), $(halfextents[3])));
  box_shape->setMargin(0.);
  box->getWorldTransform().setOrigin(btVector3($(mid[1]), $(mid[2]), $(mid[3])));
  box->setCollisionShape(box_shape);
  box;
  """
end

function convex_hull(pts::Vector{V}) where V <: AbstractVector
  convex_hull = icxx"""
  btCollisionObject* convex_hull = new btCollisionObject();
  btConvexHullShape* convex_hull_shape = new btConvexHullShape();
  convex_hull_shape->setMargin(0.);
  convex_hull->getWorldTransform().setIdentity();
  convex_hull->setCollisionShape(convex_hull_shape);
  convex_hull;
  """
  for v in pts
    icxx"""((btConvexHullShape*)($convex_hull->getCollisionShape()))->addPoint(btVector3($(v[1]), $(v[2]), $(v[3])));"""
  end
  convex_hull
end

convex_hull(pts::V...) where V <: AbstractVector = convex_hull(collect(pts))
geometry_type_to_BT(shape::HomogenousMesh) = convex_hull(shape.vertices)
convex_hull_box(lo::AbstractVector, hi::AbstractVector) = convex_hull([SVector{3}([lo hi][[1,2,3] + 3*[i,j,k]]) for i=0:1, j=0:1, k=0:1]...)

function geometry_type_to_BT(shape::HyperRectangle,hull::Bool=true)
  if hull
    return convex_hull_box(SVector{3}(shape.origin),shape.origin+shape.widths)
  end
  return box(SVector{3}(shape.origin),shape.origin+shape.widths)
end

function convex_hull_cylinder(p1::AbstractVector, p2::AbstractVector, r::AbstractFloat, n::Int=25)
  Q, _ = LinearAlgebra.qr(reshape(p2-p1,(3,1)))
  #Q, _ = qr((p2-p1)'', thin=false) # Ed's code
  d1, d2 = Q[:,2], Q[:,3]
  convex_hull((p1 + r*cos(2*pi*i/n)*d1 + r*sin(2*pi*i/n)*d2 for i in 1:n)...,
              (p2 + r*cos(2*pi*i/n)*d1 + r*sin(2*pi*i/n)*d2 for i in 1:n)...)
end

geometry_type_to_BT(shape::Cylinder) = convex_hull_cylinder(SVector{3}(shape.origin), SVector{3}(shape.origin+shape.extremity), shape.r)

function sphere(c::AbstractVector, r::AbstractFloat)
  icxx"""
  btCollisionObject* sphere = new btCollisionObject();
  btSphereShape* sphere_shape = new btSphereShape($r);
  sphere_shape->setMargin(0.);
  sphere->getWorldTransform().setOrigin(btVector3($(c[1]), $(c[2]), $(c[3])));
  sphere->setCollisionShape(sphere_shape);
  sphere;
  """
end

geometry_type_to_BT(shape::HyperSphere) = sphere(SVector{3}(shape.center),shape.r)

### concave shape made out of convex sub parts called child shapes
### each child shape has its own local offset transform relative to bBulletCompoundShapePtrtCompoundShape
function compound_collision_object(objs::Vector{BulletCollisionObjectPtr})
  compound = icxx"""
  btCollisionObject* compound = new btCollisionObject();
  btCompoundShape* compound_shape = new btCompoundShape(true, $(length(objs)));
  compound_shape->setMargin(0.);
  compound->getWorldTransform().setIdentity();
  compound->setCollisionShape(compound_shape);
  compound;
  """
  for o in objs
    icxx"""((btCompoundShape*)($compound->getCollisionShape()))->addChildShape($o->getWorldTransform(),
                                                                                 $o->getCollisionShape());"""
  end
  icxx"""((btCompoundShape*)($compound->getCollisionShape()))->recalculateLocalAabb();"""
  compound
end

compound_collision_object(objs::BulletCollisionObjectPtr...) = compound_collision_object(collect(objs))


function make_2d(co::BulletCollisionObjectPtr)
  icxx"""
  btConvex2dShape* shape2d = new btConvex2dShape((btConvexShape*)($co->getCollisionShape()));
  shape2d->setMargin(0.);
  $co->setCollisionShape(shape2d);
  """
  co
end

#### distance between two btCollisionShape's (including compound shapes)
function distance(co1::BulletCollisionObjectPtr, co2::BulletCollisionObjectPtr, max_d2 = BulletInf)
    result_ptr = icxx"""
    btScalar* result = new btScalar[7];
    result[0] = btScalar(BT_LARGE_FLOAT);
    compute_distance($co1->getCollisionShape(), $co1->getWorldTransform(),
                     $co2->getCollisionShape(), $co2->getWorldTransform(),
                     result, $max_d2);
    result;
    """
    result = unsafe_load(convert(Ptr{SVector{7,Float32}}, result_ptr))
    icxx"""delete[] $result_ptr;"""
    result[1], result[2:4], result[5:7] # signed distance, point on co1, point on co2
end

### stores all btCollisionObjects and provides an interface to perform queries
function collision_world(lo::AbstractVector, hi::AbstractVector, maxobjects = 100)
  icxx"""
  btCollisionConfiguration* collision_configuration = new btDefaultCollisionConfiguration();
  btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collision_configuration);
  
  btVector3 worldAabbMin($(lo[1]), $(lo[2]), $(lo[3]));
  btVector3 worldAabbMax($(hi[1]), $(hi[2]), $(hi[3]));
  unsigned int max_objects = $maxobjects;
  
  btBroadphaseInterface* broadphase = new btAxisSweep3(worldAabbMin, worldAabbMax, max_objects, 0, false);
  btCollisionWorld* cw = new btCollisionWorld(dispatcher, broadphase, collision_configuration);
  cw;
  """
end

get_convex_components(co::BulletCollisionObjectPtr) = get_convex_components(get_collision_shape(co), get_world_transform(co))

function get_convex_components(cs::BulletCollisionShapePtr, tr::BulletTransformValue = identity_transform())
  if icxx"""$cs->isConvex();"""
    [icxx"""btCollisionObject* cc = new btCollisionObject(); cc->setCollisionShape($cs); cc->setWorldTransform($tr); cc;"""]
  elseif icxx"""$cs->isCompound();"""
    cs = icxx"""btCompoundShape* cm = (btCompoundShape*)$cs; cm;"""
    num_children = icxx"""$cs->getNumChildShapes();"""
    vcat((get_convex_components(icxx"""$cs->getChildShape($i);""",
                                icxx"""btTransform comp = $tr * $cs->getChildTransform($i); comp;""")
          for i in 0:num_children-1)...)
  end
end

mutable struct BulletStaticEnvironment
  robot::BulletCollisionObjectPtr
  convex_robot_components::Vector{BulletCollisionObjectPtr}    # when robot's world transform is the identity; TODO(acauligi): set_transformation is unsafe
  environment::BulletCollisionWorldPtr
  convex_env_components::Vector{BulletCollisionObjectPtr}
  count::Int
end

function BulletStaticEnvironment(r::BulletCollisionObjectPtr, e::BulletCollisionWorldPtr)
  num_co = icxx"""$e->getNumCollisionObjects();"""
  e_components = vcat((get_convex_components(icxx"""(btCollisionObject *)($e->getCollisionObjectArray()[$i]);""") for i in 0:num_co-1)...)
  BulletStaticEnvironment(r, get_convex_components(get_collision_shape(r)), e, e_components, 0)
end

add_collision_object!(cw::BulletCollisionWorldPtr, co::BulletCollisionObjectPtr) =
    icxx"""$cw->addCollisionObject($co); $cw->updateAabbs();"""

function add_collision_object!(CC::BulletStaticEnvironment, co::BulletCollisionObjectPtr)
  add_collision_object!(CC.environment, co)
  append!(CC.convex_env_components, get_convex_components(co))
end

set_margin(CC::BulletStaticEnvironment, dx = 0.) = set_margin(CC.robot, dx)

function set_transformation(o::BulletCollisionObjectPtr, v::SVector{3})
  icxx"""$o->getWorldTransform().setOrigin(btVector3($(v[1]), $(v[2]), $(v[3])));"""
end

function set_transformation(o::BulletCollisionObjectPtr, v::SVector{4})
  icxx"""$o->getWorldTransform().setRotation(btQuaternion($(v[1]), $(v[2]), $(v[3]), $(v[4])));"""
end

function set_transformation(o::BulletCollisionObjectPtr, v::SVector{7})
  icxx"""
  $o->getWorldTransform().setOrigin(btVector3($(v[1]), $(v[2]), $(v[3])));
  $o->getWorldTransform().setRotation(btQuaternion($(v[4]), $(v[5]), $(v[6]), $(v[7])));
  """
end
set_transformation(o::BulletCollisionObjectPtr, v::AbstractVector) = set_transformation(o, SVector{length(v)}(v))

function is_free_state(v::SVector, CC::BulletStaticEnvironment)
  set_transformation(CC.robot, v)
  icxx"""
  BinaryCollisionCallback result;
  $(CC.environment)->contactTest($(CC.robot), result);
  !result.is_collision;
  """
end

function is_free_state(CC::BulletStaticEnvironment)
  icxx"""
  $(CC.environment)->performDiscreteCollisionDetection();
  int numManifolds = $(CC.environment)->getDispatcher()->getNumManifolds();
  bool is_free_state = true; 
  //For each contact manifold
  for (int i = 0; i < numManifolds; i++) {
    btPersistentManifold* contactManifold = $(CC.environment)->getDispatcher()->getManifoldByIndexInternal(i);
    const btCollisionObject* obA = (contactManifold->getBody0());
    const btCollisionObject* obB = (contactManifold->getBody1());
    contactManifold->refreshContactPoints(obA->getWorldTransform(), obB->getWorldTransform());
    int numContacts = contactManifold->getNumContacts();
    if (numContacts > 0) {
      is_free_state = false;
      break;
    }
    ////For each contact point in that manifold
    //for (int j = 0; j < numContacts; j++) {
    //  //Get the contact information
    //    btManifoldPoint& pt = contactManifold->getContactPoint(j);
    //    btVector3 ptA = pt.getPositionWorldOnA();
    //    btVector3 ptB = pt.getPositionWorldOnB();
    //    double ptdist = pt.getDistance();
    //}
  }
  is_free_state;
  """
end

function is_free_motion(v::SVector{3}, w::SVector{3}, CC::BulletStaticEnvironment)
  CC.count += 1
  icxx"""
  bool free_motion = true;
  btCompoundShape* robot_compound = (btCompoundShape*) ($(CC.robot)->getCollisionShape());
  btTransform v, w;
  v.setIdentity();
  w.setIdentity();
  v.setOrigin(btVector3($(v[1]), $(v[2]), $(v[3])));
  w.setOrigin(btVector3($(w[1]), $(w[2]), $(w[3])));
  for (int i = 0; i < robot_compound->getNumChildShapes(); i++) {
    BinarySweptConvexCollisionCallback result;
    btConvexShape* robot_piece = (btConvexShape*) (robot_compound->getChildShape(i));
    $(CC.environment)->convexSweepTest(robot_piece, v * robot_compound->getChildTransform(i), w * robot_compound->getChildTransform(i), result);
    if (result.is_collision) {
      free_motion = false;
      break;
    }
  }
  free_motion;
  """
end

function is_free_motion(v::SVector{6}, w::SVector{6}, CC::BulletStaticEnvironment)
  CC.count += 1
  icxx"""
  bool free_motion = true;
  btCompoundShape* robot_compound = (btCompoundShape*) ($(CC.robot)->getCollisionShape());
  btTransform v, w;
  btQuaternion vrot, wrot;
  vrot.setEulerZYX($(v[4]), $(v[5]), $(v[6]));
  wrot.setEulerZYX($(w[4]), $(w[5]), $(w[6]));
  v.setOrigin(btVector3($(v[1]), $(v[2]), $(v[3])));
  w.setOrigin(btVector3($(w[1]), $(w[2]), $(w[3])));
  v.setRotation(vrot);
  w.setRotation(wrot);
  for (int i = 0; i < robot_compound->getNumChildShapes(); i++) {
    BinarySweptConvexCollisionCallback result;
    btConvexShape* robot_piece = (btConvexShape*) (robot_compound->getChildShape(i));
    $(CC.environment)->convexSweepTest(robot_piece, v * robot_compound->getChildTransform(i), w * robot_compound->getChildTransform(i), result);
    if (result.is_collision) {
      free_motion = false;
      break;
    }
  }
  free_motion;
  """
end

is_free_motion(v::AbstractVector, w::AbstractVector, CC::BulletStaticEnvironment) = is_free_motion(SVector(v), SVector(w), CC)

function distance(CC::BulletStaticEnvironment, ri::Int, tr::AbstractVector, ei::Int)
  rc, ec = CC.convex_robot_components[ri], CC.convex_env_components[ei]
  set_transformation(rc, tr)
  distance(rc, ec)
end

function distance_gradient_finite_difference(CC::BulletStaticEnvironment, ri::Int, tr::SVector{3,T}, ei::Int, dx::T = T(.01)) where T
    trm = MVector(tr)
    function gradi(i)
        tri = trm[i]
        trm[i] = tri + dx; fp = distance(CC, ri, trm, ei)[1]
        trm[i] = tri - dx; fm = distance(CC, ri, trm, ei)[1]
        trm[i] = tri
        (fp - fm) / (dx + dx)
    end
    SVector{3}([gradi(i) for i in 1:3])
end

function pairwise_convex_convex_distances(v::AbstractVector{T}, CC::BulletStaticEnvironment, threshold = T(Inf)) where T
  foreach(rc -> set_transformation(rc, v), CC.convex_robot_components)
  pairwise_iter = ((ri, ei, distance(rc, ec)) for (ri, rc) in enumerate(CC.convex_robot_components),
                                                  (ei, ec) in enumerate(CC.convex_env_components))
  collect(filter(x -> x[3][1] < threshold, pairwise_iter))    # note: filter must be collected now because of set_transformation
end

function pairwise_convex_convex_distances(v::AbstractVector{T}, CC::BulletStaticEnvironment, W::AbstractMatrix{T}, threshold = T(Inf)) where T
  foreach(rc -> set_transformation(rc, v), CC.convex_robot_components)
  pairwise_iter = ((ri, ei, distance(rc, ec, W)) for (ri, rc) in enumerate(CC.convex_robot_components),
                                                     (ei, ec) in enumerate(CC.convex_env_components))
  collect(filter(x -> x[3][1] < threshold, pairwise_iter))    # note: filter must be collected now because of set_transformation
end

end
