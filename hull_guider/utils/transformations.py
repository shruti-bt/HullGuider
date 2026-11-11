import numpy as np


# Or matrix form (works with (N,3) or (...,3)):
T_RDF = np.array([[1, 0, 0],  # x unchanged, x is right.
                  [0, 0, 1],  # new y from old z
                  [0,-1, 0]], dtype=float) # new z from old -y 

def position_transform_to_Replica(P_cam: np.ndarray, source) -> np.ndarray:
    if source == 'RDF': # right down forward, OpenCV Image camera convention
        return P_cam @ T_RDF.T
    else:
        print(f"Error! Transform source not found: {source} . No transform applied.")
        return P_cam


def make_T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Assemble a 4x4 SE(3) from 3x3 R and 3 t."""
    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = np.asarray(R, dtype=np.float32)
    T[:3,  3] = np.asarray(t, dtype=np.float32)
    return T

def normalize_quat_wxyz(q_wxyz) -> np.ndarray:
    """Normalize quaternion (w,x,y,z)."""
    q = np.asarray(q_wxyz, dtype=np.float64)
    n = np.linalg.norm(q)
    return (q / (n + 1e-12)).astype(np.float32)

def quat_xyzw_to_R(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """
    ROS-style quaternion (x,y,z,w) -> rotation matrix.
    Interpreted as camera->world (child->parent) orientation.
    """
    n = np.sqrt(qw*qw + qx*qx + qy*qy + qz*qz) + 1e-12
    qw, qx, qy, qz = qw/n, qx/n, qy/n, qz/n
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz
    R = np.array([
        [1 - 2*(yy + zz), 2*(xy - wz),     2*(xz + wy)],
        [2*(xy + wz),     1 - 2*(xx + zz), 2*(yz - wx)],
        [2*(xz - wy),     2*(yz + wx),     1 - 2*(xx + yy)]
    ], dtype=np.float32)
    return R

def rotmat_to_quat_wxyz(R: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix -> quaternion (w,x,y,z)."""
    R = np.asarray(R, dtype=np.float64)
    tr = R[0,0] + R[1,1] + R[2,2]
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * S
        x = (R[2,1] - R[1,2]) / S
        y = (R[0,2] - R[2,0]) / S
        z = (R[1,0] - R[0,1]) / S
    else:
        i = int(np.argmax([R[0,0], R[1,1], R[2,2]]))
        if i == 0:
            S = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2.0
            w = (R[2,1] - R[1,2]) / S
            x = 0.25 * S
            y = (R[0,1] + R[1,0]) / S
            z = (R[0,2] + R[2,0]) / S
        elif i == 1:
            S = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2.0
            w = (R[0,2] - R[2,0]) / S
            x = (R[0,1] + R[1,0]) / S
            y = 0.25 * S
            z = (R[1,2] + R[2,1]) / S
        else:
            S = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2.0
            w = (R[1,0] - R[0,1]) / S
            x = (R[0,2] + R[2,0]) / S
            y = (R[1,2] + R[2,1]) / S
            z = 0.25 * S
    return normalize_quat_wxyz([w, x, y, z])

def ortho_basis_from_two(v1: np.ndarray, v2: np.ndarray) -> np.ndarray:
    """
    Build an orthonormal basis (columns) from two non-colinear vectors.
    Returns 3x3 whose columns are e1,e2,e3.
    """
    v1 = np.asarray(v1, dtype=np.float64)
    v2 = np.asarray(v2, dtype=np.float64)
    e1 = v1 / (np.linalg.norm(v1) + 1e-12)
    v2p = v2 - np.dot(v2, e1) * e1
    e2 = v2p / (np.linalg.norm(v2p) + 1e-12)
    e3 = np.cross(e1, e2)
    B = np.stack([e1, e2, e3], axis=1)
    # Ensure right-handed (det +1)
    if np.linalg.det(B) < 0:
        e2 = -e2
        e3 = np.cross(e1, e2)
        B = np.stack([e1, e2, e3], axis=1)
    return B.astype(np.float32)

def R_from_two_axis(a1, a2, b1, b2) -> np.ndarray:
    """
    Rotation mapping a1->b1 and a2->b2 (both pairs non-colinear).
    Column-vector convention: X_tgt = R * X_src.
    """
    B_src = ortho_basis_from_two(a1, a2)  # columns are src basis
    B_tgt = ortho_basis_from_two(b1, b2)  # columns are tgt basis
    return (B_tgt @ B_src.T).astype(np.float32)

# =========================
# Fixed basis changes
# =========================

# WORLD basis (right operand): (DVL world) <- (ROS world)

R_dvl_ros = R_from_two_axis(
    a1=[1,0,0],   # ROS forward
    a2=[0,0,1],   # ROS up
    b1=[0,0,1],   # DVL forward
    b2=[1,0,0]   # DVL up
    # b1=[1 , 0, 0],   # DVL forward
    # b2=[0 ,0, -1]   # DVL up
)
T_dvl_ros = make_T(R_dvl_ros, np.zeros(3, np.float32))

R_ros_dvl = np.array([
    [0, 0, 1],
    [0, -1, 0],
    [1, 0, 0]
])

# WORLD basis (right operand): (ROS world) <- (Replica world)
# We want Replica +Y (forward) -> ROS +X (forward), Replica +X (right) -> ROS -Y.
# Both are Z-up; so this is just a rotation about Z by -90°.
# R_ros_replica_world = R_from_two_axis(
#     a1=np.array([0, 1, 0], np.float32),  # +Y_rep (forward)
#     a2=np.array([0, 0, 1], np.float32),  # +Z_rep (up)
#     b1=np.array([1, 0, 0], np.float32),  # +X_ros (forward)
#     b2=np.array([0, 0, 1], np.float32),  # +Z_ros (up)
# )
# T_ros_replica = make_T(R_ros_replica_world, np.zeros(3, np.float32))

# CAMERA basis (left operand): (Replica cam RDF) <- (ROS cam)
# Case A: ROS *_optical_frame already RDF -> Identity
# R_cam_rosopt_to_RDF = np.eye(3, dtype=np.float32)
R_cam_neutral = np.eye(3, dtype=np.float32)

# Case B: ROS camera_link/center (X fwd, Y left, Z up) -> RDF (X right, Y down, Z fwd)
# Constrain by mapping forward and down:
#   forward_ros(+X) -> forward_rep(+Z)
#   down_ros(-Z)    -> down_rep(+Y)
R_cam_roslink_to_RDF = R_from_two_axis(
    a1=np.array([1, 0,  0], np.float32),  # forward in ROS cam_link
    a2=np.array([0, 0, -1], np.float32),  # down in ROS cam_link
    b1=np.array([0, 0,  1], np.float32),  # forward in Replica RDF
    b2=np.array([0, 1,  0], np.float32),  # down in Replica RDF
)

def xyzw_to_wxyz(q):
    """(x,y,z,w) -> (w,x,y,z). Works with shape (...,4)."""
    q = np.asarray(q)
    return q[..., [3, 0, 1, 2]]

def wxyz_to_xyzw(q):
    """(w,x,y,z) -> (x,y,z,w). Works with shape (...,4)."""
    q = np.asarray(q)
    return q[..., [1, 2, 3, 0]]

# =========================
# Main conversion
# =========================

def dvl_to_ros_transform(
    q_xyzw: np.ndarray,
    t_dvl: np.ndarray,
    use_ros_optical_frame: bool = False,
    add_world_origin_offset_replica_in_ros: np.ndarray | None = None,
):
    """
    Convert DVL pose (camera->world, DVL frames) to ROS pose (camera->world).

    Inputs:
      t_dvl : (3,) camera translation in DVL world (map/odom)
      R : Rotation matrix : camera -> world
      use_source_optical_frame : True if the input camera frame is *_optical_frame (already RDF).
      add_world_origin_offset_ros_in_dvl : optional (3,), if ROS world origin differs,
          provide the ROS origin expressed in source world coords; it will be added to the
          world-basis change as a translation.

    Returns:
      q_ros : (4,) quaternion (x,y,z,w) of camera in ROS world
      t_ros : (3,) camera translation in ROS world
    """

    # q_wxyz = rotmat_to_quat_wxyz(R)       # (w,x,y,z)

    # q_dvl_xyzw = wxyz_to_xyzw(q_wxyz)    

    q_dvl_xyzw = q_xyzw

    t_dvl = np.asarray(t_dvl, dtype=np.float32)
    qx, qy, qz, qw = np.asarray(q_dvl_xyzw, dtype=np.float32)

    # (1) Build T_ros_c : (DVL world) <- (DVL cam)   [camera->world]
    R_dvl = quat_xyzw_to_R(qx, qy, qz, qw)
    T_dvl_c = make_T(R_dvl, t_dvl)

    # (2) Invert for composition convenience: T_c_dvl : (DVL cam) <- (DVL world)
    T_c_dvl = np.linalg.inv(T_dvl_c)

    # (3) World basis change (right operand): (DVL world) <- (ROS world)
    T_world = T_dvl_ros.copy()
    # if add_world_origin_offset_replica_in_ros is not None:
    #     T_world[:3, 3] = np.asarray(add_world_origin_offset_replica_in_ros, dtype=np.float32)

    # (4) Camera basis change (left operand): (ROS cam) <- (DVL cam)
    R_cam = R_cam_neutral # 
    T_cam = make_T(R_cam, np.zeros(3, np.float32))

    # (5) Compose like your spec and invert back:
    #     T_c_ros = T_cam @ T_c_dvl @ T_world
    #     T_ros_c = inv(T_c_ros)  -> (ROS world) <- (ROS cam)
    T_c_ros = T_cam @ T_c_dvl @ T_world
    T_ros_c = np.linalg.inv(T_c_ros)

    # (6) Extract Replica pose
    R_ros = T_ros_c[:3, :3]
    t_ros = T_ros_c[:3, 3]
    q_ros_wxyz = rotmat_to_quat_wxyz(R_ros)


    q_ros = wxyz_to_xyzw(q_ros_wxyz)

    return q_ros, t_ros

    # return t_replica.astype(np.float32), q_replica.astype(np.float32), T_replica_c.astype(np.float32)

def dvl_to_ros_transform_beams(beam_dirs):
    new_beam_dirs = (R_ros_dvl @ beam_dirs.T).T
    return new_beam_dirs
    