import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped

def create_pose_msg(stamp, q, map_frame, X, P, z_water=0.0):
    state_dim = X.shape[0]
    has_pitch = state_dim == 9
    yaw_idx = 2 if state_dim == 5 else 3
    out = PoseWithCovarianceStamped()
    out.header.stamp = stamp
    out.header.frame_id = map_frame
    out.pose.pose.position.x = X[0, 0]
    out.pose.pose.position.y = X[1, 0]
    out.pose.pose.position.z = z_water if state_dim == 5 else X[2, 0]
    out.pose.pose.orientation.x = q[0]
    out.pose.pose.orientation.y = q[1]
    out.pose.pose.orientation.z = q[2]
    out.pose.pose.orientation.w = q[3]
    cov = np.zeros((6, 6))
    cov[0, 0] = P[0, 0]
    cov[0, 1] = P[0, 1]
    cov[1, 0] = P[1, 0]
    cov[1, 1] = P[1, 1]
    if state_dim != 5:
        cov[0, 2] = P[0, 2]
        cov[1, 2] = P[1, 2]
        cov[2, 0] = P[2, 0]
        cov[2, 1] = P[2, 1]
        cov[2, 2] = P[2, 2]
    if has_pitch:
        cov[4, 4] = P[4, 4]
    cov[5, 5] = P[yaw_idx, yaw_idx]
    out.pose.covariance = cov.reshape(-1).tolist()
    return out

def create_transform_msg(stamp, q, map_frame, estimated_auv_frame, X, z_water=0.0):
    tf_msg = TransformStamped()
    tf_msg.header.stamp = stamp
    tf_msg.header.frame_id = map_frame
    tf_msg.child_frame_id = estimated_auv_frame
    tf_msg.transform.translation.x = X[0, 0]
    tf_msg.transform.translation.y = X[1, 0]
    tf_msg.transform.translation.z = z_water if X.shape[0] == 5 else X[2, 0]
    tf_msg.transform.rotation.x = q[0]
    tf_msg.transform.rotation.y = q[1]
    tf_msg.transform.rotation.z = q[2]
    tf_msg.transform.rotation.w = q[3]
    return tf_msg
