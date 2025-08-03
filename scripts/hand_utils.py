import numpy as np

def quaternion_from_euler_numpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Converts euler roll, pitch, yaw to a quaternion (x, y, z, w)."""
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    q = np.empty((4, ))
    q[0] = sr * cp * cy - cr * sp * sy
    q[1] = cr * sp * cy + sr * cp * sy
    q[2] = cr * cp * sy - sr * sp * cy
    q[3] = cr * cp * cy + sr * sp * sy
    return q

def get_hand_pose_from_landmarks(landmarks, image_width, image_height):
    """Extract hand pose from landmarks."""
    cx = landmarks[0].x
    cy = landmarks[0].y
    index_mcp = landmarks[5]
    pinky_mcp = landmarks[9]
    angle_rad = np.arctan2(pinky_mcp.y - index_mcp.y, pinky_mcp.x - index_mcp.x)
    roll_mapped = angle_rad + np.pi/2
    while roll_mapped > np.pi:
        roll_mapped -= 2*np.pi
    while roll_mapped < -np.pi:
        roll_mapped += 2*np.pi
    return cx, cy, roll_mapped + np.pi/2

def get_finger_states(landmarks):
    """Determine which fingers are raised."""
    finger_tips = [4, 8, 12, 16, 20]
    finger_pips = [3, 6, 10, 14, 18]
    fingers_up = []
    wrist = landmarks[0]
    thumb_tip = landmarks[finger_tips[0]]
    thumb_pip = landmarks[finger_pips[0]]
    dist_tip = ((thumb_tip.x - wrist.x)**2 + (thumb_tip.y - wrist.y)**2)**0.5
    dist_pip = ((thumb_pip.x - wrist.x)**2 + (thumb_pip.y - wrist.y)**2)**0.5
    if dist_tip > dist_pip:
        fingers_up.append(1)
    else:
        fingers_up.append(0)
    for i in range(1, 5):
        if landmarks[finger_tips[i]].y < landmarks[finger_pips[i]].y:
            fingers_up.append(1)
        else:
            fingers_up.append(0)
    return fingers_up
