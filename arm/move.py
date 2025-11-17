import mujoco
import mujoco.viewer
import ikpy.chain
import transforms3d as tf
import numpy as np
import time

def viewer_init(viewer):
    """渲染器的摄像头视角初始化."""
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE # type: ignore
    viewer.cam.lookat[:] = [0, 0.5, 0.5]
    viewer.cam.distance = 2.5
    viewer.cam.azimuth = 180
    viewer.cam.elevation = -30


def get_ee_pose(model, data, ee_site_name="attachment_site"):
    """获取末端执行器的位姿信息.
    
    Returns:
        position: 3D位置 [x, y, z]
        rotation_matrix: 3x3旋转矩阵
        euler_angles: 欧拉角 [roll, pitch, yaw] (弧度)
        quaternion: 四元数 [w, x, y, z]
    """
    ee_id = model.site(ee_site_name).id
    
    # 获取位置
    position = data.site_xpos[ee_id].copy()
    
    # 获取旋转矩阵 (3x3)
    rotation_matrix = data.site_xmat[ee_id].reshape(3, 3).copy()
    
    # 转换为欧拉角
    euler_angles = tf.euler.mat2euler(rotation_matrix)
    
    # 转换为四元数
    quaternion = tf.quaternions.mat2quat(rotation_matrix)
    
    return position, rotation_matrix, euler_angles, quaternion


def format_ee_info(position, euler_angles, quaternion):
    """格式化末端执行器信息为字符串."""
    info_lines = [
        "=== End Effector Pose ===",
        f"Position (m):",
        f"  X: {position[0]:7.4f}",
        f"  Y: {position[1]:7.4f}",
        f"  Z: {position[2]:7.4f}",
        f"Euler (rad):",
        f"  Roll:  {euler_angles[0]:7.4f}",
        f"  Pitch: {euler_angles[1]:7.4f}",
        f"  Yaw:   {euler_angles[2]:7.4f}",
        f"Euler (deg):",
        f"  Roll:  {np.rad2deg(euler_angles[0]):7.2f}°",
        f"  Pitch: {np.rad2deg(euler_angles[1]):7.2f}°",
        f"  Yaw:   {np.rad2deg(euler_angles[2]):7.2f}°",
        f"Quaternion [w,x,y,z]:",
        f"  {quaternion[0]:6.3f}, {quaternion[1]:6.3f},",
        f"  {quaternion[2]:6.3f}, {quaternion[3]:6.3f}",
    ]
    return "\n".join(info_lines)


class JointSpaceTrajectory:
    """关节空间轨迹生成器."""

    def __init__(self, start_joints, end_joints, steps):
        self.start_joints = np.array(start_joints)
        self.end_joints = np.array(end_joints)
        self.steps = steps
        self.step = (self.end_joints - self.start_joints) / self.steps
        self.trajectory = self._generate_trajectory()
        self.waypoints = self.start_joints

    def _generate_trajectory(self):
        """生成关节空间轨迹."""
        for i in range(self.steps + 1):
            yield self.start_joints + i * self.step
        # 确保最后一个点是目标关节角度
        yield self.end_joints

    def get_next_waypoint(self, qpos):
        """获取下一个关节角度."""
        # 增加容差值，因为 PD 控制器可能无法精确到达目标
        if np.allclose(qpos, self.waypoints, atol=5e-2):
            try:
                self.waypoints = next(self.trajectory)
            except StopIteration:
                pass
        return self.waypoints

def main():
    model = mujoco.MjModel.from_xml_path("model/universal_robots_ur5e/scene.xml")  # type: ignore
    data = mujoco.MjData(model)  # type: ignore
    # 不使用 active_links_mask，与 arm_mujoco.py 保持一致
    my_chain = ikpy.chain.Chain.from_urdf_file("model/urdf/ur5e_robot.urdf")

    # 使用与 arm_mujoco.py 相同的起始关节角度
    start_joints = np.array([-1.57, -1.34, 2.65, -1.3, 1.55, 0])
    data.qpos[:6] = start_joints  # 设置初始关节角度

    # 目标末端执行器位置和姿态（与 scene.xml 中的 target 位置一致）
    ee_pos = [-0.13, 0.5, 0.1]
    ee_euler = [3.14, 0, 1.57]
    # ref_pos 使用相同的起始关节角度作为 IK 求解的初始猜测
    ref_pos = [0, -1.57, -1.34, 2.65, -1.3, 1.55, 0, 0]  # 8个元素：OriginLink + 6个关节 + 末端执行器
    ee_orientation = tf.euler.euler2mat(*ee_euler)  # 欧拉角转旋转矩阵

    # 计算目标关节角度
    joint_angles = my_chain.inverse_kinematics(ee_pos, ee_orientation, "all", initial_position=ref_pos)
    end_joints = joint_angles[1:-1]  # 去掉第一个（OriginLink）和最后一个（末端执行器）关节的角度

    print("=== Inverse Kinematics Solution ===")
    print(f"Target Position: {ee_pos}")
    print(f"Target Euler (rad): {ee_euler}")
    print(f"Start Joints (rad): {start_joints}")
    print(f"Target Joints (rad): {end_joints}")
    print(f"Joint difference: {np.linalg.norm(end_joints - start_joints):.4f} rad\n")

    joint_trajectory = JointSpaceTrajectory(start_joints, end_joints, steps=200)

    with mujoco.viewer.launch_passive(model, data) as viewer:  # type: ignore
        viewer_init(viewer)
        
        # 用于控制显示更新频率
        display_counter = 0
        display_interval = 10  # 每10帧更新一次显示
        
        while viewer.is_running():
            waypoint = joint_trajectory.get_next_waypoint(data.qpos[:6])
            data.ctrl[:6] = waypoint  # 设置控制信号

            mujoco.mj_step(model, data)  # type: ignore
            
            # 定期更新末端执行器位姿显示
            if display_counter % display_interval == 0:
                position, rot_mat, euler, quat = get_ee_pose(model, data)
                ee_info = format_ee_info(position, euler, quat)
                
                # 计算与目标位置的误差
                target_pos = np.array(ee_pos)
                position_error = np.linalg.norm(position - target_pos)
                joint_error = np.linalg.norm(data.qpos[:6] - waypoint)
                
                # 打印到终端
                print("\033[2J\033[H")  # 清屏并移动光标到开始
                print(ee_info)
                print(f"\n=== Error Analysis ===")
                print(f"Target Position: [{ee_pos[0]:7.4f}, {ee_pos[1]:7.4f}, {ee_pos[2]:7.4f}]")
                print(f"Position Error: {position_error:.4f} m")
                print(f"Joint Error: {joint_error:.4f} rad")
                # print(f"\nTime: {data.time:.2f}s")
            
            display_counter += 1
            viewer.sync()


if __name__ == "__main__":
    main()
