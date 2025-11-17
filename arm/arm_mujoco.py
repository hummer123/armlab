import mujoco.viewer 
import time
import ikpy.chain
import transforms3d as tf



def main():
    model = mujoco.MjModel.from_xml_path("model/universal_robots_ur5e/scene.xml") # type: ignore
    data = mujoco.MjData(model)  # type: ignore
    my_chain = ikpy.chain.Chain.from_urdf_file("model/urdf/ur5e_robot.urdf")
    
    ee_pos = [-0.13, 0.5, 0.1]  # 末端执行器位置 xyz
    ee_euler = [3.14, 0, 1.57]  # 末端执行器欧拉角 roll pitch yaw
    ref_pos = [0, -1.57, -1.34, 2.65, -1.3, 1.55, 0, 0] # 初始位置
    ee_orientation = tf.euler.euler2mat(*ee_euler)  # 欧拉角转旋转矩阵
    ee_id = model.site("attachment_site").id  # 获取末端执行器site id
    print(f"End Effector Site ID: {ee_id}")

    # argument: 目标位置，目标姿态，末端执行器名称，初始位置
    joint_angles = my_chain.inverse_kinematics(ee_pos, ee_orientation, "all", initial_position=ref_pos)
    ctrl = joint_angles[1:-1]  # 去掉第一个和最后一个元素，对应mujoco的6个关节
    data.ctrl[:6] = ctrl  # 设置控制信号

    with mujoco.viewer.launch_passive(model, data) as viewer:  # type: ignore
        while viewer.is_running():
            mujoco.mj_step(model, data)  # type: ignore
            viewer.sync()
            time.sleep(0.01)

    ee_pos = data.site_xpos[ee_id]  # 获取末端执行器位置
    print(f"End Effector Position: {ee_pos}")

if __name__ == "__main__":
    main()



