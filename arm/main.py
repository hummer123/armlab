# import mujoco.viewer 
# import time
import ikpy.chain
import ikpy.utils.plot as plot_utils
import matplotlib.pyplot as plt
import transforms3d as tf



def main():
    my_chain = ikpy.chain.Chain.from_urdf_file("model/urdf/ur5e_robot.urdf")
    ee_pos = [-0.13, 0.5, 0.1]
    ee_euler = [3.14, 0, 1.57]
    ee_orientation = tf.euler.euler2mat(*ee_euler)
    ref_pos = [0, -1.57, -1.34, 2.65, -1.3, 1.55, 0, 0]

    fig, ax = plot_utils.init_3d_figure()
    my_chain.plot(my_chain.inverse_kinematics(ee_pos, ee_orientation, "all", initial_position=ref_pos), ax)
    plt.show()


if __name__ == "__main__":
    main()



