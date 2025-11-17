"""
UR5e机械臂DH参数可视化
使用matplotlib绘制DH坐标系和机械臂结构
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class UR5eDHModel:
    """UR5e DH参数模型"""
    
    def __init__(self):
        # DH参数表: [θ, d, a, α]
        # θ: 关节角（变量）
        # d: 连杆偏距（沿Z轴）
        # a: 连杆长度（沿X轴）
        # α: 连杆扭转角（绕X轴）
        
        self.dh_params = {
            'joint_1': {'theta': 0,      'd': 0.163,  'a': 0,     'alpha': np.pi/2},
            'joint_2': {'theta': 0,      'd': 0,      'a': 0.425, 'alpha': 0},
            'joint_3': {'theta': 0,      'd': 0,      'a': 0.392, 'alpha': 0},
            'joint_4': {'theta': 0,      'd': 0.127,  'a': 0,     'alpha': np.pi/2},
            'joint_5': {'theta': 0,      'd': 0.1,    'a': 0,     'alpha': -np.pi/2},
            'joint_6': {'theta': 0,      'd': 0.1,    'a': 0,     'alpha': 0},
        }
        
        # 从URDF提取的几何参数（用于对比）
        self.urdf_params = {
            'base_height': 0.163,
            'shoulder_offset_y': 0.138,
            'upper_arm_length': 0.425,
            'elbow_offset_y': 0.131,
            'forearm_length': 0.392,
            'wrist1_offset': 0.127,
            'wrist2_offset': 0.1,
            'ee_offset': 0.1,
        }
    
    def dh_transform(self, theta, d, a, alpha):
        """
        计算DH变换矩阵
        
        T = Rot(Z,θ) * Trans(Z,d) * Trans(X,a) * Rot(X,α)
        """
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        
        T = np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,   sa,     ca,    d   ],
            [0,   0,      0,     1   ]
        ])
        return T
    
    def forward_kinematics(self, joint_angles):
        """
        正向运动学：计算每个关节的位置和姿态
        
        Args:
            joint_angles: 6个关节角度的列表 [θ1, θ2, θ3, θ4, θ5, θ6]
        
        Returns:
            transforms: 每个关节相对于基座的变换矩阵列表
        """
        transforms = [np.eye(4)]  # 基座坐标系
        
        T_cumulative = np.eye(4)
        
        for i, (joint_name, params) in enumerate(self.dh_params.items()):
            # 使用给定的关节角度
            theta = joint_angles[i] if i < len(joint_angles) else params['theta']
            d = params['d']
            a = params['a']
            alpha = params['alpha']
            
            # 计算当前关节的DH变换
            T_i = self.dh_transform(theta, d, a, alpha)
            
            # 累积变换
            T_cumulative = T_cumulative @ T_i
            transforms.append(T_cumulative.copy())
        
        return transforms
    
    def get_joint_positions(self, joint_angles):
        """获取所有关节的3D位置"""
        transforms = self.forward_kinematics(joint_angles)
        positions = [T[:3, 3] for T in transforms]
        return np.array(positions)
    
    def plot_robot_configuration(self, joint_angles, ax=None):
        """
        绘制机械臂配置
        
        Args:
            joint_angles: 关节角度
            ax: matplotlib 3D轴对象
        """
        if ax is None:
            fig = plt.figure(figsize=(12, 10))
            ax = fig.add_subplot(111, projection='3d')
        
        # 获取关节位置
        positions = self.get_joint_positions(joint_angles)
        transforms = self.forward_kinematics(joint_angles)
        
        # 绘制连杆
        ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
                'o-', linewidth=3, markersize=8, label='Robot Links')
        
        # 绘制坐标系
        axis_length = 0.1
        colors = ['r', 'g', 'b']
        
        for i, T in enumerate(transforms):
            origin = T[:3, 3]
            
            # 绘制X, Y, Z轴
            for j, color in enumerate(colors):
                axis = T[:3, j] * axis_length
                ax.quiver(origin[0], origin[1], origin[2],
                         axis[0], axis[1], axis[2],
                         color=color, arrow_length_ratio=0.3, linewidth=1.5)
            
            # 标注关节
            if i > 0:
                ax.text(origin[0], origin[1], origin[2], f'J{i}', fontsize=10)
        
        # 标注末端执行器
        ee_pos = positions[-1]
        ax.text(ee_pos[0], ee_pos[1], ee_pos[2], 'EE', fontsize=12, color='red')
        
        # 设置坐标轴
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_zlabel('Z (m)', fontsize=12)
        angles_deg = ', '.join([f'{a:.1f}' for a in np.rad2deg(joint_angles)])
        ax.set_title(f'UR5e Configuration\nAngles (deg): [{angles_deg}]', 
                     fontsize=14)
        
        # 设置相同的比例
        max_range = 1.0
        ax.set_xlim([-max_range, max_range])
        ax.set_ylim([-max_range, max_range])
        ax.set_zlim([0, max_range])
        
        ax.legend()
        
        return ax


def create_dh_diagram():
    """创建DH参数图解"""
    
    fig = plt.figure(figsize=(20, 12))
    
    ur5e = UR5eDHModel()
    
    # 配置1: 零位姿态
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    angles_zero = [0, 0, 0, 0, 0, 0]
    ur5e.plot_robot_configuration(angles_zero, ax1)
    ax1.set_title('Configuration 1: Zero Position', fontsize=14, fontweight='bold')
    
    # 配置2: 肩部旋转
    ax2 = fig.add_subplot(2, 3, 2, projection='3d')
    angles_shoulder = [np.pi/4, 0, 0, 0, 0, 0]
    ur5e.plot_robot_configuration(angles_shoulder, ax2)
    ax2.set_title('Configuration 2: Shoulder Pan 45°', fontsize=14, fontweight='bold')
    
    # 配置3: 伸展姿态
    ax3 = fig.add_subplot(2, 3, 3, projection='3d')
    angles_extend = [0, -np.pi/4, -np.pi/4, 0, 0, 0]
    ur5e.plot_robot_configuration(angles_extend, ax3)
    ax3.set_title('Configuration 3: Extended', fontsize=14, fontweight='bold')
    
    # 配置4: 折叠姿态
    ax4 = fig.add_subplot(2, 3, 4, projection='3d')
    angles_fold = [0, -np.pi/3, np.pi/2, 0, 0, 0]
    ur5e.plot_robot_configuration(angles_fold, ax4)
    ax4.set_title('Configuration 4: Folded', fontsize=14, fontweight='bold')
    
    # 配置5: 复杂姿态
    ax5 = fig.add_subplot(2, 3, 5, projection='3d')
    angles_complex = [np.pi/6, -np.pi/4, np.pi/3, -np.pi/6, np.pi/4, 0]
    ur5e.plot_robot_configuration(angles_complex, ax5)
    ax5.set_title('Configuration 5: Complex Pose', fontsize=14, fontweight='bold')
    
    # 配置6: DH参数表
    ax6 = fig.add_subplot(2, 3, 6)
    ax6.axis('off')
    
    # 创建DH参数表格
    table_data = []
    table_data.append(['Joint', 'θ (rad)', 'd (m)', 'a (m)', 'α (rad)'])
    
    for i, (name, params) in enumerate(ur5e.dh_params.items(), 1):
        row = [
            f'J{i}',
            f'θ{i}*',
            f"{params['d']:.3f}",
            f"{params['a']:.3f}",
            f"{params['alpha']:.4f}"
        ]
        table_data.append(row)
    
    # 绘制表格
    table = ax6.table(cellText=table_data, cellLoc='center', loc='center',
                      bbox=[0, 0.3, 1, 0.6])
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1, 2)
    
    # 设置表头样式
    for i in range(5):
        table[(0, i)].set_facecolor('#4CAF50')
        table[(0, i)].set_text_props(weight='bold', color='white')
    
    # 添加说明
    ax6.text(0.5, 0.15, '* θi = Joint Variable (Controlled)', 
             ha='center', fontsize=10, style='italic')
    ax6.text(0.5, 0.05, 'DH Convention: Modified Denavit-Hartenberg', 
             ha='center', fontsize=10, weight='bold')
    
    ax6.set_title('UR5e DH Parameters Table', fontsize=14, fontweight='bold', pad=20)
    
    plt.tight_layout()
    plt.savefig('/home/unitree/ws/sim/armlab/arm/ur5e_dh_visualization.png', 
                dpi=150, bbox_inches='tight')
    print("✓ DH参数可视化已保存到: arm/ur5e_dh_visualization.png")


def print_dh_summary():
    """打印DH参数摘要"""
    
    print("\n" + "="*70)
    print("UR5e 机械臂 DH 参数摘要")
    print("="*70)
    
    ur5e = UR5eDHModel()
    
    print("\n【DH参数表】")
    print("-"*70)
    print(f"{'关节':<8} {'θ (变量)':<12} {'d (m)':<10} {'a (m)':<10} {'α (rad)':<12}")
    print("-"*70)
    
    for i, (name, params) in enumerate(ur5e.dh_params.items(), 1):
        print(f"Joint {i:<2}  θ{i} (变量)    "
              f"{params['d']:<10.3f} "
              f"{params['a']:<10.3f} "
              f"{params['alpha']:<12.4f}")
    
    print("-"*70)
    
    print("\n【从URDF提取的关键尺寸】")
    print("-"*70)
    for key, value in ur5e.urdf_params.items():
        print(f"{key:<25}: {value:.3f} m")
    print("-"*70)
    
    # 计算工作空间
    print("\n【理论工作空间】")
    print("-"*70)
    
    a2 = ur5e.dh_params['joint_2']['a']
    a3 = ur5e.dh_params['joint_3']['a']
    d1 = ur5e.dh_params['joint_1']['d']
    
    max_reach = a2 + a3
    min_reach = abs(a2 - a3)
    z_max = d1 + a2 + a3
    z_min = d1 - (a2 + a3)
    
    print(f"最大水平延伸: {max_reach:.3f} m")
    print(f"最小水平延伸: {min_reach:.3f} m")
    print(f"最大高度 (Z):  {z_max:.3f} m")
    print(f"最小高度 (Z):  {z_min:.3f} m")
    print("-"*70)
    
    # 示例：计算特定姿态的末端位置
    print("\n【正向运动学示例】")
    print("-"*70)
    
    test_angles = [0, -np.pi/4, -np.pi/4, 0, 0, 0]
    positions = ur5e.get_joint_positions(test_angles)
    ee_pos = positions[-1]
    
    print(f"关节角度: {np.rad2deg(test_angles)} (度)")
    print(f"末端位置: X={ee_pos[0]:.3f}, Y={ee_pos[1]:.3f}, Z={ee_pos[2]:.3f} (m)")
    print("-"*70)
    
    print("\n" + "="*70 + "\n")


if __name__ == "__main__":
    print("正在生成UR5e DH参数可视化...")
    print_dh_summary()
    create_dh_diagram()
    print("\n所有图表已生成完成！")
