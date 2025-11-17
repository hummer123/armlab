"""
UR5e机械臂工作空间分析
根据URDF文件参数计算末端执行器可达范围
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# UR5e DH参数（从URDF提取）
class UR5eKinematics:
    def __init__(self):
        # 连杆长度参数（单位：米）
        self.d1 = 0.163      # 基座到肩部的高度
        self.a2 = 0.425      # 大臂长度
        self.a3 = 0.392      # 小臂长度
        self.d4 = 0.127      # 腕部1偏移
        self.d5 = 0.1        # 腕部2偏移
        self.d6 = 0.1        # 末端执行器偏移
        self.shoulder_offset = 0.138  # 肩部Y偏移
        self.elbow_offset = 0.131     # 肘部Y偏移
        
        # 关节角度限制（单位：弧度）
        self.joint_limits = [
            (-2*np.pi, 2*np.pi),  # shoulder_pan
            (-2*np.pi, 2*np.pi),  # shoulder_lift
            (-np.pi, np.pi),       # elbow
            (-2*np.pi, 2*np.pi),  # wrist_1
            (-2*np.pi, 2*np.pi),  # wrist_2
            (-2*np.pi, 2*np.pi),  # wrist_3
        ]
    
    def forward_kinematics(self, joint_angles):
        """
        正向运动学：根据关节角度计算末端执行器位置
        joint_angles: [θ1, θ2, θ3, θ4, θ5, θ6]
        返回: [x, y, z] 末端执行器位置
        """
        θ1, θ2, θ3, θ4, θ5, θ6 = joint_angles
        
        # 简化的正向运动学计算（基于URDF的几何关系）
        # 肩部位置
        shoulder_x = 0
        shoulder_y = 0
        shoulder_z = self.d1
        
        # 大臂端点位置（shoulder_lift旋转后）
        c1, s1 = np.cos(θ1), np.sin(θ1)
        c2, s2 = np.cos(θ2), np.sin(θ2)
        c23 = np.cos(θ2 + θ3)
        s23 = np.sin(θ2 + θ3)
        
        # 简化计算（忽略腕部关节对位置的影响，只考虑方向）
        x = c1 * (self.a2*c2 + self.a3*c23)
        y = s1 * (self.a2*c2 + self.a3*c23)
        z = self.d1 + self.a2*s2 + self.a3*s23
        
        return np.array([x, y, z])
    
    def calculate_workspace(self, num_samples=1000):
        """
        蒙特卡洛采样计算工作空间
        """
        points = []
        
        for _ in range(num_samples):
            # 随机生成关节角度
            joint_angles = []
            for lower, upper in self.joint_limits[:3]:  # 只考虑前3个关节
                angle = np.random.uniform(lower, upper)
                joint_angles.append(angle)
            
            # 后3个关节对位置影响较小，设为0
            joint_angles.extend([0, 0, 0])
            
            # 计算末端位置
            pos = self.forward_kinematics(joint_angles)
            points.append(pos)
        
        return np.array(points)
    
    def calculate_reach_range(self):
        """
        计算理论可达范围
        """
        # 最大延伸（所有连杆伸直）
        max_reach = self.a2 + self.a3
        
        # 最小延伸（连杆折叠）
        min_reach = abs(self.a2 - self.a3)
        
        # 垂直范围
        z_max = self.d1 + self.a2 + self.a3
        z_min = self.d1 - (self.a2 + self.a3)
        
        return {
            'max_horizontal_reach': max_reach,
            'min_horizontal_reach': min_reach,
            'z_max': z_max,
            'z_min': z_min,
            'base_height': self.d1
        }


def visualize_workspace():
    """
    可视化工作空间
    """
    ur5e = UR5eKinematics()
    
    # 计算理论范围
    reach = ur5e.calculate_reach_range()
    print("=" * 50)
    print("UR5e 末端执行器工作空间分析")
    print("=" * 50)
    print(f"基座高度: {reach['base_height']:.3f} m")
    print(f"最大水平延伸: {reach['max_horizontal_reach']:.3f} m")
    print(f"最小水平延伸: {reach['min_horizontal_reach']:.3f} m")
    print(f"最大高度 (Z): {reach['z_max']:.3f} m")
    print(f"最小高度 (Z): {reach['z_min']:.3f} m")
    print("=" * 50)
    
    # 蒙特卡洛采样
    print("正在生成工作空间点云...")
    points = ur5e.calculate_workspace(num_samples=5000)
    
    # 3D可视化
    fig = plt.figure(figsize=(15, 5))
    
    # 子图1: 3D视图
    ax1 = fig.add_subplot(131, projection='3d')
    ax1.scatter(points[:, 0], points[:, 1], points[:, 2], 
                c=points[:, 2], cmap='viridis', s=1, alpha=0.5)
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('UR5e 3D Workspace')
    ax1.set_box_aspect([1,1,1])
    
    # 子图2: XY平面投影
    ax2 = fig.add_subplot(132)
    ax2.scatter(points[:, 0], points[:, 1], s=1, alpha=0.3)
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('Top View (XY Plane)')
    ax2.set_aspect('equal')
    ax2.grid(True)
    
    # 画圆环表示最大/最小半径
    theta = np.linspace(0, 2*np.pi, 100)
    ax2.plot(reach['max_horizontal_reach']*np.cos(theta), 
             reach['max_horizontal_reach']*np.sin(theta), 
             'r--', label=f"Max Reach: {reach['max_horizontal_reach']:.2f}m")
    ax2.plot(reach['min_horizontal_reach']*np.cos(theta), 
             reach['min_horizontal_reach']*np.sin(theta), 
             'g--', label=f"Min Reach: {reach['min_horizontal_reach']:.2f}m")
    ax2.legend()
    
    # 子图3: XZ平面投影
    ax3 = fig.add_subplot(133)
    ax3.scatter(points[:, 0], points[:, 2], s=1, alpha=0.3)
    ax3.set_xlabel('X (m)')
    ax3.set_ylabel('Z (m)')
    ax3.set_title('Side View (XZ Plane)')
    ax3.axhline(y=reach['base_height'], color='k', linestyle='--', 
                label=f"Base Height: {reach['base_height']:.2f}m")
    ax3.grid(True)
    ax3.legend()
    
    plt.tight_layout()
    plt.savefig('/home/unitree/ws/sim/armlab/arm/workspace_visualization.png', dpi=150)
    print("工作空间可视化已保存到: arm/workspace_visualization.png")
    plt.show()


if __name__ == "__main__":
    visualize_workspace()
