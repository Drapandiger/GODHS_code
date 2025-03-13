import numpy as np
from scipy.spatial.transform import Rotation as R

class IK_Validation:
    def __init__(self):
        """七轴机械臂IK求解器"""
        # DH参数
        self.robot_params = {
            # DH参数
            'a1': -0.273,  'd1': 1.023,  'alpha1': 0,         # Joint 1
            'a2': 0.0,    'd2': 0.0,    'alpha2': -np.pi/2,   # Joint 2
            'a3': 0.0,    'd3': 0.316,  'alpha3': np.pi/2,    # Joint 3
            'a4': 0.0825, 'd4': 0.0,    'alpha4': np.pi/2,    # Joint 4
            'a5': -0.0825,'d5': 0.384,  'alpha5': -np.pi/2,   # Joint 5
            'a6': 0.0,    'd6': 0.0,    'alpha6': np.pi/2,    # Joint 6
            'a7': 0.088,  'd7': 0.107,  'alpha7': np.pi/2,    # Joint 7

            # 关节限位 (rad)
            'joint_limits': [
                [-2.8973, 2.8973],   # Joint 1
                [-1.7628, 1.7628],   # Joint 2
                [-2.8973, 2.8973],   # Joint 3
                [-3.0718, -0.0698],  # Joint 4
                [-2.8973, 2.8973],   # Joint 5
                [-0.0175, 3.7525],   # Joint 6
                [-2.8973, 2.8973]    # Joint 7
            ]
        }

        self.base_transform = np.array([
            [-1, 0, 0, 0],
            [0, -1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]

        ])

    def compute_jacobian(self, q):
        """
        计算机械臂的几何雅可比矩阵
        q: 关节角度列表 [q1, ..., q7]
        """
        J = np.zeros((6, 7))
        T_current = self.base_transform.copy()
        
        # 计算每个关节的轴向量和位置
        z_axes = []
        positions = []
        
        # 根据DH参数计算每个关节的变换
        for i in range(7):
            d = self.robot_params[f'd{i+1}']
            a = self.robot_params[f'a{i+1}']
            alpha = self.robot_params[f'alpha{i+1}']
            T = self._dh_transform(alpha, a, d, q[i])
            
            T_current = T_current @ T
            z_axes.append(T_current[:3, 2])
            positions.append(T_current[:3, 3])
        
        # 计算末端执行器位置
        p_ee = positions[-1]
        
        # 填充雅可比矩阵
        for i in range(7):
            # 线速度分量
            J[:3, i] = np.cross(z_axes[i], p_ee - positions[i])
            # 角速度分量
            J[3:, i] = z_axes[i]
        
        return J

    def _dh_transform(self, alpha, a, d, theta):
        """计算DH变换矩阵"""
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        
        return np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])

    def compute_fk(self, q):
        """计算正向运动学"""
        T = self.base_transform.copy()
        
        # 依次计算每个关节的变换
        for i in range(7):
            d = self.robot_params[f'd{i+1}']
            a = self.robot_params[f'a{i+1}']
            alpha = self.robot_params[f'alpha{i+1}']
            T = T @ self._dh_transform(alpha, a, d, q[i])
        
        return T

    def solve_ik(self, target_pose, current_config, weights=None, max_iter=100, tol=1e-3):
        """
        求解七轴机械臂的IK问题
        
        参数:
        target_pose: 目标位姿矩阵 (4x4)
        current_config: 当前关节角度 [q1,...,q7]
        weights: 优化权重 [position_w, rotation_w]
        """
        if weights is None:
            weights = {
                'position': 1.0,
                'rotation': 0.5
            }
        
        q = current_config.copy()
        
        for iteration in range(max_iter):
            # 1. 计算当前末端执行器位姿
            current_pose = self.compute_fk(q)
            
            # 2. 计算位姿误差
            pos_error = (target_pose[:3, 3] - current_pose[:3, 3]) * weights['position']
            rot_error = R.from_matrix(target_pose[:3, :3]) * R.from_matrix(current_pose[:3, :3]).inv()
            rot_error = rot_error.as_rotvec() * weights['rotation']
            
            error = np.concatenate([pos_error, rot_error])
            
            if np.linalg.norm(error) < tol:
                return q
            
            # 3. 计算雅可比矩阵
            J = self.compute_jacobian(q)
            
            # 4. 计算关节角度增量
            try:
                # 使用阻尼最小二乘法
                lambda_ = 0.5  # 阻尼因子
                delta_q = J.T @ np.linalg.inv(J @ J.T + lambda_**2 * np.eye(6)) @ error
            except np.linalg.LinAlgError:
                print("雅可比矩阵求逆失败")
                return None
            
            # 5. 更新关节角度（考虑关节限位）
            for i in range(7):
                joint_limits = self.robot_params['joint_limits'][i]
                new_angle = q[i] + delta_q[i]
                q[i] = np.clip(new_angle, joint_limits[0], joint_limits[1])
        
        print("达到最大迭代次数")
        return q

    def validate_ik_solution(self, base_pose, ee_pose, max_attempts=10):
        """
        验证给定的底座位姿和末端执行器位姿是否存在IK解

        参数:
        base_pose: 底座位姿 [x, y, yaw]
        ee_pose: 末端执行器位姿 [x, y, z, roll, pitch, yaw]
        max_attempts: 最大尝试求解次数

        返回:
        bool: 是否存在有效的IK解
        """
        # 构建底座变换矩阵
        base_rotation = R.from_euler('z', base_pose[2]).as_matrix()
        base_transform = np.eye(4)
        base_transform[:3, :3] = base_rotation
        base_transform[:2, 3] = base_pose[:2]  # x, y平移

        # 构建末端执行器变换矩阵
        ee_rotation = R.from_euler('zyx', ee_pose[3:][::-1]).as_matrix()
        ee_transform = np.eye(4)
        ee_transform[:3, :3] = ee_rotation
        ee_transform[:3, 3] = ee_pose[:3]

        # 计算相对末端执行器位姿
        relative_ee_transform = np.linalg.inv(base_transform) @ ee_transform

        # 尝试求解
        for _ in range(max_attempts):
            # 生成随机初始构型
            initial_config = np.zeros(7)
            for i in range(7):
                limits = self.robot_params['joint_limits'][i]
                initial_config[i] = np.random.uniform(limits[0], limits[1])
            
            solution = self.solve_ik(
                relative_ee_transform, 
                initial_config, 
                weights={'position': 1.0, 'rotation': 0.5},
                max_iter=200,
                tol=1e-4
            )
            
            if solution is not None:
                # 验证解的有效性
                final_pose = self.compute_fk(solution)
                pos_error = np.linalg.norm(final_pose[:3, 3] - relative_ee_transform[:3, 3])
                rot_error = R.from_matrix(final_pose[:3, :3]).inv() * R.from_matrix(relative_ee_transform[:3, :3])
                
                # 更严格的误差判断
                if pos_error < 1e-3 and np.linalg.norm(rot_error.as_rotvec()) < 1e-3:
                    return True  # 找到有效解，立即返回True
        
        return False  # 未找到有效解

def test_ik_validation():
    """测试IK解的验证功能"""
    solver = IK_Validation()

    # 测试场景1: 简单可达位姿
    base_pose1 = [0, 0, 0]  # 底座原点，无旋转
    ee_pose1 = [0.5, 0.3, 1.2, 0, 0, 0]  # 可达位姿

    # 测试场景2: 可能不可达的位姿
    base_pose2 = [0, 0, np.pi/4]  # 底座旋转45度
    ee_pose2 = [2.5, 3.0, 5.0, 0, 0, np.pi/2]  # 超出工作空间

    print("场景1是否存在IK解:", solver.validate_ik_solution(base_pose1, ee_pose1))
    print("场景2是否存在IK解:", solver.validate_ik_solution(base_pose2, ee_pose2))

if __name__ == "__main__":
    test_ik_validation()