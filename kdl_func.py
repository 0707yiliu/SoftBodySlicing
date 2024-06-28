import PyKDL as kdl
from kdl_parser.urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
import numpy as np

class KDL_ROBOT:
    def __init__(self, urdf_file, root_link, end_link):
        robot = URDF.from_xml_file(urdf_file)
        tree = kdl_tree_from_urdf_model(robot)
        self.chain = tree.getChain(root_link, end_link)

    def forward(self, qpos):
        fk = kdl.ChainFkSolverPos_recursive(self.chain)
        pos = kdl.Frame()
        q = kdl.JntArray(self.chain.getNrOfJoints())
        for i in range(self.chain.getNrOfJoints()):
            q[i] = qpos[i]
        fk_flag = fk.JntToCart(q, pos)
        f_pos = np.zeros(3)
        for i in range(3):
            f_pos[i] = pos.p[i]
        return f_pos, kdl.Rotation(pos.M).GetQuaternion()

    def inverse(self, init_joint, goal_pose, goal_rot):
        try:
            rot = kdl.Rotation()
            rot = rot.Quaternion(goal_rot[0], goal_rot[1], goal_rot[2], goal_rot[3])  # radium x y z w
            pos = kdl.Vector(goal_pose[0], goal_pose[1], goal_pose[2])
        except ValueError:
            print("The target pos can not be transfor to IK-function.")
        target_pos = kdl.Frame(rot, pos)
        # print(target_pos)
        fk = kdl.ChainFkSolverPos_recursive(self.chain)
        # inverse kinematics
        ik_v = kdl.ChainIkSolverVel_pinv(self.chain)
        # ik = kdl.ChainIkSolverPos_NR(chain, fk, ik_v, maxiter=100, eps=math.pow(10, -9))

        # try:
        #     q_min = kdl.JntArray(len(joint_limit_lower))
        #     q_max = kdl.JntArray(len(joint_limit_lower))
        #     for i in range(len(joint_limit_lower)):
        #         q_min[i] = joint_limit_lower[i]
        #         q_max[i] = joint_limit_lower[i]
        # except ValueError:
        #     print("you should input the joint limitation value.")

        # ik_p_kdl = kdl.ChainIkSolverPos_NR_JL(chain, q_min, q_max, fk, ik_v)
        ik_p_kdl = kdl.ChainIkSolverPos_NR(self.chain, fk, ik_v)
        q_init = kdl.JntArray(self.chain.getNrOfJoints())
        for i in range(6):
            q_init[i] = init_joint[i]
        q_out = kdl.JntArray(self.chain.getNrOfJoints())
        ik_p_kdl.CartToJnt(q_init, target_pos, q_out)
        # print("Output angles:", q_out)
        q_out_trans = np.zeros(self.chain.getNrOfJoints())
        for i in range(self.chain.getNrOfJoints()):
            q_out_trans[i] = np.array(q_out[i])
        # print(q_out_trans)
        return (q_out_trans)

