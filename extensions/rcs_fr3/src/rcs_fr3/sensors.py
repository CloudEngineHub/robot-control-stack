
from rcs.sensors import BaseTorqueSensor


class FrankaTorqueSensor(BaseTorqueSensor):
    def __init__(self, simulation: sim.Sim, cfg: sim.SimRobotConfig):
        self.sim = simulation
        self.cfg = cfg

        self.idxs = [
            self.sim.model.jnt_dofadr[mujoco.mj_name2id(self.sim.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)]
            for joint_name in cfg.joints
        ]

    def dof(self):
        return common.robots_meta_config(self.cfg.robot_type).dof

    def torque(self):
        # TODO: get this mapping by joint names
        return self.sim.data.qfrc_constraint[self.idxs]