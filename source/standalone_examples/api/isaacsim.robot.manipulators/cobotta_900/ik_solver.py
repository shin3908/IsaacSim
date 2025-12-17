from isaacsim.robot_motion.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver
from isaacsim.core.prims import Articulation
from typing import Optional


class KinematicsSolver(ArticulationKinematicsSolver):
    def __init__(self, robot_articulation: Articulation, end_effector_frame_name: Optional[str] = None) -> None:
        #TODO: change the config path
        self._kinematics = LulaKinematicsSolver(robot_description_path="/home/shinsakuo/workspace/IsaacSim/source/standalone_examples/api/isaacsim.robot.manipulators/cobotta_900/rmpflow/robot_descriptor.yaml",
                                                urdf_path="/home/shinsakuo/workspace/IsaacSim_4.5/extscache/isaacsim.asset.importer.urdf-2.3.10+106.4.0.lx64.r.cp310/data/urdf/robots/cobotta_pro_900/cobotta_pro_900.urdf")
        if end_effector_frame_name is None:
            end_effector_frame_name = "onrobot_rg6_base_link"
        ArticulationKinematicsSolver.__init__(self, robot_articulation, self._kinematics, end_effector_frame_name)
        return