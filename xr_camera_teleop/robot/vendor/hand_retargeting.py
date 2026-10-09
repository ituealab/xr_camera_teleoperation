# Vendored from unitreerobotics/xr_teleoperate @ 817fb00 (Apache-2.0, see LICENSE-THIRD-PARTY).
# Source file: teleop/robot_control/hand_retargeting.py (Dex3 branch only)
# MODIFIED: Inspire/BrainCo variants removed; cwd-relative asset paths replaced by xr_camera_teleop.paths;
# dex_retargeting (needs torch) is imported lazily inside HandRetargeting.__init__ so that importing this
# module is light; logging_mp -> logging.
import logging
from pathlib import Path

import yaml

from xr_camera_teleop.paths import ASSETS, DEX3_YML

logger_mp = logging.getLogger(__name__)

# section "Sort by message structure" of https://support.unitree.com/home/en/G1_developer/dexterous_hand
LEFT_DEX3_API_JOINT_NAMES = ['left_hand_thumb_0_joint', 'left_hand_thumb_1_joint', 'left_hand_thumb_2_joint',
                             'left_hand_middle_0_joint', 'left_hand_middle_1_joint',
                             'left_hand_index_0_joint', 'left_hand_index_1_joint']
RIGHT_DEX3_API_JOINT_NAMES = ['right_hand_thumb_0_joint', 'right_hand_thumb_1_joint', 'right_hand_thumb_2_joint',
                              'right_hand_middle_0_joint', 'right_hand_middle_1_joint',
                              'right_hand_index_0_joint', 'right_hand_index_1_joint']


class HandRetargeting:
    """Unitree Dex3-1 DexPilot retargeting (left + right) from (25,3) wrist-local WebXR joint positions."""

    def __init__(self, config_path=DEX3_YML):
        from dex_retargeting import RetargetingConfig

        RetargetingConfig.set_default_urdf_dir(str(ASSETS))
        config_file_path = Path(config_path)

        try:
            with config_file_path.open('r') as f:
                self.cfg = yaml.safe_load(f)

            if 'left' not in self.cfg or 'right' not in self.cfg:
                raise ValueError("Configuration file must contain 'left' and 'right' keys.")

            left_retargeting_config = RetargetingConfig.from_dict(self.cfg['left'])
            right_retargeting_config = RetargetingConfig.from_dict(self.cfg['right'])
            self.left_retargeting = left_retargeting_config.build()
            self.right_retargeting = right_retargeting_config.build()

            self.left_retargeting_joint_names = self.left_retargeting.joint_names
            self.right_retargeting_joint_names = self.right_retargeting.joint_names
            self.left_indices = self.left_retargeting.optimizer.target_link_human_indices
            self.right_indices = self.right_retargeting.optimizer.target_link_human_indices

            self.left_dex3_api_joint_names = list(LEFT_DEX3_API_JOINT_NAMES)
            self.right_dex3_api_joint_names = list(RIGHT_DEX3_API_JOINT_NAMES)
            self.left_dex_retargeting_to_hardware = [self.left_retargeting_joint_names.index(name)
                                                     for name in self.left_dex3_api_joint_names]
            self.right_dex_retargeting_to_hardware = [self.right_retargeting_joint_names.index(name)
                                                      for name in self.right_dex3_api_joint_names]

        except FileNotFoundError:
            logger_mp.warning(f"Configuration file not found: {config_file_path}")
            raise
        except yaml.YAMLError as e:
            logger_mp.warning(f"YAML error while reading {config_file_path}: {e}")
            raise
        except Exception as e:
            logger_mp.error(f"An error occurred: {e}")
            raise
