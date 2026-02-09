import logging
import os
from importlib.resources import files
from typing import Any

import cv2
import gymnasium as gym
from rcs import sim
import tacto
from omegaconf import OmegaConf

logger = logging.getLogger(__name__)

# TODO: make this a camera
class TactoSimWrapper(gym.Wrapper):
    """Wrapper to use Tacto with RCS Sim."""
    RGB_KEY = "rgb"

    def __init__(
        self,
        env: gym.Env,
        tacto_sites: list[str],
        tacto_geoms: list[str],
        simulation: sim.Sim,
        tacto_meshes: dict[str, str] | None = None,
        tacto_config: str | None = None,
        tacto_bg: str | None = None,
        enable_depth: bool = False,
        tacto_fps: int = 60,
        visualize: bool = False,
    ):
        """
        Initialize Tacto sensor with the given configuration.
        Args:
            env (gym.Env): The environment to wrap.
            simulation (sim.Sim): The simulation instance.
            tacto_sites (list[str]): List of sites to mount Tacto cameras.
            tacto_geoms (list[str]): List of mjOBJ_GEOM names to add.
            tacto_meshes (dict[str, str] | None): Dictionary mapping geom names to mesh names.
                                                  Needed when geom names are not the same as the mesh name in the XML.
            tacto_config (str)=None: Absolute path to the Tacto configuration folder containing "digit.yaml".
                                     If None, package default is used.
            tacto_bg (str)=None: Absolute path to the background image for Tacto, ending with ".jpg".
                                 If None, package default is used.
            enable_depth (bool)=False: Whether to enable depth rendering.
            tacto_fps (int)=60: Frames per second for Tacto rendering.
            visualize (bool)=False: Whether to visualize Tacto rendering in a separate window.
        """
        super().__init__(env)
        self.env = env
        if tacto_config is None:
            tacto_config = os.path.dirname(str(files("tacto") / "cfg" / "digit.yaml"))
            logger.warning(f"No tacto_config provided, using default from package: {tacto_config}/digit.yaml")
        if tacto_bg is None:
            tacto_bg = str(files("tacto") / "assets" / "bg_digit_240_320.jpg")
            logger.warning(f"No tacto_bg provided, using default from package: {tacto_bg}")
        config_path = os.path.join(tacto_config, "digit.yaml")
        t_config = OmegaConf.load(config_path)
        self.tacto_sensor = tacto.Sensor(**t_config.tacto, background=cv2.imread(tacto_bg))
        self.tacto_fps = tacto_fps
        self.tacto_last_render = -1
        self.tacto_sites = tacto_sites
        self.tacto_geoms = tacto_geoms
        self.tacto_meshes = tacto_meshes if tacto_meshes is not None else {}
        self.enable_depth = enable_depth
        self.sim = simulation

        self.initialized = False
        self.visualize = visualize

        self.observation_space: gym.spaces.Dict
        # rgb is always included
        params: dict = {
            f"/{name}/{self.RGB_KEY}/frame": {
                "height": camera_set.config(name).resolution_height,
                "width": camera_set.config(name).resolution_width,
            }
            for name in camera_set.camera_names
        }
        # TODO: recursively update
        self.observation_space.spaces.update(
            get_space(
                CameraDictType,
                child_dict_keys_to_unfold={
                    "camera_names": camera_set.camera_names,
                    "camera_type": [self.RGB_KEY],
                },
                params=params,
            ).spaces
        )
        self.camera_key = get_space_keys(CameraDictType)[0]

    def reset(
        self, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict[str, Any], dict[str, Any]]:
        obs, info = super().reset(seed=seed, options=options)
        if not self.initialized:
            # Set up Tacto sensor with the simulation
            for site in self.tacto_sites:
                self.tacto_sensor.add_camera_mujoco(site, self.sim.model, self.sim.data)
            for geom in self.tacto_geoms:
                if geom in self.tacto_meshes:
                    self.tacto_sensor.add_geom_mujoco(geom, self.sim.model, self.sim.data, self.tacto_meshes[geom])
                else:
                    self.tacto_sensor.add_geom_mujoco(geom, self.sim.model, self.sim.data)
            self.initialized = True
        self.tacto_last_render = -1  # Reset last render time
        colors, depths = self.tacto_sensor.render(self.sim.model, self.sim.data)
        for site, color, depth in zip(self.tacto_sites, colors, depths, strict=False):
            obs.setdefault("frames", {}).setdefault(f"tactile_{site}", {}).setdefault("rgb", {})["data"] = color
            if self.enable_depth:
                obs.setdefault("frames", {}).setdefault(f"tactile_{site}", {}).setdefault("depth", {})["data"] = depth
        return obs, info

    def step(self, action: dict[str, Any]):
        obs, reward, done, truncated, info = super().step(action)
        if self.tacto_last_render + (1 / self.tacto_fps) < self.sim.data.time:
            colors, depths = self.tacto_sensor.render(self.sim.model, self.sim.data)
            self.tacto_sensor.updateGUI(colors, depths) if self.visualize else None
            self.tacto_last_render = self.sim.data.time
            for site, color, depth in zip(self.tacto_sites, colors, depths, strict=False):
                obs.setdefault("frames", {}).setdefault(f"tactile_{site}", {}).setdefault("rgb", {})["data"] = color
                if self.enable_depth:
                    obs.setdefault("frames", {}).setdefault(f"tactile_{site}", {}).setdefault("depth", {})[
                        "data"
                    ] = depth
        return obs, reward, done, truncated, info
