import ast
from pathlib import Path

import yaml

try:
    from yaml import CLoader as Loader
except ImportError as e:
    from yaml import Loader


class Config:
    def __init__(self, config_filename: str = "ipconfig.yaml"):

        path = Path(__file__).resolve()
        # use with the auto open and close
        with open(path.parent / config_filename, "r") as file:
            # read and sets the robot addr and base ID
            self.set_config(yaml.load(file, Loader))

    def set_config(self, raw):
        self.blue = raw["blue"]
        self.yellow = raw["yellow"]
        self.grSim_addr = (raw["grSim"]["ip"], raw["grSim"]["port"])
        self.vision = raw["vision"]["multicast-group"], raw["vision"]["port"]
        self.game_controller = raw["gc"]["multicast-group"], raw["gc"]["port"]


        self.robot_ip = raw["network"]["robot_ip"]
        self.vision_ip = raw["network"]["vision_ip"]
        self.use_grSim_vision = raw["use_grSim_vision"]
        self.us_yellow = raw["us_yellow"]
        self.us_positive = raw["us_positive"]
        self.send_to_grSim = raw["send_to_grSim"]
        self.team_name = raw.get("team_name", "TurtleRabbit")
        self.record_world_snapshots = bool(raw.get("record_world_snapshots", False))
        self.record_world_snapshot_dir = raw.get(
            "record_world_snapshot_dir", "match_replays"
        )

        goalie_cfg = raw.get("goalie", {})
        self.goalie_yellow_id = self._letter_to_grsim_id(self.yellow, goalie_cfg.get("yellow"))
        self.goalie_blue_id = self._letter_to_grsim_id(self.blue, goalie_cfg.get("blue"))

    @staticmethod
    def _letter_to_grsim_id(team_cfg: dict, letter: str | None) -> int | None:
        if letter is None or letter not in team_cfg:
            return None
        return team_cfg[letter]["grSimID"]


if __name__ == "__main__":
    server_config = Config()
    print(server_config.blue)
