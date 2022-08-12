from pathlib import Path

from pytransform3d.transform_manager import TransformManager

from .configuration import Configuration
from .localisation import Localisation
from .mission_control import MissionControl
from .motors import Motors
from .navigation import Navigation
from .sensors import Battery, Compass, Encoder, ExternalPositioning, Lidar, Screen
from .tools import Console
from .tools.rate import Rate


class Robot:
    def __init__(self, config: Configuration):
        Console.set_logging_file(config.logging_folder)

        self._tm = TransformManager()
        self._config = config

        Console.info("Initializing robot", config.robot_name, "...")

        # Create empty sensors
        self.lidar = None
        self.compass = None
        self.encoder = None
        self.external_positioning = None

        # Check for sensors:
        # if config.sensors.lidar is not None:
        #     self.lidar = Lidar(config.sensors.lidar, config.logging_folder)
        # if config.sensors.compass is not None:
        #     self.compass = Compass(config.sensors.compass, config.logging_folder)
        # if config.sensors.encoder is not None:
        #     self.encoder = Encoder(config.sensors.encoder, config.logging_folder)
        if config.sensors.external_positioning is not None:
            self.external_positioning = ExternalPositioning(
                config.sensors.external_positioning, config.logging_folder
            )
        if config.sensors.battery is not None:
            self.battery = Battery(config.sensors.battery, config.logging_folder)
        if config.sensors.screen is not None:
            self.screen = Screen(config.sensors.screen, config.logging_folder)

        # Create controllers
        self.localisation = Localisation(
            config.control.localisation, config.logging_folder
        )
        self.navigation = Navigation(config.control.navigation)
        self.mission_control = MissionControl(
            config.control.mission,
            Path(config.filename).parent / "missions",
        )

        # Create motors
        self.motors = Motors(config.motors, config.logging_folder)

    def run(self):
        Console.info("Running robot...")

        r = Rate(5.0)

        while not self.mission_control.finished:
            # Read sensors
            measurements = []
            if self.compass is not None:
                msg = self.compass.read()
                print("compass", msg)
                measurements.append(msg)
            if self.encoder is not None:
                self.encoder.yaw_rad = self.compass.yaw_rad
                msg = self.encoder.read()
                print("encoder", msg)
                measurements.append(msg)
            if self.external_positioning is not None:
                msg = self.external_positioning.read()
                print("external_positioning", msg)
                measurements.append(msg)
            if self.lidar is not None:
                msg = self.lidar.read()

            # Update navigation
            if len(measurements) > 0:
                print(measurements[0])
                for measurement in sorted(measurements, key=lambda m: m.stamp_s):
                    self.state = self.localisation.update(measurement)
            
            # print("State", self.state)
            # print("current waypoint", self.mission_control.waypoint)
            self.mission_control.update(self.state)
            speed_request = self.navigation.compute_request(
                self.state, self.mission_control.waypoint
            )
            self.motors.move(speed_request)

            r.sleep()
