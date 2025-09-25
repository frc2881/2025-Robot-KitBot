from typing import Callable
from wpilib import SmartDashboard
from commands2 import Subsystem, Command
from rev import SparkMax, SparkMaxConfig, SparkBase
from lib import logger, utils
import core.constants as constants

class Roller(Subsystem):
    def __init__(
       self,
       intakeSensorHasTarget: Callable[[], bool]      
    ):
      super().__init__()
      self._constants = constants.Subsystems.Roller

    self.intakeSensorHasTarget = intake
    self._Motor = SparkMax(self._constants., self.MotorType._brushless)

        