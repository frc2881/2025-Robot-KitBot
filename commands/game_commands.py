from typing import TYPE_CHECKING
from commands2 import Command, cmd
from wpilib import RobotBase
from lib import logger, utils
from lib.classes import ControllerRumbleMode, ControllerRumblePattern
if TYPE_CHECKING: from robot_container import RobotContainer
import constants

class GameCommands:
  def __init__(
      self,
      robot: "RobotContainer"
    ) -> None:
    self._robot = robot

  def rumbleControllersCommand(self, mode: ControllerRumbleMode, pattern: ControllerRumblePattern) -> Command:
    return cmd.parallel(
      self._robot.driverController.rumbleCommand(pattern).onlyIf(lambda: mode == ControllerRumbleMode.Driver or mode == ControllerRumbleMode.Both),
      self._robot.operatorController.rumbleCommand(pattern).onlyIf(lambda: mode == ControllerRumbleMode.Operator or mode == ControllerRumbleMode.Both)
    ).onlyIf(
      lambda: RobotBase.isReal() and not utils.isAutonomousMode()
    ).withName("GameCommands:RumbleControllers")
  
  def scoreCommand(self) -> Command:
    return self._robot.rollerSubsystem.ejectCommand().withTimeout(2.0)