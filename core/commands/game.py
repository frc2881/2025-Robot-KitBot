from typing import TYPE_CHECKING
from commands2 import Command, cmd
from wpilib import RobotBase
from lib import logger, utils
from lib.classes import ControllerRumbleMode, ControllerRumblePattern, TargetAlignmentMode
if TYPE_CHECKING: from core.robot import RobotCore
from core.classes import TargetAlignmentLocation, TargetType
import core.constants as constants

class Game:
  def __init__(
      self,
      robot: "RobotCore"
    ) -> None:
    self._robot = robot

  def alignRobotToTarget(self, targetAlignmentMode: TargetAlignmentMode, targetAlignmentLocation: TargetAlignmentLocation) -> Command:
    return self._robot.drive.alignToTarget(
      self._robot.localization.getRobotPose, 
      lambda: self._robot.localization.getTargetPose(targetAlignmentLocation),
      targetAlignmentMode
    ).andThen(
      self.rumbleControllers(ControllerRumbleMode.Driver)
    ).withName(f'Game:AlignRobotToTarget:{ targetAlignmentMode.name }:{ targetAlignmentLocation.name }')
  
  def isRobotAlignedToTarget(self) -> bool:
    return self._robot.drive.isAlignedToTarget()

  def intakeCoral(self) -> Command:
    return cmd.waitSeconds(1.5).withName("GameCommands:IntakeCoral") # TODO: replace with intake sensor once installed and enabled

  def scoreCoral(self) -> Command:
    return self._robot.roller.score().withTimeout(0.75).withName("GameCommands:ScoreCoral")

  def rumbleControllers(
    self, 
    mode: ControllerRumbleMode = ControllerRumbleMode.Both, 
    pattern: ControllerRumblePattern = ControllerRumblePattern.Short
  ) -> Command:
    return cmd.parallel(
      self._robot.driver.rumble(pattern).onlyIf(lambda: mode != ControllerRumbleMode.Operator),
      self._robot.operator.rumble(pattern).onlyIf(lambda: mode != ControllerRumbleMode.Driver)
    ).onlyIf(
      lambda: RobotBase.isReal() and not utils.isAutonomousMode()
    ).withName(f'Game:RumbleControllers:{ mode.name }:{ pattern.name }')
