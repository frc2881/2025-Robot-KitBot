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
    return (
      self._robot.drive.alignToTarget(
        self._robot.localization.getRobotPose, 
        lambda: self._robot.localization.getTargetPose(targetAlignmentLocation),
        targetAlignmentMode)
      .andThen(self.rumbleControllers(ControllerRumbleMode.Driver))
      .withName(f'Game:AlignRobotToTarget:{ targetAlignmentMode.name }:{ targetAlignmentLocation.name }')
    )
  
  def intakeCoral(self) -> Command:
    return (
      cmd.waitUntil(lambda: self.isIntakeHolding())
      .andThen(self.rumbleControllers(ControllerRumbleMode.Driver))
      .withName("GameCommands:IntakeCoral")
    )

  def scoreCoral(self) -> Command:
    return (
      self._robot.roller.score()
      .raceWith(
        cmd.waitUntil(lambda: not self.isIntakeHolding())
        .andThen(cmd.waitSeconds(constants.Game.Commands.kScoreEndingDelay))
      )
      .andThen(self.rumbleControllers(ControllerRumbleMode.Driver))
      .withName("GameCommands:ScoreCoral")
    )

  def isIntakeHolding(self) -> bool:
    return self._robot.intakeSensor.hasTarget()

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
