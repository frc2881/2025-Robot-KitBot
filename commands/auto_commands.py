from typing import TYPE_CHECKING
from enum import Enum, auto
from commands2 import Command, cmd
from wpilib import SendableChooser, SmartDashboard
from wpimath.geometry import Transform2d
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath
from lib import logger, utils
from lib.classes import Alliance
if TYPE_CHECKING: from robot_container import RobotContainer
import constants

class AutoPath(Enum):
  Move1 = auto()

class AutoCommands:
  def __init__(
      self,
      robot: "RobotContainer"
    ) -> None:
    self._robot = robot

    self._paths = { path: PathPlannerPath.fromPathFile(path.name) for path in AutoPath }

    AutoBuilder.configure(
      self._robot.localizationSubsystem.getPose, 
      self._robot.localizationSubsystem.resetPose, 
      self._robot.driveSubsystem.getChassisSpeeds, 
      self._robot.driveSubsystem.drive, 
      constants.Subsystems.Drive.kPathPlannerController,
      constants.Subsystems.Drive.kPathPlannerRobotConfig,
      lambda: utils.getAlliance() == Alliance.Red,
      self._robot.driveSubsystem
    )

    self._autoCommandChooser = SendableChooser()
    self._autoCommandChooser.setDefaultOption("None", cmd.none)
    self._autoCommandChooser.addOption("[0]_1", self.auto_0_1_)
    SmartDashboard.putData("Robot/Auto/Command", self._autoCommandChooser)

  def getSelected(self) -> Command:
    return self._autoCommandChooser.getSelected()()
  
  def _reset(self, path: AutoPath) -> Command:
    return cmd.sequence(
      AutoBuilder.resetOdom(self._paths.get(path).getPathPoses()[0].transformBy(Transform2d(0, 0, self._paths.get(path).getInitialHeading()))),
      cmd.waitSeconds(0.1)
    )
  
  def _move(self, path: AutoPath) -> Command:
    return cmd.sequence(
      AutoBuilder.followPath(self._paths.get(path))
    ).withTimeout(
      constants.Game.Commands.kAutoMoveTimeout
    )

  def _score(self) -> Command:
    return self._robot.gameCommands.scoreCommand()

  def auto_0_1_(self) -> Command:
    return cmd.sequence(
      self._reset(AutoPath.Move1),
      self._move(AutoPath.Move1),
      self._score()
    ).withName("AutoCommands:[0]_1_")
  