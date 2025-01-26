from typing import TYPE_CHECKING
from enum import Enum, auto
from commands2 import Command, cmd
from wpilib import SendableChooser, SmartDashboard
from wpimath.geometry import Transform2d, Pose2d, Rotation2d
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath
from lib import logger, utils
from lib.classes import Alliance, TargetAlignmentMode
if TYPE_CHECKING: from core.robot import RobotCore
from core.classes import TargetAlignmentLocation
import core.constants as constants

class AutoPath(Enum):
  Move1 = auto()
  Move2 = auto()
  Pickup1 = auto()
  Pickup2 = auto()

class AutoCommands:
  def __init__(
      self,
      robot: "RobotCore"
    ) -> None:
    self._robot = robot

    self._paths = { path: PathPlannerPath.fromPathFile(path.name) for path in AutoPath }

    AutoBuilder.configure(
      self._robot.localizationService.getRobotPose, 
      self._robot.localizationService.resetRobotPose, 
      self._robot.driveSubsystem.getChassisSpeeds, 
      self._robot.driveSubsystem.drive, 
      constants.Subsystems.Drive.kPathPlannerController,
      constants.Subsystems.Drive.kPathPlannerRobotConfig,
      lambda: utils.getAlliance() == Alliance.Red,
      self._robot.driveSubsystem
    )

    self._autoCommandChooser = SendableChooser()
    self._autoCommandChooser.setDefaultOption("None", cmd.none)
    self._autoCommandChooser.addOption("[0]_1_", self.auto_0_1_())
    SmartDashboard.putData("Robot/Auto/Command", self._autoCommandChooser)

  def getSelected(self) -> Command:
    return self._autoCommandChooser.getSelected()
  
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
  
  def _alignToTarget(self, targetAlignmentLocation: TargetAlignmentLocation) -> Command:
    return cmd.sequence(self._robot.gameCommands.alignRobotToTargetCommand(TargetAlignmentMode.Translation, targetAlignmentLocation))

  def _score(self) -> Command:
    return self._robot.gameCommands.scoreCommand()

  def auto_0_1_(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.Move1),
      self._alignToTarget(TargetAlignmentLocation.Center),
      self._score(),
      self._move(AutoPath.Pickup1),
      self._alignToTarget(TargetAlignmentLocation.Left),
      cmd.waitSeconds(0.75),
      self._move(AutoPath.Move2),
      self._alignToTarget(TargetAlignmentLocation.Left),
      self._score(),
      self._move(AutoPath.Pickup2),
      self._alignToTarget(TargetAlignmentLocation.Right),
      cmd.waitSeconds(0.75),
      self._move(AutoPath.Move2),
      self._alignToTarget(TargetAlignmentLocation.Right),
      self._score()
    ).withName("AutoCommands:[0]_1_")
  
  def autoTest(self) -> Command:
    return AutoBuilder.pathfindToPose(
      Pose2d(15.7, 5.9, Rotation2d().fromDegrees(-120)),
      constants.Subsystems.Drive.kPathPlannerConstraints
    )
  
  def autoTeleop(self) -> Command:
    return cmd.sequence(
      self.autoTest(),
      self._alignToTarget(TargetAlignmentLocation.Right),
      cmd.waitSeconds(.5),
      AutoBuilder.pathfindToPose(
        Pose2d(10.9, 4.1, Rotation2d().fromDegrees(0)),
        constants.Subsystems.Drive.kPathPlannerConstraints
      ),
      self._alignToTarget(TargetAlignmentLocation.Left),
      self._score(),
      self.autoTest(),
      self._alignToTarget(TargetAlignmentLocation.Right),
      cmd.waitSeconds(.5),
      AutoBuilder.pathfindToPose(
        Pose2d(10.9, 4.1, Rotation2d().fromDegrees(0)),
        constants.Subsystems.Drive.kPathPlannerConstraints
      ),
      self._alignToTarget(TargetAlignmentLocation.Right),
      self._score()

    )
  