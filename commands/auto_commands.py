from typing import TYPE_CHECKING
from enum import Enum, auto
from commands2 import Command, cmd
from wpilib import SendableChooser, SmartDashboard
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath
from lib import logger, utils
from lib.classes import Alliance
if TYPE_CHECKING: from robot_container import RobotContainer
import constants
from wpimath.geometry import Pose2d, Rotation2d

class AutoPath(Enum):
  Move0 = auto()
  Move2 = auto()

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
    self._autoCommandChooser.addOption("[0] 0_", self.auto_0_)
    self._autoCommandChooser.addOption("[2] 2_", self.auto_2_)
    self._autoCommandChooser.addOption("[0]_1", self.auto_0_1_)
    SmartDashboard.putData("Robot/Auto/Command", self._autoCommandChooser)
    autoPath = self._paths.get(AutoPath.Move2)
    SmartDashboard.putNumber("Robot/Auto/InitialHeading", autoPath.getInitialHeading().degrees())
    # SmartDashboard.putData("Robot/Auto/InitialPose", autoPath.getStartingHolonomicPose())


  def getSelected(self) -> Command:
    return self._autoCommandChooser.getSelected()()
  
  def _resetPoseForAuto(self, autoPath):
    pose: Pose2d = autoPath.getPathPoses()[0]
    # pose = Pose2d(pose.x, pose.y, Rotation2d.fromDegrees(autoPath.getInitialHeading().degrees()))
    # pose = autoPath.getStartingHolonomicPose()
    pose = Pose2d(7.547, 4.074, Rotation2d.fromDegrees(180.000))
    # autoPath.getInitialHeading()
    # pose = Pose2d(pose.x, pose.y, Rotation2d.fromDegrees(180))
    # self._robot.gyroSensor._reset(180)
    self._robot.localizationSubsystem.resetPose(pose)

  def _move(self, path: AutoPath, resetPose: bool = False) -> Command:
    autoPath = self._paths.get(path)
    return cmd.sequence(
      cmd.runOnce(lambda:self._resetPoseForAuto(autoPath)),
      cmd.waitSeconds(0.1),
      AutoBuilder.followPath(autoPath)
    ).withTimeout(
      constants.Game.Commands.kAutoMoveTimeout
    )
    # if resetPose: 
    #   cmd = AutoBuilder.resetOdom(autoPath.getPathPoses()[0]).andThen(cmd)
  
  def _alignToTarget(self) -> Command:
    return cmd.sequence(self._robot.gameCommands.alignRobotToTargetCommand())

  def auto_0_(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.Move0, True)
    ).withName("AutoCommands:[0] 0_")

  def auto_2_(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.Move2),
      self._alignToTarget()
    ).withName("AutoCommands:[2] 2_")
  
  def auto_0_1_(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.Move2, True),
      self._score()
    ).withName("AutoCommands:[0]_1_")
  
  def _score(self) -> Command:
    return self._robot.gameCommands.scoreCommand()

  