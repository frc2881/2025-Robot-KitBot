from typing import TYPE_CHECKING
from enum import Enum, auto
from commands2 import Command, cmd
from wpilib import SendableChooser, SmartDashboard
from wpimath.geometry import Transform2d
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath
from lib import logger, utils
from lib.classes import Alliance, TargetAlignmentMode
if TYPE_CHECKING: from core.robot import RobotCore
from core.classes import TargetAlignmentLocation
import core.constants as constants

class AutoPath(Enum):
  Start_2R_2R = auto()
  Start_2L_2L = auto()
  Start_3_3L = auto()
  Intake_3L_B = auto()
  Score_B_4R = auto()
  Intake_4R_B = auto()
  Score_B_4L = auto()
  Start_1_1R = auto()
  Intake_1R_A = auto()
  Score_A_6L = auto()
  Intake_6L_A = auto()
  Score_A_6R = auto()
  Move_2R_A = auto()
  Move_2L_B = auto()

class Auto:
  def __init__(
      self,
      robot: "RobotCore"
    ) -> None:
    self._robot = robot

    self._paths = { path: PathPlannerPath.fromPathFile(path.name) for path in AutoPath }
    self._auto = cmd.none()

    AutoBuilder.configure(
      self._robot.localization.getRobotPose, 
      self._robot.localization.resetRobotPose,
      self._robot.drive.getChassisSpeeds, 
      self._robot.drive.setChassisSpeeds, 
      constants.Subsystems.Drive.kPathPlannerController,
      constants.Subsystems.Drive.kPathPlannerRobotConfig,
      lambda: utils.getAlliance() == Alliance.Red,
      self._robot.drive
    )

    self._autos = SendableChooser()
    self._autos.setDefaultOption("None", cmd.none)
    
    self._autos.addOption("[1]", self.auto_1)
    self._autos.addOption("[2L]", self.auto_2L)
    self._autos.addOption("[2R]", self.auto_2R)
    self._autos.addOption("[3]", self.auto_3)

    self._autos.onChange(lambda auto: self.set(auto()))
    SmartDashboard.putData("Robot/Auto", self._autos)

  def get(self) -> Command:
    return self._auto
  
  def set(self, auto: Command) -> None:
    self._auto = auto
  
  def _reset(self, path: AutoPath) -> Command:
    return (
      AutoBuilder.resetOdom(self._paths.get(path).getPathPoses()[0].transformBy(Transform2d(0, 0, self._paths.get(path).getInitialHeading())))
      .andThen(cmd.waitSeconds(0.1))
    )
  
  def _move(self, path: AutoPath) -> Command:
    return (
      AutoBuilder.followPath(self._paths.get(path))
      .deadlineFor(logger.log_(f'Auto:Move:{path.name}'))
    )

  def _alignToTarget(self, targetAlignmentLocation: TargetAlignmentLocation) -> Command:
    return (
      self._robot.game.alignRobotToTarget(TargetAlignmentMode.Translation, targetAlignmentLocation)
      .withTimeout(constants.Game.Commands.kAutoTargetAlignmentTimeout)
      .deadlineFor(logger.log_(f'Auto:AlignToTarget:{targetAlignmentLocation.name}'))
    )

  def _moveAlignScore(self, autoPath: AutoPath, targetAlignmentLocation: TargetAlignmentLocation) -> Command:
    return (
      self._move(autoPath)
      .andThen(self._alignToTarget(targetAlignmentLocation))
      .deadlineFor(self._robot.roller.hold())
      .andThen(
        self._robot.game.scoreCoral()
        .deadlineFor(logger.log_("Auto:ScoreCoral"))
      )
    )

  def _moveAlignIntake(self, autoPath: AutoPath, targetAlignmentLocation: TargetAlignmentLocation) -> Command:
    return (
      self._move(autoPath)
      .andThen(self._alignToTarget(targetAlignmentLocation))
      .andThen(
        self._robot.game.intakeCoral()
        .deadlineFor(logger.log_("Auto:IntakeCoral"))
      )
    )

  def auto_1(self) -> Command:
    return cmd.sequence(
      self._moveAlignScore(AutoPath.Start_1_1R, TargetAlignmentLocation.Right),
      self._moveAlignIntake(AutoPath.Intake_1R_A, TargetAlignmentLocation.Right), 
      self._moveAlignScore(AutoPath.Score_A_6L, TargetAlignmentLocation.Left),
      self._moveAlignIntake(AutoPath.Intake_6L_A, TargetAlignmentLocation.Right),
      self._moveAlignScore(AutoPath.Score_A_6R, TargetAlignmentLocation.Right)
    ).withName("Auto:[1]")

  def auto_2L(self) -> Command:
    return cmd.sequence(
      self._moveAlignScore(AutoPath.Start_2L_2L, TargetAlignmentLocation.Left),
      cmd.waitSeconds(3.0),
      self._move(AutoPath.Move_2L_B)
    ).withName("Auto:[2L]")

  def auto_2R(self) -> Command:
    return cmd.sequence(
      self._moveAlignScore(AutoPath.Start_2R_2R, TargetAlignmentLocation.Right),
      cmd.waitSeconds(3.0),
      self._move(AutoPath.Move_2R_A)
    ).withName("Auto:[2R]")
  
  def auto_3(self) -> Command:
    return cmd.sequence(
      self._moveAlignScore(AutoPath.Start_3_3L, TargetAlignmentLocation.Left),
      self._moveAlignIntake(AutoPath.Intake_3L_B, TargetAlignmentLocation.Left), 
      self._moveAlignScore(AutoPath.Score_B_4R, TargetAlignmentLocation.Right),
      self._moveAlignIntake(AutoPath.Intake_4R_B, TargetAlignmentLocation.Left),
      self._moveAlignScore(AutoPath.Score_B_4L, TargetAlignmentLocation.Left),
      self._moveAlignIntake(AutoPath.Intake_4R_B, TargetAlignmentLocation.Left),
      self._moveAlignScore(AutoPath.Score_B_4L, TargetAlignmentLocation.Left)
    ).withName("Auto:[3]")