from typing import TYPE_CHECKING
from enum import Enum, auto
from commands2 import Command, cmd
from wpilib import SendableChooser, SmartDashboard
from pathplannerlib.auto import AutoBuilder, NamedCommands
from lib import logger, utils
from lib.classes import Alliance, TargetAlignmentMode
if TYPE_CHECKING: from core.robot import RobotCore
from core.classes import TargetAlignmentLocation
import core.constants as constants

class AutoName(Enum):
  Default = auto()
  Auto2_2 = auto()

class CommandName(Enum):
  AlignCenter = auto()
  AlignLeft = auto()
  AlignRight = auto()
  Intake = auto()
  Score = auto()

class Auto:
  def __init__(
      self,
      robot: "RobotCore"
    ) -> None:
    self._robot = robot

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

    NamedCommands.registerCommand(CommandName.AlignCenter.name, self._alignToTarget(TargetAlignmentLocation.Center))
    NamedCommands.registerCommand(CommandName.AlignLeft.name, self._alignToTarget(TargetAlignmentLocation.Left))
    NamedCommands.registerCommand(CommandName.AlignRight.name, self._alignToTarget(TargetAlignmentLocation.Right))
    NamedCommands.registerCommand(CommandName.Intake.name, self._intake())
    NamedCommands.registerCommand(CommandName.Score.name, self._score())

    self._autos = SendableChooser()
    self._autos.setDefaultOption("None", AutoName.Default)
    
    self._autos.addOption("[2]_2", AutoName.Auto2_2)

    self._autos.onChange(lambda auto: self._set(auto))
    SmartDashboard.putData("Robot/Auto", self._autos)

  def _set(self, auto: AutoName) -> None:
    self._auto = AutoBuilder.buildAuto(auto.name) if auto != AutoName.Default else cmd.none()

  def get(self) -> Command:
    return self._auto
  
  def _alignToTarget(self, targetAlignmentLocation: TargetAlignmentLocation) -> Command:
    return self._robot.game.alignRobotToTarget(TargetAlignmentMode.Translation, targetAlignmentLocation)
  
  def _intake(self) -> Command:
    return cmd.waitSeconds(2.0) # TODO: replace with intake sensor once installed and enabled

  def _score(self) -> Command:
    return self._robot.game.scoreCoral()
 