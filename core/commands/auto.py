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
  Auto2R_2L = auto()

class CommandName(Enum):
  IntakeCoral = auto()
  ScoreCoral = auto()

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

    NamedCommands.registerCommand(CommandName.IntakeCoral.name, self._intakeCoral())
    NamedCommands.registerCommand(CommandName.ScoreCoral.name, self._scoreCoral())

    self._autos = SendableChooser()
    self._autos.setDefaultOption("None", AutoName.Default)
    
    self._autos.addOption("[2R]_2L", AutoName.Auto2R_2L)

    self._autos.onChange(lambda auto: self._set(auto))
    SmartDashboard.putData("Robot/Auto", self._autos)

  def _set(self, auto: AutoName) -> None:
    self._auto = AutoBuilder.buildAuto(auto.name) if auto != AutoName.Default else cmd.none()

  def get(self) -> Command:
    return self._auto
  
  def _intakeCoral(self) -> Command:
    return self._robot.game.intakeCoral()

  def _scoreCoral(self) -> Command:
    return self._robot.game.scoreCoral()
 