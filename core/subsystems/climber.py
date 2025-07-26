from typing import Callable
from wpilib import SmartDashboard
from commands2 import Subsystem, Command
from rev import SparkMax, SparkMaxConfig, SparkBase
from lib import logger, utils
import core.constants as constants

class Climber(Subsystem):
  def __init__(
      self
      ):
    super().__init__()
    self._constants = constants.Subsystems.Climber
    
    self._motor = SparkMax(self._constants.kClimberMotorCANId, SparkBase.MotorType.kBrushless)
    self._sparkConfig = SparkMaxConfig()
    (self._sparkConfig
      .smartCurrentLimit(self._constants.kClimberMotorCurrentLimit)
      .inverted(True))
    utils.setSparkConfig(
      self._motor.configure(
        self._sparkConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      )
    )

  def periodic(self) -> None:
    self._updateTelemetry()

  def climb(self) -> Command:
    return self.startEnd(
      lambda: self._motor.set(self._constants.kClimberMotorClimbSpeed),
      lambda: self._motor.stopMotor()
    ).withName("Climber:Climb")
  
  def resetClimb(self) -> Command:
    return self.startEnd(
      lambda: self._motor.set(self._constants.kClimberMotorResetSpeed),
      lambda: self._motor.stopMotor()
    ).withName("Climber:Reset")
  
  def reset(self) -> None:
    self._motor.stopMotor()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Climber/Speed", self._motor.get())
