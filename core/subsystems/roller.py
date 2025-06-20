from typing import Callable
from wpilib import SmartDashboard
from commands2 import Subsystem, Command
from rev import SparkMax, SparkMaxConfig, SparkBase
from lib import logger, utils
import core.constants as constants

class Roller(Subsystem):
  def __init__(
      self,
      intakeSensorHasTarget: Callable[[], bool]
      ):
    super().__init__()
    self._constants = constants.Subsystems.Roller

    self._intakeSensorHasTarget = intakeSensorHasTarget
    
    self._motor = SparkMax(self._constants.kRollerMotorCANId, SparkBase.MotorType.kBrushless)
    self._sparkConfig = SparkMaxConfig()
    (self._sparkConfig
      .smartCurrentLimit(self._constants.kRollerMotorCurrentLimit)
      .inverted(True))
    utils.setSparkConfig(
      self._motor.configure(
        self._sparkConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      )
    )

    self.setDefaultCommand(self.hold())

  def periodic(self) -> None:
    self._updateTelemetry()

  def score(self) -> Command:
    return self.startEnd(
      lambda: self._motor.set(self._constants.kRollerMotorScoreSpeed),
      lambda: self._motor.stopMotor()
    ).withName("Roller:Score")
  
  def resetCoral(self) -> Command:
    return self.startEnd(
      lambda: self._motor.set(self._constants.kRollerMotorResetSpeed),
      lambda: self._motor.stopMotor()
    ).withName("Roller:Reset")
  
  def hold(self) -> Command:
    return self.runEnd(
      lambda: self._motor.set(self._constants.kRollerMotorHoldSpeed if self.isIntakeHolding() else 0),
      lambda: self._motor.stopMotor()
    ).withName("Roller:Hold")
  
  def isIntakeHolding(self) -> bool:
    return self._intakeSensorHasTarget()
  
  def reset(self) -> None:
    self._motor.stopMotor()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Roller/IsHolding", self.isIntakeHolding())
    SmartDashboard.putNumber("Robot/Roller/Speed", self._motor.get())
