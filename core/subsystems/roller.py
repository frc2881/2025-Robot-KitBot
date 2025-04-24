from wpilib import SmartDashboard
from commands2 import Subsystem, Command
from rev import SparkMax, SparkMaxConfig, SparkBase
from lib import logger, utils
import core.constants as constants

class Roller(Subsystem):
  def __init__(self):
    super().__init__()
    self._constants = constants.Subsystems.Roller
    
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

  def periodic(self) -> None:
    self._updateTelemetry()

  def eject(self) -> Command:
    return self.run(
      lambda: self._motor.set(self._constants.kRollerMotorEjectSpeed)
    ).finallyDo(
      lambda end: self._motor.stopMotor()
    ).withName("Roller:Eject")
  
  def reset(self) -> None:
    self._motor.stopMotor()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Roller/Speed", self._motor.get())