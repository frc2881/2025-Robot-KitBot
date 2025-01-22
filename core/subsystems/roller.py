from typing import Callable
from wpilib import SmartDashboard
from wpimath import units
from commands2 import Subsystem, Command
from rev import SparkMax, SparkMaxConfig, SparkBase
from lib import logger, utils
import core.constants as constants

class RollerSubsystem(Subsystem):
  def __init__(self):
    super().__init__()
    self._constants = constants.Subsystems.Roller
    
    self._rollerMotor = SparkMax(self._constants.kRollerMotorCANId, SparkBase.MotorType.kBrushless)
    self._sparkConfig = SparkMaxConfig()
    (self._sparkConfig
      .smartCurrentLimit(self._constants.kRollerMotorCurrentLimit)
      .inverted(True))
    utils.setSparkConfig(
      self._rollerMotor.configure(
        self._sparkConfig, 
        SparkBase.ResetMode.kResetSafeParameters, 
        SparkBase.PersistMode.kPersistParameters
      )
    )

  def periodic(self) -> None:
    self._updateTelemetry()

  def ejectCommand(self) -> Command:
    return self.run(
      lambda: self._rollerMotor.set(self._constants.kRollerMotorEjectSpeed)
    ).finallyDo(
      lambda end: self.reset()
    ).withName("RollerSubsystem:Eject")
  
  def reset(self) -> None:
    self._rollerMotor.stopMotor()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Roller/Speed", self._rollerMotor.get())