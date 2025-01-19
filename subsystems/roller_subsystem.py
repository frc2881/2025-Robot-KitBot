from typing import Callable
import math
from commands2 import Subsystem, Command
from wpimath import units
from rev import SparkMax, SparkMaxConfig, SparkBase
from lib import logger, utils
import constants

class RollerSubsystem(Subsystem):
  def __init__(self):
    super().__init__()
    self._config = constants.Subsystems.Roller
    
    self._rollerMotor = SparkMax(self._config.kRollerMotorCANId, SparkBase.MotorType.kBrushless)
    self._sparkConfig = SparkMaxConfig()
    self._sparkConfig.smartCurrentLimit(self._config.kRollerMotorCurrentLimit)
    self._sparkConfig.inverted(True)
    utils.setSparkConfig(self._rollerMotor.configure(self._sparkConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters))
    
  def stop(self) -> None:
    self._rollerMotor.set(0)

  def ejectCommand(self) -> Command:
    return self.run(
      lambda: self._rollerMotor.set(self._config.kRollerMotorEjectSpeed)
    ).finallyDo(
      lambda end: self.stop()
    ).withName("RollerSubsystem:Roller")