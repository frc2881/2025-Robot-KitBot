from typing import Callable
from commands2 import Subsystem, Command
from wpimath import units
import constants
from rev import SparkMax, SparkMaxConfig, SparkBase

class RollerSubsystem:
    def __init__(self):
        super().__init__()
        self._rollerMotor = SparkMax (constants.Subsystems.Roller.kRollerMotorCANId, SparkBase.MotorType.kBrushless)

        self.sparkConfig = SparkMaxConfig()
        self.sparkConfig.voltageCompensation(constants.Subsystems.Roller.kRollerMotorVComp)
        self.sparkConfig.smartCurrentLimit(constants.Subsystems.Roller.kRollerMotorCurrentLimit)
        self._rollerMotor.configure(self.sparkConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters)
    
    def rollerCommand(
        self, 
        getForward: Callable[[], units.percent], 
        getReverse: Callable[[], units.percent] 
        ) -> Command:
        return self.run(
        lambda: self._rollerMotor.set(getForward() - getReverse())
        ).withName("RollerSubsystem:Roller")