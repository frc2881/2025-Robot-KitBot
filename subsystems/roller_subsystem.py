from typing import Callable
import math
from commands2 import Subsystem, Command
from wpilib import SmartDashboard, SendableChooser
from wpimath import units
from wpimath.controller import PIDController
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Rotation2d, Pose2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition, SwerveModuleState, SwerveDrive4Kinematics
from pathplannerlib.util import DriveFeedforwards
from lib import logger, utils
from lib.classes import MotorIdleMode, SpeedMode, DriveOrientation, OptionState, LockState
from lib.components.swerve_module import SwerveModule
import constants
from rev import SparkMax, SparkMaxConfig, SparkBase

class RollerSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        self.config = constants.Subsystems.Roller
        self._rollerMotor = SparkMax(self.config.kRollerMotorCANId, SparkBase.MotorType.kBrushless)
        self.sparkConfig = SparkMaxConfig()
        self.sparkConfig.inverted(True)
        self.sparkConfig.voltageCompensation(self.config.kRollerMotorVComp)
        self.sparkConfig.smartCurrentLimit(self.config.kRollerMotorCurrentLimit)
        utils.setSparkConfig(self._rollerMotor.configure(self.sparkConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters))
        
    def stop(self) -> None:
        self._rollerMotor.set(0)

    def ejectCommand(self) -> Command:
        return self.run(
            lambda: self._rollerMotor.set(self.config.kRollerMotorEjectSpeed)
        ).finallyDo(lambda _: self.stop()).withName("RollerSubsystem:Roller")