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
        super()._init_()
        self._rollerMotor = SparkMax(constants.Subystems.Roller.kRollerMotorCANId, SparkBase.MotorType.kBrushless)

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

