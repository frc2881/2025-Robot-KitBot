import math
from wpimath import units
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Translation2d, Rotation2d
from wpimath.kinematics import SwerveDrive4Kinematics
import wpilib
from robotpy_apriltag import AprilTagFieldLayout
from navx import AHRS
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController, PIDConstants
from pathplannerlib.pathfinding import PathConstraints
from photonlibpy.photonPoseEstimator import PoseStrategy
from rev import SparkLowLevel
from lib import logger, utils
from lib.classes import (
  Alliance, 
  PID, 
  Tolerance, 
  SwerveModuleConstants, 
  SwerveModuleConfig, 
  SwerveModuleLocation, 
  DriftCorrectionConstants, 
  TargetAlignmentConstants,
  PoseSensorConstants,
  PoseSensorConfig
)
from core.classes import Target, TargetType, TargetAlignmentLocation

APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout(f'{ wpilib.getDeployDirectory() }/localization/2025-reefscape-andymark-filtered.json')
PATHPLANNER_ROBOT_CONFIG = RobotConfig.fromGUISettings()

class Subsystems:
  class Drive:
    kRobotWidth: units.meters = units.inchesToMeters(31.5)
    kRobotLength: units.meters = units.inchesToMeters(34.5)
    kTrackWidth: units.meters = units.inchesToMeters(21.5)
    kWheelBase: units.meters = units.inchesToMeters(24.5)

    kTranslationSpeedMax: units.meters_per_second = 4.46
    kRotationSpeedMax: units.radians_per_second = 4 * math.pi

    kInputLimitDemo: units.percent = 0.5
    kInputRateLimitDemo: units.percent = 0.33

    _swerveModuleConstants = SwerveModuleConstants(
      wheelDiameter = units.inchesToMeters(3.0),
      wheelBevelGearTeeth = 45,
      wheelSpurGearTeeth = 22,
      wheelBevelPinionTeeth = 15,
      drivingMotorPinionTeeth = 13,
      drivingMotorFreeSpeed = 5676,
      drivingMotorControllerType = SparkLowLevel.SparkModel.kSparkMax,
      drivingMotorType = SparkLowLevel.MotorType.kBrushless,
      drivingMotorCurrentLimit = 80,
      drivingMotorPID = PID(0.04, 0, 0),
      turningMotorCurrentLimit = 20,
      turningMotorPID = PID(1.0, 0, 0)
    )

    kSwerveModuleConfigs: tuple[SwerveModuleConfig, ...] = (
      SwerveModuleConfig(SwerveModuleLocation.FrontLeft, 2, 3, -math.pi / 2, Translation2d(kWheelBase / 2, kTrackWidth / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.FrontRight, 4, 5, 0, Translation2d(kWheelBase / 2, -kTrackWidth / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.RearLeft, 6, 7, math.pi, Translation2d(-kWheelBase / 2, kTrackWidth / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.RearRight, 8, 9, math.pi / 2, Translation2d(-kWheelBase / 2, -kTrackWidth / 2), _swerveModuleConstants)
    )

    kDriveKinematics = SwerveDrive4Kinematics(*(c.translation for c in kSwerveModuleConfigs))

    kPathPlannerRobotConfig = PATHPLANNER_ROBOT_CONFIG
    kPathPlannerController = PPHolonomicDriveController(PIDConstants(5.0, 0, 0), PIDConstants(5.0, 0, 0))

    kDriftCorrectionConstants = DriftCorrectionConstants(
      rotationPID = PID(0.01, 0, 0), 
      rotationTolerance = Tolerance(0.5, 1.0)
    )

    kTargetAlignmentConstants = TargetAlignmentConstants(
      translationPID = PID(5.0, 0, 0),
      translationTolerance = Tolerance(0.05, 0.1),
      translationSpeedMax = kTranslationSpeedMax * 0.2,   
      rotationPID = PID(0.1, 0, 0),
      rotationTolerance = Tolerance(2.0, 4.0),
      rotationSpeedMax = kRotationSpeedMax * 0.2, 
      rotationHeadingModeOffset = 0,
      rotationTranslationModeOffset = 180
    )

  class Roller:
    kRollerMotorCANId: int = 10
    kRollerMotorCurrentLimit: int = 60
    kRollerMotorScoreSpeed: float = 0.25
    kRollerMotorResetSpeed: float = -0.2
    kRollerMotorHoldSpeed: float = 0.08

class Services:
  class Localization:
    kVisionMaxTargetDistance: units.meters = 4.0
    kVisionMaxPoseAmbiguity: units.percent = 0.2
    kRobotPoseMaxGroundPlaneDelta: units.meters = 0.25

class Sensors: 
  class Gyro:
    class NAVX2:
      kComType = AHRS.NavXComType.kUSB1

  class Binary:
    class Intake:
      kChannel = 9

  class Pose:
    _poseSensorConstants = PoseSensorConstants(
      aprilTagFieldLayout = APRIL_TAG_FIELD_LAYOUT,
      poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      fallbackPoseStrategy = PoseStrategy.LOWEST_AMBIGUITY
    )

    kPoseSensorConfigs: tuple[PoseSensorConfig, ...] = (
      PoseSensorConfig(
        "Front",
        Transform3d(
          Translation3d(units.inchesToMeters(9.0), units.inchesToMeters(1.5), units.inchesToMeters(18.5)),
          Rotation3d(units.degreesToRadians(0), units.degreesToRadians(11.0), units.degreesToRadians(0))
        ), _poseSensorConstants
      ),
      PoseSensorConfig(
        "Rear",
        Transform3d(
          Translation3d(units.inchesToMeters(-13.0), units.inchesToMeters(-6.5), units.inchesToMeters(27.75)),
          Rotation3d(units.degreesToRadians(0), units.degreesToRadians(-10.0), units.degreesToRadians(180.0))
        ), _poseSensorConstants
      ),
    )

  class Camera:
    kStreams: dict[str, str] = {
      "Front": "http://10.99.91.6:1184/?action=stream",
      "Driver": "http://10.99.91.6:1184/?action=stream"
    }

class Controllers:
  kDriverControllerPort: int = 0
  kOperatorControllerPort: int = 1
  kInputDeadband: units.percent = 0.1

class Game: 
  class Commands:
    kScoreEndingDelay: units.seconds = 0.5
    kAutoTargetAlignmentTimeout: units.seconds = 1.5

  class Field:
    kAprilTagFieldLayout = APRIL_TAG_FIELD_LAYOUT
    kLength = APRIL_TAG_FIELD_LAYOUT.getFieldLength()
    kWidth = APRIL_TAG_FIELD_LAYOUT.getFieldWidth()
    kBounds = (Translation2d(0, 0), Translation2d(kLength, kWidth))

    class Targets:
      kTargets: dict[Alliance, dict[int, Target]] = {
        Alliance.Red: {
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(1).toPose2d()): Target(TargetType.CoralStation, APRIL_TAG_FIELD_LAYOUT.getTagPose(1)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(2).toPose2d()): Target(TargetType.CoralStation, APRIL_TAG_FIELD_LAYOUT.getTagPose(2)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(6).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(6)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(7).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(7)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(8).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(8)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(9).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(9)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(10).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(10)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(11).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(11))
        },
        Alliance.Blue: {
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(12).toPose2d()): Target(TargetType.CoralStation, APRIL_TAG_FIELD_LAYOUT.getTagPose(12)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(13).toPose2d()): Target(TargetType.CoralStation, APRIL_TAG_FIELD_LAYOUT.getTagPose(13)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(17).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(17)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(18).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(18)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(19).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(19)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(20).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(20)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(21).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(21)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(22).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(22))
        }
      }

      kTargetAlignmentTransforms: dict[TargetType, dict[TargetAlignmentLocation, Transform3d]] = {
        TargetType.Reef: {
          TargetAlignmentLocation.Center: Transform3d(units.inchesToMeters(16.5), 0, 0, Rotation3d(Rotation2d.fromDegrees(0))),
          TargetAlignmentLocation.Left: Transform3d(units.inchesToMeters(16.5), units.inchesToMeters(-10.0), 0, Rotation3d(Rotation2d.fromDegrees(0))),
          TargetAlignmentLocation.Right: Transform3d(units.inchesToMeters(16.5), units.inchesToMeters(10.0), 0, Rotation3d(Rotation2d.fromDegrees(0)))
        },
        TargetType.CoralStation: {
          TargetAlignmentLocation.Center: Transform3d(units.inchesToMeters(15), 0, 0, Rotation3d(Rotation2d.fromDegrees(180))),
          TargetAlignmentLocation.Left: Transform3d(units.inchesToMeters(15), units.inchesToMeters(-24), 0, Rotation3d(Rotation2d.fromDegrees(180))),
          TargetAlignmentLocation.Right: Transform3d(units.inchesToMeters(15), units.inchesToMeters(24), 0, Rotation3d(Rotation2d.fromDegrees(180)))
        }
      }
