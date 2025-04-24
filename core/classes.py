from enum import Enum, auto
from dataclasses import dataclass
from wpimath import units
from wpimath.geometry import Pose2d, Pose3d

class TargetType(Enum):
  Reef = auto()
  CoralStation = auto()

class TargetPositionType(Enum):
  ReefCoralL1 = auto()
  CoralStation = auto()

class TargetAlignmentLocation(Enum):
  Center = auto()
  Left = auto()
  Right = auto()

@dataclass(frozen=True, slots=True)
class Target():
  type: TargetType
  pose: Pose3d

@dataclass(frozen=False, slots=True)
class TargetAlignmentInfo:
  pose: Pose2d
  distance: units.meters
  heading: units.degrees
  pitch: units.degrees
