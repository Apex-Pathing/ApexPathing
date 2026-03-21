# Apex Pathing Architecture

## Package Overview

```
com.apexpathing
├── geometry/        Core math types (Pose2d, Vector2d, Vector)
├── kinematics/      Drivetrain kinematics (swerve, mecanum, tank)
├── follower/        Trajectory generation and following
├── drivetrain/      Drive system implementations
├── localization/    Position tracking (Localizer interface + implementations)
├── hardware/        FTC hardware wrappers (MotorEx, LynxModuleUtil)
└── util/
    ├── math/        Pose, CoordinateSystem (ApexCoordinates, PedroCoordinates)
    └── ...          Controllers and rate limiters
```

## Drive System

```mermaid
classDiagram
    class Drivetrain {
        <<abstract>>
        + leftFront, leftRear, rightFront, rightRear: DcMotorEx
        + drive(x, y, turn)
        + driveFieldCentric(x, y, turn, heading)
        + stop()
    }
    class MecanumDrive {
        - constants: MecanumConstants
        + botCentricDrive(x, y, turn)
        + fieldCentricDrive(x, y, turn, heading)
        + calculateDrive(corrective, heading, pathing, robotHeading)
    }
    class TankDrive {
        - fourWheelDrive: boolean
    }
    class CustomDrive {
        <<abstract>>
        # localizer: Localizer
        # controller: HolonomicTrajectoryFollower
        + setDrivePowers(Pose2d)
        + update()
        + followTrajectory(Trajectory)
    }
    class SwerveDrive {
        - modules: SwerveModule[4]
        - kinematics: SwerveKinematics
    }
    class SwerveModule {
        - driveMotor: MotorEx
        - turnServo: Servo
        - absoluteEncoder: AnalogInput
        + setDesiredState(velocity, angle)
    }

    Drivetrain <|-- MecanumDrive
    Drivetrain <|-- TankDrive
    CustomDrive <|-- SwerveDrive
    SwerveDrive *-- SwerveModule
    MecanumDrive --> MecanumConstants
```

## Kinematics

```mermaid
classDiagram
    class KinematicsSwitcher {
        <<interface>>
        + calculate(ChassisSpeeds): Object
    }
    class Kinematics {
        <<abstract>>
    }

    KinematicsSwitcher <|.. Kinematics
    Kinematics <|-- SwerveKinematics
    Kinematics <|-- MecanumKinematics
    Kinematics <|-- TankKinematics
```

## Localization

```mermaid
classDiagram
    class Localizer {
        <<interface>>
        + update()
        + getPose(): Pose2d
        + getVelocity(): Pose2d
        + setPose(Pose2d)
    }
    class PinpointLocalizer {
        - pinpoint: GoBildaPinpointDriver
        + init()
    }

    Localizer <|.. PinpointLocalizer
```

## Math Utilities

```mermaid
classDiagram
    class Pose {
        + x, y, heading: Double
        + coordSystem: CoordinateSystem
        + distanceTo(Pose): Double
        + inCoordinateSystem(CoordinateSystem): Pose
        + rotate(theta): Pose
        + rotated(theta): Pose
        + reflectX(at), reflectY(at)
        + asVector(): Vector
    }
    class CoordinateSystem {
        <<interface>>
        + toApexCoordinates(Pose): Pose
        + fromApexCoordinates(Pose): Pose
    }
    class ApexCoordinates {
        <<object>>
    }
    class PedroCoordinates {
        <<object>>
    }

    CoordinateSystem <|.. ApexCoordinates
    CoordinateSystem <|.. PedroCoordinates
    Pose --> CoordinateSystem
```

## Trajectory Following

```mermaid
classDiagram
    class HolonomicTrajectoryFollower {
        - kinematics: KinematicsSwitcher
        + update(currentPose, target): Object
    }
    class QuinticHermiteSpline {
        + getPoint(t): Vector2d
        + getVelocity(t): Vector2d
        + getAcceleration(t): Vector2d
    }
    class ArcLengthParameterizer {
        + getT(s): double
        + getTotalArcLength(): double
    }

    HolonomicTrajectoryFollower --> TrajectorySample
    ArcLengthParameterizer --> QuinticHermiteSpline
```
