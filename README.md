# Apex Pathing

FTC path following library with swerve drive support.

## Installation

Add JitPack to your `build.gradle` repositories:

```gradle
repositories {
    maven { url 'https://jitpack.io' }
}
```

Add the dependency:

```gradle
dependencies {
    implementation 'com.github.MooseX-Pathing:ApexPathing:TAG'
}
```

Replace `TAG` with a release tag or commit hash.

## Packages

| Package | Contents |
|---|---|
| `com.apexpathing.geometry` | Pose2d, Vector2d |
| `com.apexpathing.kinematics` | ChassisSpeeds, Swerve/Mecanum/TankKinematics |
| `com.apexpathing.follower` | HolonomicTrajectoryFollower, QuinticHermiteSpline, ArcLengthParameterizer |
| `com.apexpathing.drivetrain` | CustomDrive, SwerveDrive, SwerveModule |
| `com.apexpathing.localization` | Localizer interface, PinpointLocalizer, FieldCoordinates |
| `com.apexpathing.hardware` | MotorEx, LynxModuleUtil |
| `com.apexpathing.util` | PIDFController, SlewRateLimiter |

## Usage

```java
import com.apexpathing.drivetrain.SwerveDrive;
import com.apexpathing.localization.PinpointLocalizer;
import com.apexpathing.geometry.Vector2d;
```

## License

See [LICENSE](LICENSE) for details.
