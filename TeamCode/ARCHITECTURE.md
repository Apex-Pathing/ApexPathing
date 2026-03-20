# Pathing Lib Structure

## Drive System
```mermaid
classDiagram
    class Drivetrain {
        <<interface>>
        +hwManager: HardwareManager
    }
    class HardwareManager {
        - fl, bl, fr, br motors
        - Stuff that u need for directly interacting with the motors
        - Maybe put PID systems for runToVel if needed?
        +setPower methods
    }
    class MotorManager {
        impl: HardwareManager
        - Uses motors and only motors
        - For mec, tank
    }
    class MecanumDrivetrain {
        impl: Drivetrain
        +hwManager: Motormanager
        - Normal Mec Drive
    }
    class SwerveDrivetrain {
        impl: Drivetrain
        +hwManager: SwervePods
        - Swerve Drive, prob input coax/diffy in constructor
    }
    class SwervePods {
        impl: HardwareManager
        - Thingy to manage swerve pods
        - Maay have to make ts into an inf and have diffy vs coax
    }
    class TankDrivetrain {
        impl: Drivetrain
        +hwManager: MotorManager
        - Tank Drive
    }

    Drivetrain <|.. MecanumDrivetrain : Mec Drive
    HardwareManager --> Drivetrain : DT has hw managing system
    Drivetrain <|.. SwerveDrivetrain : Swerve Drive
    HardwareManager <|.. MotorManager : Motor HW System
    HardwareManager <|.. SwervePods : Swerve HW System
    
```
