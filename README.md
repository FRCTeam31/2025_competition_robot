# Prime Movers 2025 Competition Robot
This is the code for the Prime Movers 2025 FRC Competition Robot, which will play the game [REEFSCAPE](https://www.firstinspires.org/robotics/frc/game-and-season).

### [Game Manual](https://firstfrc.blob.core.windows.net/frc2025/Manual/2025GameManual.pdf)
### [Game animation](https://www.youtube.com/watch?v=YWbxcjlY9JY)
[![REEFSCAPE Animation Video](https://img.youtube.com/vi/YWbxcjlY9JY/0.jpg)](https://www.youtube.com/watch?v=YWbxcjlY9JY)

## Project Tracking
The Programming team's work is tracked in [this GitHub Project](https://github.com/orgs/FRCTeam31/projects/2/views/1?pane=info), each with one of the below statuses:
- _Planning_ - We're planning how we want to code this
- _Active_ - Coding is in progress
- _Awaiting Mech/Elec_ - Progress is blocked by a mechanical or electrical problem
- _Needs Testing_ - Initial code is complete and needs to be tested on the real robot.
- _Bugged_ - There is a bug in this code
- _In Review_ - This code needs to be reviewed by a coach
- _Done_ - This is code-complete, tested, and merged into the `main` branch

## Code conventions
These rules apply to all code committed to this repository.
- Package names are always lowercase.
- `public` or `static` variables are named in `PascalCase`.
- `private` variables are prefixed with an underscore, and named in `_camelCase`.
- Any function with greater than 8 parameters must use a [DTO class](https://www.baeldung.com/java-dto-pattern) to wrap the parameters
    - These DTOs must be suffixed with `Parameters`

## Subsystem conventions
These rules apply to subsystems in the robot, unless otherwise approved by a coach.
- Subsystems must follow the IO pattern.
    - Must include an `IO` interface.
    - Must include an `IOReal` class, which inherits from the interface.
    - Must include an `IOSim` class, which inherits from the interface.
    - Must include an `IOInputs` class.
    - Must include an `IOOutputs` class.
- Subsystems must include a member `<Subsystem>Map` class, containing only `public static final` values.
    - Values in the `Map` are values which are recorded for configuration of the robot, and are changed _very_ rarely.
- Subsystems are located in the `subsystems` package, in their own package.

## Software required to use this repository
- [WPILib 2025](https://github.com/wpilibsuite/allwpilib/releases)
- [RevLib](https://docs.revrobotics.com/revlib/install#c-and-java-installation)
- [CTRE Phoenix 5 and 6](https://docs.ctr-electronics.com/)
- [PathPlanner](https://github.com/mjansen4857/pathplanner)

## Resources
- [WPILib Documentation](https://docs.wpilib.org/en/latest/docs/zero-to-robot/introduction.html)
- [RevLib Java Documentation](https://docs.revrobotics.com/revlib)
- [CTRE Phoenix 6 Documentation](https://v6.docs.ctr-electronics.com/en/stable/)
    - [Examples](https://github.com/CrossTheRoadElec/Phoenix6-Examples/tree/main/java)
- [PathPlanner Docs](https://pathplanner.dev/home.html)
    - [Examples](https://github.com/mjansen4857/pathplanner/tree/main/examples/java)