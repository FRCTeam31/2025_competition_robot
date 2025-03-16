// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.dashboard.TeleopDashboardTab;
import frc.robot.dashboard.DashboardSection;
import frc.robot.oi.OperatorInterface;
import frc.robot.oi.BuildableAutoRoutine;
import frc.robot.subsystems.*;
import frc.robot.subsystems.climbing.ClimberSubsystem;
import frc.robot.subsystems.drivetrain.SwerveMap;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.subsystems.climbing.ClimberInputs.ClimberPosition;
import frc.robot.subsystems.climbing.ClimberInputs.HooksPosition;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.endEffector.EndEffectorSubsystem;
import frc.robot.subsystems.endEffector.EndEffectorSubsystem.WristSetpointFromElevatorPosition;
import frc.robot.subsystems.vision.VisionSubsystem;

public class Container {
  public static TeleopDashboardTab TeleopDashboardSection;
  public static DashboardSection CommandsDashboardSection;
  public static DashboardSection AutoDashboardSection;
  public static DashboardSection TestDashboardSection;
  public static BuildableAutoRoutine AutoBuilder;

  public static SwerveSubsystem Swerve;
  public static VisionSubsystem Vision;
  public static ClimberSubsystem Climber;
  public static PwmLEDs LEDs;
  public static EndEffectorSubsystem EndEffector;
  public static ElevatorSubsystem Elevator;
  public static OperatorInterface OperatorInterface;

  public static void initialize(boolean isReal) {
    try {
      // Create subsystems
      LEDs = new PwmLEDs();
      Vision = new VisionSubsystem();
      Swerve = new SwerveSubsystem(isReal);
      Climber = new ClimberSubsystem(isReal);
      EndEffector = new EndEffectorSubsystem(isReal);

      // Register the named commands from each subsystem that may be used in PathPlanner
      // var namedCommandsMap = Swerve.getNamedCommands();
      // ...add other named commands to the map using "otherNamedCommands.putAll(namedCommandsMap);"
      // NamedCommands.registerCommands(namedCommandsMap);

      // Create our custom auto builder
      AutoDashboardSection = new DashboardSection("Auto");
      // AutoBuilder = new BuildableAutoRoutine(namedCommandsMap);
      TeleopDashboardSection = new TeleopDashboardTab();
      CommandsDashboardSection = new DashboardSection("Commands");
      TestDashboardSection = new DashboardSection("Test");

      // Create Elevator Subsystem
      OperatorInterface = new OperatorInterface();
      Elevator = new ElevatorSubsystem(isReal);

      // Configure controller bindings
      // OperatorInterface.bindDriverControls(
      //     Swerve.resetGyroCommand(),
      //     Swerve.enableLockOnCommand(),
      //     Swerve.disableAutoAlignCommand(),
      //     Swerve::setAutoAlignSetpointCommand,
      //     Swerve::setDefaultCommand,
      //     Swerve::driveFieldRelativeCommand);

      OperatorInterface.bindDriverControls(Swerve, Climber);
      OperatorInterface.bindOperatorControls(Elevator, EndEffector);

    } catch (Exception e) {
      DriverStation.reportError("[ERROR] >> Failed to initialize Container: " + e.getMessage(), e.getStackTrace());
    }
  }

  //#region Commands

  public static Command setCombinedHeightAndAngle(ElevatorPosition position, WristSetpointFromElevatorPosition angle) {
    return Commands.parallel(
        Commands.runOnce(() -> System.out.println("Setting combined height and angle: " + position)),
        Elevator.setElevatorSetpointCommand(position),
        EndEffector.scheduleWristSetpointCommand(angle))
        .finallyDo(() -> System.out.println("Finished setting combined setpoints"));
  }

  //#endregion
}
