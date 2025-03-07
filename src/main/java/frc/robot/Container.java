// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.prime.control.Controls;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.dashboard.TeleopDashboardTab;
import frc.robot.dashboard.DashboardSection;
import frc.robot.oi.OperatorInterface;
import frc.robot.oi.BuildableAutoRoutine;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivetrain.SwerveMap;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.vision.VisionSubsystem;

public class Container {
  public static TeleopDashboardTab TeleopDashboardSection;
  public static DashboardSection CommandsDashboardSection;
  public static DashboardSection AutoDashboardSection;
  public static DashboardSection TestDashboardSection;
  public static BuildableAutoRoutine AutoBuilder;

  // public static SwerveSubsystem Swerve;
  public static ElevatorSubsystem Elevator;
  public static VisionSubsystem Vision;
  public static PwmLEDs LEDs;
  public static OperatorInterface OperatorInterface;

  public static void initialize(boolean isReal) {
    try {
      // Create subsystems
      LEDs = new PwmLEDs();
      Vision = new VisionSubsystem();
      // Swerve = new SwerveSubsystem(isReal);

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

      // Elevator.setDefaultCommand(
      //     Elevator.elevatorDefaultCommand(
      //         OperatorInterface.OperatorController.getTriggerSupplier(SwerveMap.Control.DriveDeadband,
      //             SwerveMap.Control.DeadbandCurveWeight)));

      // Configure controller bindings
      // OperatorInterface.bindDriverControls(
      //     Swerve.resetGyroCommand(),
      //     Swerve.enableLockOnCommand(),
      //     Swerve.disableAutoAlignCommand(),
      //     Swerve::setAutoAlignSetpointCommand,
      //     Swerve::setDefaultCommand,
      //     Swerve::driveFieldRelativeCommand);
      // OperatorInterface.bindOperatorControls(
      //     Elevator.goToElevatorPositionCommand(ElevatorPosition.kSource),
      //     Elevator.goToElevatorPositionCommand(ElevatorPosition.kTrough),
      //     Elevator.goToElevatorPositionCommand(ElevatorPosition.kLow),
      //     Elevator.goToElevatorPositionCommand(ElevatorPosition.kMid),
      //     Elevator.goToElevatorPositionCommand(ElevatorPosition.kHigh));
      // OperatorInterface.OperatorController.povUp()
      //     .onTrue(Elevator.runSysIdDynamicRoutineCommand(Direction.kForward))
      //     .onFalse(Elevator.stopMotorsCommand());
      // OperatorInterface.OperatorController.povDown()
      //     .onTrue(Elevator.runSysIdDynamicRoutineCommand(Direction.kReverse))
      //     .onFalse(Elevator.stopMotorsCommand());
      // OperatorInterface.OperatorController.povRight()
      //     .onTrue(Elevator.runSysIdQuasistaticRoutineCommand(Direction.kForward))
      //     .onFalse(Elevator.stopMotorsCommand());
      // OperatorInterface.OperatorController.povLeft()
      //     .onTrue(Elevator.runSysIdQuasistaticRoutineCommand(Direction.kReverse))
      //     .onFalse(Elevator.stopMotorsCommand());
      OperatorInterface.OperatorController.a()
          .onTrue(Elevator.goToElevatorPositionCommand(ElevatorPosition.kHigh));
      OperatorInterface.OperatorController.y().onTrue(Elevator.goToElevatorPositionCommand(ElevatorPosition.kMid));
      OperatorInterface.OperatorController.x().onTrue(Elevator.goToElevatorPositionCommand(ElevatorPosition.kSource));
      OperatorInterface.OperatorController.b().onTrue(Elevator.goToElevatorPositionCommand(ElevatorPosition.kTrough));
      Elevator.setDefaultCommand(Elevator.runElevatorAutomaticSeekCommand());
      // Elevator.setDefaultCommand(
      //     Elevator.elevatorDefaultCommand(
      //         OperatorInterface.OperatorController.getTriggerSupplier(
      //             Controls.AXIS_DEADBAND,
      //             SwerveMap.Control.DeadbandCurveWeight)));

    } catch (Exception e) {
      DriverStation.reportError("[ERROR] >> Failed to initialize Container: " + e.getMessage(), e.getStackTrace());
    }
  }
}
