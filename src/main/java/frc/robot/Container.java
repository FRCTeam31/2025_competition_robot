// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.dashboard.TeleopDashboardTab;
import frc.robot.dashboard.DashboardSection;
import frc.robot.oi.OperatorInterface;
import frc.robot.oi.BuildableAutoRoutine;
import frc.robot.subsystems.*;
import frc.robot.subsystems.climbing.ClimberSubsystem;
import frc.robot.subsystems.climbing.ClimberInputs.ClimberPosition;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class Container {
  public static TeleopDashboardTab TeleopDashboardSection;
  public static DashboardSection CommandsDashboardSection;
  public static DashboardSection AutoDashboardSection;
  public static BuildableAutoRoutine AutoBuilder;

  // public static SwerveSubsystem Swerve;
  public static VisionSubsystem Vision;
  public static ClimberSubsystem Climber;
  public static PwmLEDs LEDs;
  public static OperatorInterface OperatorInterface;

  public static void initialize(boolean isReal) {
    try {
      // Create subsystems
      LEDs = new PwmLEDs();
      Vision = new VisionSubsystem();
      // Swerve = new SwerveSubsystem(isReal);
      Climber = new ClimberSubsystem(isReal);

      // Register the named commands from each subsystem that may be used in PathPlanner
      // var namedCommandsMap = Swerve.getNamedCommands();
      // ...add other named commands to the map using "otherNamedCommands.putAll(namedCommandsMap);"
      // NamedCommands.registerCommands(namedCommandsMap);

      // Create our custom auto builder
      AutoDashboardSection = new DashboardSection("Auto");
      // AutoBuilder = new BuildableAutoRoutine(namedCommandsMap);
      TeleopDashboardSection = new TeleopDashboardTab();
      CommandsDashboardSection = new DashboardSection("Commands");

      // Configure controller bindings
      OperatorInterface = new OperatorInterface();
      // OperatorInterface.bindDriverControls(
      //     Swerve.resetGyroCommand(),
      //     Swerve.enableLockOnCommand(),
      //     Swerve.disableAutoAlignCommand(),
      //     Swerve::setAutoAlignSetpointCommand,
      //     Swerve::setDefaultCommand,
      //     Swerve::driveFieldRelativeCommand);
      // OperatorInterface.bindOperatorControls(
      //     Climber.toggleHooksStateCommand(), Climber.setCLimberOutCommand(), Climber.setClimberInCommand());

      OperatorInterface.OperatorController.a().onTrue(Climber.setClimberStateCommand(ClimberPosition.OUT));
      OperatorInterface.OperatorController.b().onTrue(Climber.setClimberStateCommand(ClimberPosition.IN));

    } catch (Exception e) {
      DriverStation.reportError("[ERROR] >> Failed to initialize Container: " + e.getMessage(), e.getStackTrace());
    }
  }
}
