// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.oi.OperatorInterface;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class Container {
  public OperatorInterface OperatorInterface;
  public DrivetrainSubsystem Drivetrain;
  public VisionSubsystem Vision;
  public PwmLEDs LEDs;

  public Container(boolean isReal) {
    try {
      // Create subsystems
      LEDs = new PwmLEDs();
      Vision = new VisionSubsystem();
      Drivetrain = new DrivetrainSubsystem(isReal,
          LEDs::clearForegroundPattern,
          LEDs::setForegroundPattern,
          Vision::getAllLimelightInputs);

      // Register the named commands from each subsystem that may be used in PathPlanner
      NamedCommands.registerCommands(Drivetrain.getNamedCommands());

      // Create Auto chooser and Auto tab in Shuffleboard
      DriverDashboard.initialize(isReal);
      DriverDashboard.setupAutonomousDashboardItems();

      // Configure controller bindings
      OperatorInterface = new OperatorInterface();
      OperatorInterface.bindDriverControls(
          Drivetrain.resetGyroCommand(),
          Drivetrain.enableLockOnCommand(),
          Drivetrain.disableAutoAlignCommand(),
          Drivetrain::setAutoAlignSetpointCommand,
          Drivetrain::setDefaultCommand,
          Drivetrain::driveRobotRelativeCommand);
    } catch (Exception e) {
      DriverStation.reportError("[ERROR] >> Failed to initialize Container: " + e.getMessage(), e.getStackTrace());
    }
  }
}
