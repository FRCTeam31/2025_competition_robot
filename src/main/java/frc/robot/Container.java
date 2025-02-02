// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.dashboard.DriverDashboardTab;
import frc.robot.dashboard.GenericDashboardSection;
import frc.robot.dashboard.TestDashboardSection;
import frc.robot.oi.OperatorInterface;
import frc.robot.oi.PrimeAutoRoutine;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class Container {
  public static DriverDashboardTab DriverDashboardSection;
  public static TestDashboardSection TestDashboardSection;
  public static GenericDashboardSection CommandsDashboardSection;

  public static DrivetrainSubsystem Drivetrain;
  public static VisionSubsystem Vision;
  public static PwmLEDs LEDs;
  public static OperatorInterface OperatorInterface;

  public static void initialize(boolean isReal) {
    try {
      // Create subsystems
      LEDs = new PwmLEDs();
      Vision = new VisionSubsystem();
      Drivetrain = new DrivetrainSubsystem(isReal);

      // Register the named commands from each subsystem that may be used in PathPlanner
      var namedCommandsMap = Drivetrain.getNamedCommands();
      // ...add other named commands to the map using "otherNamedCommands.putAll(namedCommandsMap);"
      NamedCommands.registerCommands(namedCommandsMap);

      // Create our custom auto builder
      var autoBuilder = new PrimeAutoRoutine(namedCommandsMap);
      DriverDashboardSection = new DriverDashboardTab(autoBuilder);
      TestDashboardSection = new TestDashboardSection();
      CommandsDashboardSection = new GenericDashboardSection("Commands");

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
