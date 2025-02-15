// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.dashboard.DriverDashboardTab;
import frc.robot.dashboard.DashboardSection;
import frc.robot.dashboard.TestDashboardSection;
import frc.robot.oi.OperatorInterface;
import frc.robot.oi.PrimeAutoRoutine;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.*;
import frc.robot.maps.EndEffectorMap;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.subsystems.endEffector.EndEffectorSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class Container {
  public static DriverDashboardTab DriverDashboardSection;
  public static TestDashboardSection TestDashboardSection;
  public static DashboardSection CommandsDashboardSection;

  public static SwerveSubsystem Swerve;
  public static VisionSubsystem Vision;
  public static PwmLEDs LEDs;
  public static OperatorInterface OperatorInterface;

  public static void initialize(boolean isReal) {
    try {
      // Create subsystems
      LEDs = new PwmLEDs();
      Vision = new VisionSubsystem();
      Swerve = new SwerveSubsystem(isReal);
      EndEffector = new EndEffectorSubsystem(isReal);

      // Register the named commands from each subsystem that may be used in PathPlanner
      var namedCommandsMap = Swerve.getNamedCommands();
      // ...add other named commands to the map using "otherNamedCommands.putAll(namedCommandsMap);"
      NamedCommands.registerCommands(namedCommandsMap);

      // Create our custom auto builder
      var autoBuilder = new PrimeAutoRoutine(namedCommandsMap);
      DriverDashboardSection = new DriverDashboardTab(autoBuilder);
      TestDashboardSection = new TestDashboardSection();
      CommandsDashboardSection = new DashboardSection("Commands");

      // Configure controller bindings
      OperatorInterface = new OperatorInterface();
      OperatorInterface.bindDriverControls(
          Swerve.resetGyroCommand(),
          Swerve.enableLockOnCommand(),
          Swerve.disableAutoAlignCommand(),
          Swerve::setAutoAlignSetpointCommand,
          Swerve::setDefaultCommand,
          Swerve::driveRobotRelativeCommand);
    } catch (Exception e) {
      DriverStation.reportError("[ERROR] >> Failed to initialize Container: " + e.getMessage(), e.getStackTrace());
    }
  }
}
