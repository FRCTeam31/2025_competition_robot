// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.dashboard.DriverDashboardTab;
import frc.robot.dashboard.TestDashboardTab;
import frc.robot.oi.OperatorInterface;
import frc.robot.oi.PrimeAutoRoutine;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class Container {
  public static DriverDashboardTab DriverDashboardTab;
  public static TestDashboardTab TestDashboardTab;

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
      NamedCommands.registerCommands(namedCommandsMap);

      // Create our custom auto builder
      var autoBuilder = new PrimeAutoRoutine(namedCommandsMap);
      DriverDashboardTab = new DriverDashboardTab(autoBuilder);
      TestDashboardTab = new TestDashboardTab();

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

  /**
   * Gets the named commands from each subsystem and adds them to a tab on the dashboard called "Test"
   */
  public static void configureTestDashboard() {
    var driveList = TestDashboardTab.addLayoutWithHiddenLabels("Drivetrain", 0, 0, 4, 16);
    var namedCommands = Drivetrain.getNamedCommands();
    var names = namedCommands.keySet().toArray();
    var commands = namedCommands.values().toArray();
    for (var i = 0; i < namedCommands.size(); i++) {
      driveList.add(names[i].toString(), (Command) commands[i]);
    }
  }
}
