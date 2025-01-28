// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.dashboard.DriverDashboardTab;
import frc.robot.dashboard.TestDashboardTab;
import frc.robot.oi.OperatorInterface;
import frc.robot.subsystems.*;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class Container {
  public DrivetrainSubsystem Drivetrain;
  public VisionSubsystem Vision;
  public PwmLEDs LEDs;

  public OperatorInterface OperatorInterface;
  public DriverDashboardTab DriverDashboardTab;
  public TestDashboardTab TestDashboardTab;

  public Container(boolean isReal) {
    try {
      // Create subsystems
      DriverDashboardTab = new DriverDashboardTab();
      LEDs = new PwmLEDs();
      Vision = new VisionSubsystem(DriverDashboardTab);
      Drivetrain = new DrivetrainSubsystem(isReal,
          DriverDashboardTab,
          LEDs::clearForegroundPattern,
          LEDs::setForegroundPattern,
          Vision::getAllLimelightInputs);

      // Register the named commands from each subsystem that may be used in PathPlanner
      NamedCommands.registerCommands(Drivetrain.getNamedCommands());
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
  public void configureTestDashboard() {
    var driveList = TestDashboardTab.addLayoutWithHiddenLabels("Drivetrain", 0, 0, 4, 16);
    var namedCommands = Drivetrain.getNamedCommands();
    var names = namedCommands.keySet().toArray();
    var commands = namedCommands.values().toArray();
    for (var i = 0; i < namedCommands.size(); i++) {
      driveList.add(names[i].toString(), (Command) commands[i]);
    }
  }
}
