// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.*;
import frc.robot.maps.DriveMap;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

import prime.control.Controls;
import prime.control.HolonomicControlStyle;
import prime.control.PrimeXboxController;

@Logged(strategy = Strategy.OPT_IN)
public class Container {
  private PrimeXboxController m_driverController;

  @Logged(name = "Vision", importance = Importance.CRITICAL)
  public VisionSubsystem Vision;
  @Logged(name = "Drive", importance = Importance.CRITICAL)
  public DrivetrainSubsystem Drivetrain;
  @Logged(name = "LEDs", importance = Importance.CRITICAL)
  public PwmLEDs LEDs;

  public Container(boolean isReal) {
    try {
      DriverDashboard.init(isReal);
      m_driverController = new PrimeXboxController(Controls.DRIVER_PORT);

      // Create new subsystems
      LEDs = new PwmLEDs();
      Vision = new VisionSubsystem();
      Drivetrain = new DrivetrainSubsystem(isReal,
          LEDs::clearForegroundPattern,
          LEDs::setForegroundPattern,
          Vision::getAllLimelightInputs);

      // Register the named commands from each subsystem that may be used in
      // PathPlanner
      NamedCommands.registerCommands(Drivetrain.getNamedCommands());

      // Create Auto chooser and Auto tab in Shuffleboard
      configAutonomousDashboardItems();

      // Reconfigure bindings
      configureDriverControls();
    } catch (Exception e) {
      DriverStation.reportError("[ERROR] >> Failed to configure robot: " + e.getMessage(), e.getStackTrace());
    }
  }

  /**
   * Configures the autonomous dashboard items
   */
  public void configAutonomousDashboardItems() {
    // Build an auto chooser. This will use Commands.none() as the default option.
    DriverDashboard.addAutoChooser(AutoBuilder.buildAutoChooser("Straight Park"));

    // Add all autos to the auto tab
    var possibleAutos = AutoBuilder.getAllAutoNames();
    for (int i = 0; i < possibleAutos.size(); i++) {
      var autoCommand = new PathPlannerAuto(possibleAutos.get(i));
      DriverDashboard.AutoTab.add(possibleAutos.get(i), autoCommand).withWidget(BuiltInWidgets.kCommand).withSize(2, 1);
    }
  }

  /**
   * Returns the selected autonomous command to run
   * 
   * @return
   */
  public Command getAutonomousCommand() {
    return DriverDashboard.AutoChooser.getSelected();
  }

  /**
   * Creates the controller and configures the driver's controls
   */
  public void configureDriverControls() {
    // Controls for Driving
    m_driverController.a().onTrue(Drivetrain.resetGyroCommand());
    Drivetrain.setDefaultCommand(
        Drivetrain.driveRobotRelativeCommand(
            m_driverController.getSwerveControlProfile(
                HolonomicControlStyle.Drone,
                DriveMap.DriveDeadband,
                DriveMap.DeadbandCurveWeight)));

    // While holding b, auto-aim the robot to the apriltag target using snap-to
    m_driverController.leftStick().whileTrue(Drivetrain.enableLockOnCommand())
        .onFalse(Drivetrain.disableSnapToCommand());

    // Controls for Snap-To with field-relative setpoints
    m_driverController.x().onTrue(Drivetrain.disableSnapToCommand());
    m_driverController.pov(Controls.up).onTrue(Drivetrain.setSnapToSetpointCommand(0));
    m_driverController.pov(Controls.left).onTrue(Drivetrain.setSnapToSetpointCommand(270));
    m_driverController.pov(Controls.down).onTrue(Drivetrain.setSnapToSetpointCommand(180));
    m_driverController.pov(Controls.right).onTrue(Drivetrain.setSnapToSetpointCommand(90));

    // Uncomment to enable SysID Routines.
    // m_driverController.pov(Controls.up).onTrue(Drivetrain.runSysIdQuasistaticRoutineCommand(Direction.kForward));
    // m_driverController.pov(Controls.down).onTrue(Drivetrain.runSysIdQuasistaticRoutineCommand(Direction.kReverse));
    // m_driverController.pov(Controls.right).onTrue(Drivetrain.runSysIdDynamicRoutineCommand(Direction.kForward));
    // m_driverController.pov(Controls.left).onTrue(Drivetrain.runSysIdDynamicRoutineCommand(Direction.kReverse));
  }

}