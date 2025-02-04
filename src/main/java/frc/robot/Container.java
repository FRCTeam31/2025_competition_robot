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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.*;
import frc.robot.subsystems.climbing.ClimberSubsystem;
import frc.robot.maps.DriveMap;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

import org.prime.control.Controls;
import org.prime.control.HolonomicControlStyle;
import org.prime.control.PrimeXboxController;

@Logged(strategy = Strategy.OPT_IN)
public class Container {
  private PrimeXboxController m_driverController;
  //private double _voltage = 0;

  @Logged(name = "Vision", importance = Importance.CRITICAL)
  public VisionSubsystem Vision;
  @Logged(name = "Drive", importance = Importance.CRITICAL)
  public DrivetrainSubsystem Drivetrain;
  @Logged(name = "LEDs", importance = Importance.CRITICAL)
  public PwmLEDs LEDs;
  @Logged(name = "Climber", importance = Importance.CRITICAL)
  private ClimberSubsystem Climber;

  public Container(boolean isReal) {
    try {
      System.out.println("Robot diameter: " + DriveMap.WheelBaseCircumferenceMeters);
      DriverDashboard.init(isReal);
      m_driverController = new PrimeXboxController(Controls.DRIVER_PORT);

      // Create new subsystems
      LEDs = new PwmLEDs();
      Vision = new VisionSubsystem();
      Drivetrain = new DrivetrainSubsystem(isReal, LEDs::clearForegroundPattern,
          LEDs::setForegroundPattern, Vision::getAllLimelightInputs);
      Climber = new ClimberSubsystem(isReal);
      // Register the named commands from each subsystem that may be used in
      // PathPlanner
      NamedCommands.registerCommands(Drivetrain.getNamedCommands());

      // Create Auto chooser and Auto tab in Shuffleboard
      configAutonomousDashboardItems();

      // Reconfigure bindings
      configureDriverControls();
    } catch (Exception e) {
      DriverStation.reportError("[ERROR] >> Failed to configure robot: " + e.getMessage(),
          e.getStackTrace());
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
      DriverDashboard.AutoTab.add(possibleAutos.get(i), autoCommand)
          .withWidget(BuiltInWidgets.kCommand).withSize(2, 1);
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
        Drivetrain.driveRobotRelativeCommand(m_driverController.getSwerveControlProfile(
            HolonomicControlStyle.Drone, DriveMap.DriveDeadband, DriveMap.DeadbandCurveWeight)));

    // While holding b, auto-aim the robot to the apriltag target using snap-to
    m_driverController.leftStick().whileTrue(Drivetrain.enableLockOnCommand())
        .onFalse(Drivetrain.disableSnapToCommand());
    m_driverController.y().onTrue(Climber.toggleClimbers());
    // Controls for Snap-To with field-relative setpoints
    m_driverController.x().onTrue(Drivetrain.disableSnapToCommand());
    m_driverController.pov(Controls.up)
        .onTrue(Drivetrain.setSnapToSetpointCommand(Controls.up));
    m_driverController.pov(Controls.upRight)
        .onTrue(Drivetrain.setSnapToSetpointCommand(Controls.upRight - 90));
    m_driverController.pov(Controls.right)
        .onTrue(Drivetrain.setSnapToSetpointCommand(Controls.right - 180));
    m_driverController.pov(Controls.downRight)
        .onTrue(Drivetrain.setSnapToSetpointCommand(Controls.downRight + 90));
    m_driverController.pov(Controls.down)
        .onTrue(Drivetrain.setSnapToSetpointCommand(Controls.down));
    m_driverController.pov(Controls.downLeft)
        .onTrue(Drivetrain.setSnapToSetpointCommand(Controls.downLeft - 90));
    m_driverController.pov(Controls.left)
        .onTrue(Drivetrain.setSnapToSetpointCommand(Controls.left - 180));
    m_driverController.pov(Controls.upLeft)
        .onTrue(Drivetrain.setSnapToSetpointCommand(Controls.upLeft + 90));

    // Uncomment to enable SysID Routines.
    // m_driverController.pov(Controls.up)
    //     .onTrue(Drivetrain.runSysIdQuasistaticRoutineCommand(Direction.kForward))
    //     .onFalse(Drivetrain.stopAllMotors());
    // m_driverController.pov(Controls.down).onTrue(Drivetrain.runSysIdQuasistaticRoutineCommand(Direction.kReverse))
    //     .onFalse(Drivetrain.stopAllMotors());

    // m_driverController.pov(Controls.right).onTrue(Drivetrain.runSysIdDynamicRoutineCommand(Direction.kForward))
    //     .onFalse(Drivetrain.stopAllMotors());

    // m_driverController.pov(Controls.left).onTrue(Drivetrain.runSysIdDynamicRoutineCommand(Direction.kReverse))
    //     .onFalse(Drivetrain.stopAllMotors());

    // Uncomment below and voltage variable at the top of file to enable voltage step drive for kS testing
    // m_driverController.pov(Controls.up)
    //     .onTrue(Commands.runOnce(() -> {
    //       _voltage += 0.05;
    //       System.out.println("Set to voltage: " + _voltage);
    //     }));

    // m_driverController.pov(Controls.down)
    //     .onTrue(Commands.runOnce(() -> {
    //       _voltage -= 0.01;
    //       System.out.println("Set to voltage: " + _voltage);
    //     }));

    // m_driverController.pov(Controls.right)
    //     .onTrue(Commands.run(() -> {
    //       Drivetrain.driveSwerveVoltage(_voltage);
    //     }, Drivetrain).alongWith(Commands.runOnce(()->{
    //       System.out.println("Voltage Step Enabled!");
    //     })));

    // m_driverController.pov(Controls.left)
    //     .onTrue(Commands.runOnce(() -> {
    //       _voltage = 0;
    //       Drivetrain.driveSwerveVoltage(0);
    //       System.out.println("Set to voltage: " + _voltage);
    //     }, Drivetrain).alongWith(Commands.runOnce(()->{
    //       System.out.println("Voltage Step Disabled!");
    //     })));

  }

}
