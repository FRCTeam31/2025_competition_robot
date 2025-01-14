// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.PwmLEDs;

@Logged(strategy = Strategy.OPT_IN)
public class Robot extends TimedRobot {

  @Logged(name = "Container", importance = Importance.CRITICAL)
  private Container m_robotContainer;
  private Command m_autonomousCommand;

  public Robot() {
    super(0.02); // Run the robot loop at 50Hz

    // Configure logging
    DataLogManager.start();
    DataLogManager.logConsoleOutput(true);
    DataLogManager.logNetworkTables(true);
    DriverStation.startDataLog(DataLogManager.getLog());
    Epilogue.configure(config -> {
      if (isSimulation()) {
        // If running in simulation, then we'd want to re-throw any errors that
        // occur so we can debug and fix them!
        config.errorHandler = ErrorHandler.crashOnError();
      }

      // Change the root data path
      config.root = "Telemetry";

      // Configure minimum logging level
      // config.minimumImportance = Logged.Importance.CRITICAL;
    });
    Epilogue.bind(this);

    // Initialize the robot container
    m_robotContainer = new Container(isReal());

    // Schedule the LED patterns to run at 200Hz
    // This is the recommended way to schedule a periodic task that runs faster than the robot loop
    // This also adds a delay of 3ms to the task to ensure it does not run at the same time as 
    // subsystem periodic functions
    addPeriodic(m_robotContainer.LEDs::updateLedStrip, 
      Units.Milliseconds.of(5), 
      Units.Milliseconds.of(3));
  }

  @Override
  public void disabledInit() {
    DataLogManager.log("Robot disabled");
    var disabledPattern = LEDPattern.solid(getAllianceColor()).breathe(Units.Seconds.of(2.0));
    m_robotContainer.LEDs.setBackgroundPattern(disabledPattern);
    m_robotContainer.LEDs.clearForegroundPattern();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * things that you want ran during all modes.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    DriverDashboard.AllianceBox.setBoolean(onRedAlliance());
  }

  /**
   * This function is called once each time the robot enters Autonomous mode.
   */
  @Override
  public void autonomousInit() {
    var autoPattern = LEDPattern.gradient(GradientType.kDiscontinuous, getAllianceColor(), Color.kBlack)
      .offsetBy(-PwmLEDs.VMap.PixelsPerStrip / 2)
      .scrollAtRelativeSpeed(Units.Hertz.of(2))
      .reversed();
    var combinedPattern = LEDPattern.gradient(GradientType.kDiscontinuous, getAllianceColor(), Color.kBlack)
      .offsetBy(PwmLEDs.VMap.PixelsPerStrip / 2)
      .scrollAtRelativeSpeed(Units.Hertz.of(2))
      .blend(autoPattern);
    m_robotContainer.LEDs.setBackgroundPattern(combinedPattern);
    m_robotContainer.LEDs.clearForegroundPattern();

    // Cancel any auto command that's still running and reset the subsystem states
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();

      // Stop the shooter and intake motors in case they're still running and set the intake IN
      // m_robotContainer.Shooter.stopMotorsCommand().schedule();
      // m_robotContainer.Intake.stopRollersCommand().schedule();
      // m_robotContainer.Intake.setIntakeInCommand().schedule();
    }

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // Exit without scheduling an auto command if none is selected
    if (m_autonomousCommand == null || m_autonomousCommand == Commands.none()) {
      DriverStation.reportError("[ERROR] >> No auto command selected", false);
      m_robotContainer.Drivetrain.resetGyro();
    } else {
      // Schedule the auto command
      m_robotContainer.Drivetrain.EstimatePoseUsingFrontCamera = true;
      m_robotContainer.Drivetrain.EstimatePoseUsingRearCamera = true;

      if (onRedAlliance()) m_robotContainer.Drivetrain.resetGyro();

      SmartDashboard.putString("Robot/Auto/CommandName", m_autonomousCommand.getName());
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called once each time the robot enters Teleop mode.
   */
  @Override
  public void teleopInit() {
    DataLogManager.log("Teleop Enabled");
    if (m_autonomousCommand != null) {
      // Cancel the auto command if it's still running
      m_autonomousCommand.cancel();

      // Stop the shooter and intake motors in case they're still running
      // m_robotContainer.Shooter.stopMotorsCommand().schedule();
      // m_robotContainer.Intake.stopRollersCommand().schedule();
    }

    // Set teleop LED pattern
    var telePattern = LEDPattern.solid(getAllianceColor()).scrollAtRelativeSpeed(Units.Hertz.of(2));
    m_robotContainer.LEDs.setBackgroundPattern(telePattern);
    m_robotContainer.LEDs.clearForegroundPattern();

    m_robotContainer.Drivetrain.EstimatePoseUsingFrontCamera = false;
    m_robotContainer.Drivetrain.EstimatePoseUsingRearCamera = false;
  }

  /**
   * This function is called once each time the robot enters Test mode.
   */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  public static boolean onRedAlliance() {
    var alliance = DriverStation.getAlliance();

    return alliance.isPresent() && alliance.get() == Alliance.Red;
  }

  public static boolean onBlueAlliance() {
    var alliance = DriverStation.getAlliance();

    return alliance.isPresent() && alliance.get() == Alliance.Blue;
  }

  public static Color getAllianceColor() {
    var alliance = DriverStation.getAlliance();
    Color allianceColor = Color.kGhostWhite;
    if (alliance.isPresent())
      allianceColor = alliance.get() == Alliance.Red 
        ? Color.kRed 
        : Color.kBlue;

    return allianceColor;
  }
}
