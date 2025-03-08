// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.prime.util.BuildConstants;

import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.PwmLEDs;
import frc.robot.subsystems.drivetrain.util.LocalADStarADK;

public class Robot extends LoggedRobot {

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static EventLoop EventLoop = new EventLoop();
  private Command _autonomousCommand;

  public Robot() {
    // Set up pathfinding compatibility with AdvantageKit
    Pathfinding.setPathfinder(new LocalADStarADK());

    // Configure logging
    configureLogging();

    // Start the web server for downloading elastic layout from robot
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    // Initialize the robot container
    Container.initialize(isReal());

    // Schedule the LED patterns to run at 120Hz
    Container.LEDs.startUpdateLoop();

    Elastic.selectTab("Auto");
  }

  /**
   * Logs program metadata, configures logging, and AdvantageKit data receivers, and configures robot logging mode.
   */
  private void configureLogging() {
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    Mode currentMode = isReal()
        ? Mode.REAL
        : isSimulation()
            ? Mode.SIM
            : Mode.REPLAY;

    switch (currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();
  }

  @Override
  public void disabledInit() {
    DataLogManager.log("Robot disabled");
    var disabledPattern = LEDPattern.solid(getAllianceColor()).breathe(Units.Seconds.of(2.0));
    Container.LEDs.setBackgroundPattern(disabledPattern);
    Container.LEDs.clearForegroundPattern();
    // Container.Swerve.disableAutoAlignCommand().schedule();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * things that you want ran during all modes.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    EventLoop.poll();

    Container.TeleopDashboardSection.setAllianceColor(onRedAlliance());
  }

  /**
   * This function is called once each time the robot enters Autonomous mode.
   */
  @Override
  public void autonomousInit() {
    Elastic.selectTab("Auto");
    var autoPattern = LEDPattern.gradient(GradientType.kDiscontinuous, getAllianceColor(), Color.kBlack)
        .offsetBy(-PwmLEDs.VMap.PixelsPerStrip / 2)
        .scrollAtRelativeSpeed(Units.Hertz.of(2))
        .reversed();
    var combinedPattern = LEDPattern.gradient(GradientType.kDiscontinuous, getAllianceColor(), Color.kBlack)
        .offsetBy(PwmLEDs.VMap.PixelsPerStrip / 2)
        .scrollAtRelativeSpeed(Units.Hertz.of(2))
        .blend(autoPattern);
    Container.LEDs.setBackgroundPattern(combinedPattern);
    Container.LEDs.clearForegroundPattern();
    // Container.Swerve.disableAutoAlignCommand().schedule();

    // Cancel any auto command that's still running and reset the subsystem states
    if (_autonomousCommand != null) {
      _autonomousCommand.cancel();

      // Stop any subsystems still running
      // Container.Swerve.stopAllMotors();
    }

    _autonomousCommand = Container.AutoBuilder.exportCombinedAutoRoutine();

    if (_autonomousCommand == null || _autonomousCommand == Commands.none()) {
      // Exit without scheduling an auto command if none is selected
      DriverStation.reportError("[ERROR] >> No auto command selected", false);
      // Container.Swerve.resetGyro();
    } else {
      // Reset the gyro if we're on the red alliance
      if (onRedAlliance()) {
        // Container.Swerve.resetGyro();
      }

      // Schedule the auto command
      _autonomousCommand.schedule();
    }
  }

  /**
   * This function is called once each time the robot enters Teleop mode.
   */
  @Override
  public void teleopInit() {
    DataLogManager.log("Teleop Enabled");

    if (DriverStation.isFMSAttached()) {
      Elastic.selectTab("Teleop");
    }

    if (_autonomousCommand != null) {
      // Cancel the auto command if it's still running
      _autonomousCommand.cancel();

      // Stop any subsystems still running
      Container.Elevator.stopMotorsCommand().schedule();
      // Container.Swerve.stopAllMotors();
    } else {
      // Container.Swerve.resetGyro();
    }

    // Set teleop LED pattern
    var telePattern = LEDPattern.solid(getAllianceColor()).scrollAtRelativeSpeed(Units.Hertz.of(2));
    Container.LEDs.setBackgroundPattern(telePattern);
    Container.LEDs.clearForegroundPattern();
    // Container.Swerve.disableAutoAlignCommand().schedule();
  }

  /**
   * This function is called once each time the robot enters Test mode.
   */
  @Override
  public void testInit() {
    Elastic.selectTab("Test");

    CommandScheduler.getInstance().cancelAll();
    // Container.Swerve.disableAutoAlignCommand().schedule();
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
