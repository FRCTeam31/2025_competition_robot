// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.dashboard.TeleopDashboardTab;
import frc.robot.game.ReefPegSide;
import frc.robot.dashboard.DashboardSection;
import frc.robot.oi.OperatorInterface;
import frc.robot.oi.routine.BuildableAutoRoutine;
import frc.robot.subsystems.climbing.ClimberSubsystem;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.subsystems.elevator.ElevatorPosition;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endEffector.EndEffectorSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class Container {
  public static TeleopDashboardTab TeleopDashboardSection;
  public static DashboardSection CommandsDashboardSection;
  public static DashboardSection AutoDashboardSection;
  public static DashboardSection TestDashboardSection;
  public static BuildableAutoRoutine AutoBuilder;

  public static SwerveSubsystem Swerve;
  public static VisionSubsystem Vision;
  public static ClimberSubsystem Climber;
  public static EndEffectorSubsystem EndEffector;
  public static ElevatorSubsystem Elevator;
  public static OperatorInterface OperatorInterface;

  public static void initialize(boolean isReal) {
    try {
      // Create subsystems
      Vision = new VisionSubsystem();
      Swerve = new SwerveSubsystem(isReal);
      Climber = new ClimberSubsystem(isReal);
      EndEffector = new EndEffectorSubsystem(isReal);

      // Create our custom auto builder
      AutoDashboardSection = new DashboardSection("Auto");
      TeleopDashboardSection = new TeleopDashboardTab();
      CommandsDashboardSection = new DashboardSection("Commands");
      TestDashboardSection = new DashboardSection("Test");

      // Create Elevator Subsystem
      Elevator = new ElevatorSubsystem(isReal);
      OperatorInterface = new OperatorInterface();

      OperatorInterface.bindDriverControls(Swerve, Climber, Vision);
      OperatorInterface.bindOperatorControls(Elevator, EndEffector, Vision);

      // Register the named commands from each subsystem that may be used in PathPlanner
      var swerveCommands = Swerve.getNamedCommands();
      var elevatorCommands = Elevator.getNamedCommands();
      var containerCommands = getNamedCommands();

      NamedCommands.registerCommands(swerveCommands);
      NamedCommands.registerCommands(elevatorCommands);
      NamedCommands.registerCommands(containerCommands);

      // Map<String, Command> combinedCommands = new HashMap<>();
      // // ...add other named commands to the map using "otherNamedCommands.putAll(namedCommandsMap);"
      // combinedCommands.putAll(swerveCommands);
      // combinedCommands.putAll(containerCommands);
      // combinedCommands.putAll(elevatorCommands);

      AutoBuilder = new BuildableAutoRoutine(containerCommands);
    } catch (Exception e) {
      DriverStation.reportError("[ERROR] >> Failed to initialize Container: " + e.getMessage(), e.getStackTrace());
    }
  }

  //#region Commands

  public static Command setCombinedHeightAndAngle(ElevatorPosition position) {
    return Commands.parallel(
        Commands.print("Setting combined height and angle: " + position),
        Elevator.setElevatorSetpointCommand(position),
        EndEffector.scheduleWristSetpointCommand(position))
        .finallyDo(() -> System.out.println("Finished setting combined setpoints"));
  }

  public static Command scoreAtHeight(ElevatorPosition position) {
    return setCombinedHeightAndAngle(position)
        // .alongWith(EndEffector.enableIntakeCommand().withTimeout(2))
        .alongWith(Commands.waitSeconds(0.25).andThen(EndEffector.scoreCoral()));
  }

  public static Command scoreAtHeightAndLower(ElevatorPosition position) {
    return scoreAtHeight(position)
        .andThen(setCombinedHeightAndAngle(ElevatorPosition.kAbsoluteMinimum));
  }

  public static Command pickupFromSource() {
    return setCombinedHeightAndAngle(ElevatorPosition.kSource)
        .andThen(EndEffector.pickupCoral());
  }

  public static Command pickupFromSourceAndLower() {
    return pickupFromSource()
        .andThen(setCombinedHeightAndAngle(ElevatorPosition.kAbsoluteMinimum));
  }

  public static Command scoreOnSideAndLower(ReefPegSide side, ElevatorPosition position) {
    // return Swerve.pathfindToReefPegSide(side)
    // .andThen(scoreAtHeightAndLower(position));

    return scoreAtHeightAndLower(position);
  }

  public static Map<String, Command> getNamedCommands() {
    return Map.of(
        "Score-L4-L", scoreOnSideAndLower(ReefPegSide.kLeft, ElevatorPosition.kHigh),
        "Score-L4-R", scoreOnSideAndLower(ReefPegSide.kRight, ElevatorPosition.kHigh),
        "Score-L3-L", scoreOnSideAndLower(ReefPegSide.kLeft, ElevatorPosition.kMid),
        "Score-L3-R", scoreOnSideAndLower(ReefPegSide.kRight, ElevatorPosition.kMid),
        "Score-L2-L", scoreOnSideAndLower(ReefPegSide.kLeft, ElevatorPosition.kLow),
        "Score-L2-R", scoreOnSideAndLower(ReefPegSide.kRight, ElevatorPosition.kLow),
        "Score-Trough", scoreAtHeightAndLower(ElevatorPosition.kTrough),
        "Pickup-Source", pickupFromSourceAndLower());
  }

  //#endregion
}
