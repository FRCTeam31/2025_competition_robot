// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.dashboard.TeleopDashboardTab;
import frc.robot.game.ReefBranchSide;
import frc.robot.dashboard.DashboardSection;
import frc.robot.oi.OperatorInterface;
import frc.robot.oi.routine.BuildableAutoRoutine;
import frc.robot.subsystems.PwmLEDs;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.elevator.ElevatorPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.vision.Vision;

public class Container {
  public static TeleopDashboardTab TeleopDashboardSection;
  public static DashboardSection CommandsDashboardSection;
  public static DashboardSection AutoDashboardSection;
  public static DashboardSection TestDashboardSection;
  public static BuildableAutoRoutine AutoBuilder;

  public static PwmLEDs LEDs;
  public static Swerve Swerve;
  public static Vision Vision;
  public static Climber Climber;
  public static EndEffector EndEffector;
  public static Elevator Elevator;
  public static OperatorInterface OperatorInterface;

  public static void initialize(boolean isReal) {
    try {
      // Create dashboard sections
      AutoDashboardSection = new DashboardSection("Auto");
      TeleopDashboardSection = new TeleopDashboardTab();
      CommandsDashboardSection = new DashboardSection("Commands");
      TestDashboardSection = new DashboardSection("Test");

      // Create subsystems
      LEDs = new PwmLEDs();
      Vision = new Vision();
      Swerve = new Swerve(isReal);
      Elevator = new Elevator(isReal);
      Climber = new Climber(isReal);
      EndEffector = new EndEffector(isReal);

      // Create and bind the operator interface
      OperatorInterface = new OperatorInterface();
      OperatorInterface.bindDriverControls(Swerve, Climber, Vision);
      OperatorInterface.bindOperatorControls(Elevator, EndEffector, Vision, Swerve);

      // Register the named commands from each subsystem that may be used in PathPlanner
      NamedCommands.registerCommands(Swerve.getNamedCommands());
      NamedCommands.registerCommands(Elevator.getNamedCommands());
      NamedCommands.registerCommands(getNamedCommands());

      // Create our custom auto builder
      AutoBuilder = new BuildableAutoRoutine(getNamedCommandSuppliers());
    } catch (Exception e) {
      DriverStation.reportError("[ERROR] >> Failed to initialize Container: " + e.getMessage(), e.getStackTrace());
    }
  }

  //#region Commands

  // public static Command setCombinedHeightAndAngle(ElevatorPosition position) {
  //   return Commands.parallel(
  //       Commands.print("Setting combined height and angle: " + position),
  //       LEDs.setAllSectionPatternsCommand(LEDPattern.solid(Color.kYellow)
  //           .mask(LEDPattern.progressMaskLayer(Elevator::getElevatorPositionPercent))),
  //       Elevator.setElevatorSetpointCommand(position),
  //       EndEffector.setWristSetpointCommand(position))
  //       .alongWith(Commands.waitUntil(() -> EndEffector.wristAtSetpoint() && Elevator.atSetpoint()).withTimeout(3))
  //       .andThen(LEDs.setAllSectionPatternsCommand(LEDPattern.solid(Color.kGreen)))
  //       .finallyDo(() -> {
  //         System.out.println("Finished setting combined setpoints");
  //         LEDs.setAllSectionPatterns(LEDPattern.solid(Color.kGreen));
  //       });
  // }

  public static Command setCombinedHeightAndAngle(ElevatorPosition position) {
    return Commands.parallel(
        Commands.print("Setting combined height and angle: " + position),
        Elevator.setElevatorSetpointCommand(position),
        EndEffector.setWristSetpointCommand(position))
        .alongWith(Commands.waitUntil(() -> EndEffector.wristAtSetpoint() && Elevator.atSetpoint()).withTimeout(3))
        .finallyDo(() -> {
          System.out.println("Finished setting combined setpoints");
        });
  }

  public static Command scoreAtHeight(ElevatorPosition position) {
    return setCombinedHeightAndAngle(position)
        .andThen(EndEffector.scoreCoral());
  }

  public static Command scoreAtHeightAndLower(ElevatorPosition position) {
    return scoreAtHeight(position)
        .andThen(Commands.print(">> COMMAND: Returning to AbsoluteMinimum"))
        .andThen(setCombinedHeightAndAngle(ElevatorPosition.kAbsoluteMinimum));
  }

  public static Command pickupFromSource() {
    return setCombinedHeightAndAngle(ElevatorPosition.kSource)
        .andThen(Commands.print(">> COMMAND: Picking up coral"))
        .andThen(EndEffector.pickupCoral());
  }

  public static Command pickupFromSourceAndLower() {
    return pickupFromSource()
        .andThen(Commands.print(">> COMMAND: Returning to AbsoluteMinimum"))
        .andThen(setCombinedHeightAndAngle(ElevatorPosition.kAbsoluteMinimum));
  }

  public static Command scoreOnSideAndLower(ReefBranchSide side, ElevatorPosition position) {
    return scoreAtHeightAndLower(position);
  }

  public static Map<String, Supplier<Command>> getNamedCommandSuppliers() {
    return Map.of(
        "Score-L4", () -> scoreOnSideAndLower(ReefBranchSide.kRight, ElevatorPosition.kL4),
        "Score-L3", () -> scoreOnSideAndLower(ReefBranchSide.kRight, ElevatorPosition.kL3),
        "Score-L2", () -> scoreOnSideAndLower(ReefBranchSide.kRight, ElevatorPosition.kL2),
        "Score-Trough", () -> scoreAtHeightAndLower(ElevatorPosition.kTrough),
        "Pickup-Source", () -> pickupFromSourceAndLower());
  }

  public static Map<String, Command> getNamedCommands() {
    return Map.of(
        "Score-L4", scoreOnSideAndLower(ReefBranchSide.kRight, ElevatorPosition.kL4),
        "Score-L3", scoreOnSideAndLower(ReefBranchSide.kRight, ElevatorPosition.kL3),
        "Score-L2", scoreOnSideAndLower(ReefBranchSide.kRight, ElevatorPosition.kL2),
        "Score-Trough", scoreAtHeightAndLower(ElevatorPosition.kTrough),
        "Pickup-Source", pickupFromSourceAndLower());
  }

  //#endregion
}
