package frc.robot.oi;

import java.time.LocalTime;
import java.util.function.Consumer;
import java.util.function.Function;

import org.prime.control.Controls;
import org.prime.control.HolonomicControlStyle;
import org.prime.control.SupplierXboxController;
import org.prime.control.SwerveControlSuppliers;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Container;
import frc.robot.subsystems.drivetrain.SwerveMap;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorMap;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.endEffector.EndEffectorSubsystem;
import frc.robot.subsystems.endEffector.EndEffectorSubsystem.EndEffectorMap;

public class OperatorInterface {
        public static class OIMap {
                public static final HolonomicControlStyle DefaultDriveControlStyle = HolonomicControlStyle.Drone;
        }

        public SupplierXboxController DriverController;
        public SupplierXboxController OperatorController;

        private long _time;

        public OperatorInterface() {
                DriverController = new SupplierXboxController(Controls.DRIVER_PORT);
                OperatorController = new SupplierXboxController(Controls.OPERATOR_PORT);
        }

        /**
         * Binds the driver controls to the input commands
         * @param resetGyroCommand
         * @param enableLockOnCommand
         * @param disableSnapAngleCommand
         * @param setSnapToSetpointCommandFunc
         * @param setDefaultDrivetrainCommandConsumer
         * @param getDriveCommandFunc
         */
        public void bindDriverControls(Command resetGyroCommand,
                        Command enableLockOnCommand,
                        Command disableSnapAngleCommand,
                        Function<Integer, Command> setSnapToSetpointCommandFunc,
                        Consumer<Command> setDefaultDrivetrainCommandConsumer,
                        Function<SwerveControlSuppliers, Command> getDriveCommandFunc) {

                var controlProfile = DriverController.getSwerveControlProfile(
                                OIMap.DefaultDriveControlStyle,
                                SwerveMap.Control.DriveDeadband,
                                SwerveMap.Control.DeadbandCurveWeight);

                setDefaultDrivetrainCommandConsumer.accept(getDriveCommandFunc.apply(controlProfile));

                DriverController.a().onTrue(resetGyroCommand);

                // While holding b, auto-aim the robot to an apriltag target using snap angle
                DriverController.leftStick().whileTrue(enableLockOnCommand).onFalse(disableSnapAngleCommand);

                DriverController.x().onTrue(disableSnapAngleCommand);
                // DriverController.pov(Controls.up).onTrue(setSnapToSetpointCommandFunc.apply(Controls.up));
                // DriverController.pov(Controls.upRight).onTrue(setSnapToSetpointCommandFunc.apply(Controls.upRight - 90));
                // DriverController.pov(Controls.right).onTrue(setSnapToSetpointCommandFunc.apply(Controls.right - 180));
                // DriverController.pov(Controls.downRight).onTrue(setSnapToSetpointCommandFunc.apply(Controls.downRight + 90));
                // DriverController.pov(Controls.down).onTrue(setSnapToSetpointCommandFunc.apply(Controls.down));
                // DriverController.pov(Controls.downLeft).onTrue(setSnapToSetpointCommandFunc.apply(Controls.downLeft - 90));
                // DriverController.pov(Controls.left).onTrue(setSnapToSetpointCommandFunc.apply(Controls.left - 180));
                // DriverController.pov(Controls.upLeft).onTrue(setSnapToSetpointCommandFunc.apply(Controls.upLeft + 90));

                // DriverController.povUp().onTrue(Commands.runOnce(() -> Container.Elevator.addVoltage(0.05)));
                // DriverController.povDown().onTrue(Commands.runOnce(() -> Container.Elevator.addVoltage(-0.01)));
                // DriverController.b().whileTrue(Commands.run(() -> Container.Elevator.setMotorVoltages(0.8)))
                //         .onFalse(Commands.runOnce(() -> Container.Elevator.setMotorVoltages(0)));

                //         DriverController.b().onTrue(
                //                         Commands.runOnce(() -> _time = System.nanoTime())
                //                                         .andThen(Commands.run(() -> Container.Elevator.setMotorVoltages(-1.5))
                //                                                         .until(() -> Container.Elevator
                //                                                                         .getElevatorPositionMeters() <= 0.3))
                //                                         .andThen(() -> {
                //                                                 Container.Elevator.setMotorVoltages(0);

                //                                                 long deltaNano = System.nanoTime() - _time;
                //                                                 long deltaSeconds = deltaNano / 1000000000;

                //                                                 double kV = (1.5 * deltaSeconds * deltaSeconds) / (0.627 - 0.3);

                //                                                 System.out.println("New kV Value: " + kV);
                //                                         }));
        }

        // public void bindOperatorControls(
        //         Function<ElevatorPosition, Command> setElevatorEndEffectorCombinedSetpointFunc,
        //         Function<Double, Command> runIntakeFunc,
        //         Command stopIntakeCommand,
        //         Command resetEndEffectorManualControlCommand,
        //         Command resetElevatorManualControlCommand,
        //         Consumer<Command> setDefaultElevatorCommandConsumer,
        //         Consumer<Command> setDefaultEndEffectorCommandConsumer,
        //         Function<DoubleSupplier, Command> elevatorDefaultCommand,
        //         TriFunction<BooleanSupplier, BooleanSupplier, DoubleSupplier, Command> endEffectorDefaultCommand) {

        //     setDefaultElevatorCommandConsumer
        //             .accept(elevatorDefaultCommand.apply(OperatorController.getTriggerSupplier(
        //                     0.06, 0)));

        //     setDefaultEndEffectorCommandConsumer
        //             .accept(endEffectorDefaultCommand.apply(OperatorController.rightBumper(),
        //                     OperatorController.leftBumper(),
        //                     OperatorController.getLeftStickYSupplier(0.06, 0)));

        //     OperatorController.povDown().onTrue(setElevatorEndEffectorCombinedSetpointFunc.apply(ElevatorPosition.kTrough));
        //     OperatorController.a().onTrue(setElevatorEndEffectorCombinedSetpointFunc.apply(ElevatorPosition.kLow));
        //     OperatorController.x().onTrue(setElevatorEndEffectorCombinedSetpointFunc.apply(ElevatorPosition.kMid));
        //     OperatorController.povUp().onTrue(setElevatorEndEffectorCombinedSetpointFunc.apply(ElevatorPosition.kHigh));
        //     OperatorController.start().onTrue(setElevatorEndEffectorCombinedSetpointFunc.apply(ElevatorPosition.kSource));
        //     OperatorController.b()
        //             .onTrue(setElevatorEndEffectorCombinedSetpointFunc.apply(ElevatorPosition.kAbsoluteMinimum));

        //     // OperatorController.rightBumper().whileTrue(runIntakeFunc.apply(EndEffectorMap.EjectSpeed))
        //     //         .onFalse(stopIntakeCommand);
        //     // OperatorController.leftBumper().whileTrue(runIntakeFunc.apply(EndEffectorMap.IntakeSpeed))
        //     //         .onFalse(stopIntakeCommand);

        //     OperatorController.leftStick().onTrue(resetEndEffectorManualControlCommand);
        // }

        public void bindOperatorControls(ElevatorSubsystem elevatorSubsystem,
                        EndEffectorSubsystem endEffectorSubsystem) {

                elevatorSubsystem.setDefaultCommand(elevatorSubsystem
                                .ElevatorDefaultCommand(OperatorController.getTriggerSupplier(0.06, 0)));

                endEffectorSubsystem.setDefaultCommand(
                                endEffectorSubsystem.defaultCommand(
                                                OperatorController.rightBumper(),
                                                OperatorController.leftBumper(),
                                                OperatorController.getLeftStickYSupplier(0.3, 0)));

                OperatorController.povDown().onTrue(Container.setCombinedHeightAndAngle(ElevatorPosition.kTrough));
                OperatorController.a().onTrue(Container.setCombinedHeightAndAngle(ElevatorPosition.kLow));
                OperatorController.x().onTrue(Container.setCombinedHeightAndAngle(ElevatorPosition.kMid));
                OperatorController.povUp().onTrue(Container.setCombinedHeightAndAngle(ElevatorPosition.kHigh));
                OperatorController.start().onTrue(Container.setCombinedHeightAndAngle(ElevatorPosition.kSource));
                OperatorController.b().onTrue(Container.setCombinedHeightAndAngle(ElevatorPosition.kAbsoluteMinimum));

                OperatorController.leftStick().onTrue(endEffectorSubsystem.resetWristManualControlCommand());
                OperatorController.rightStick().onTrue(elevatorSubsystem.resetElevatorManualControlCommand());
        }

        public void setDriverRumbleIntensity(double intensity) {
                DriverController.setRumble(RumbleType.kBothRumble, intensity);
        }

}
