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
import frc.robot.subsystems.climbing.ClimberSubsystem;
import frc.robot.subsystems.climbing.ClimberInputs.HooksPosition;
import frc.robot.subsystems.drivetrain.SwerveMap;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
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

        public OperatorInterface() {
                DriverController = new SupplierXboxController(Controls.DRIVER_PORT);
                OperatorController = new SupplierXboxController(Controls.OPERATOR_PORT);
        }

        // /**
        //  * Binds the driver controls to the input commands
        //  * @param resetGyroCommand
        //  * @param enableLockOnCommand
        //  * @param disableSnapAngleCommand
        //  * @param setSnapToSetpointCommandFunc
        //  * @param setDefaultDrivetrainCommandConsumer
        //  * @param getDriveCommandFunc
        //  */
        // public void bindDriverControls(Command resetGyroCommand,
        //                 Command enableLockOnCommand,
        //                 Command disableSnapAngleCommand,
        //                 Function<Integer, Command> setSnapToSetpointCommandFunc,
        //                 Consumer<Command> setDefaultDrivetrainCommandConsumer,
        //                 Function<SwerveControlSuppliers, Command> getDriveCommandFunc) {

        //         var controlProfile = DriverController.getSwerveControlProfile(
        //                         OIMap.DefaultDriveControlStyle,
        //                         SwerveMap.Control.DriveDeadband,
        //                         SwerveMap.Control.DeadbandCurveWeight);

        //         setDefaultDrivetrainCommandConsumer.accept(getDriveCommandFunc.apply(controlProfile));

        //         DriverController.a().onTrue(resetGyroCommand);

        //         // While holding leftStick, auto-aim the robot to an apriltag target using snap angle
        //         DriverController.leftStick().whileTrue(enableLockOnCommand).onFalse(disableSnapAngleCommand);

        //         DriverController.x().onTrue(disableSnapAngleCommand);
        //         // DriverController.pov(Controls.up).onTrue(setSnapToSetpointCommandFunc.apply(Controls.up));
        //         // DriverController.pov(Controls.upRight).onTrue(setSnapToSetpointCommandFunc.apply(Controls.upRight - 90));
        //         // DriverController.pov(Controls.right).onTrue(setSnapToSetpointCommandFunc.apply(Controls.right - 180));
        //         // DriverController.pov(Controls.downRight).onTrue(setSnapToSetpointCommandFunc.apply(Controls.downRight + 90));
        //         // DriverController.pov(Controls.down).onTrue(setSnapToSetpointCommandFunc.apply(Controls.down));
        //         // DriverController.pov(Controls.downLeft).onTrue(setSnapToSetpointCommandFunc.apply(Controls.downLeft - 90));
        //         // DriverController.pov(Controls.left).onTrue(setSnapToSetpointCommandFunc.apply(Controls.left - 180));
        //         // DriverController.pov(Controls.upLeft).onTrue(setSnapToSetpointCommandFunc.apply(Controls.upLeft + 90));

        // }

        public void bindDriverControls(SwerveSubsystem swerveSubsystem, ClimberSubsystem climber) {
                var controlProfile = DriverController.getSwerveControlProfile(
                                OIMap.DefaultDriveControlStyle,
                                SwerveMap.Control.DriveDeadband,
                                SwerveMap.Control.DeadbandCurveWeight);

                swerveSubsystem.setDefaultCommand(swerveSubsystem.driveFieldRelativeCommand(controlProfile));

                // While holding leftStick, auto-aim the robot to an apriltag target using snap angle
                DriverController.leftStick().whileTrue(swerveSubsystem.enableLockOnCommand())
                                .onFalse(swerveSubsystem.disableAutoAlignCommand());

                DriverController.x().onTrue(swerveSubsystem.disableAutoAlignCommand());
                //Map Autoalign to Pov
                DriverController.pov(Controls.up).onTrue(swerveSubsystem.setAutoAlignSetpointCommand(Controls.up));
                DriverController.pov(Controls.upRight)
                                .onTrue(swerveSubsystem.setAutoAlignSetpointCommand(Controls.upRight - 90));
                DriverController.pov(Controls.right)
                                .onTrue(swerveSubsystem.setAutoAlignSetpointCommand(Controls.right - 180));
                DriverController.pov(Controls.downRight)
                                .onTrue(swerveSubsystem.setAutoAlignSetpointCommand(Controls.downRight + 90));
                DriverController.pov(Controls.down).onTrue(swerveSubsystem.setAutoAlignSetpointCommand(Controls.down));
                DriverController.pov(Controls.downLeft)
                                .onTrue(swerveSubsystem.setAutoAlignSetpointCommand(Controls.downLeft - 90));
                DriverController.pov(Controls.left)
                                .onTrue(swerveSubsystem.setAutoAlignSetpointCommand(Controls.left - 180));
                DriverController.pov(Controls.upLeft)
                                .onTrue(swerveSubsystem.setAutoAlignSetpointCommand(Controls.upLeft + 90));

                //Climber Controls
                DriverController.y().and(DriverController.leftBumper()).onTrue(climber.setClimberInCommand());
                DriverController.y().and(DriverController.rightBumper()).onTrue(climber.setClimberOutCommand());
                DriverController.b().and(DriverController.leftBumper()
                                .onTrue(climber.setHooksStateCommand(HooksPosition.CLOSED)));
                DriverController.b().and(DriverController.rightBumper()
                                .onTrue(climber.setHooksStateCommand(HooksPosition.OPEN)));
                DriverController.a().and(DriverController.leftBumper()).whileTrue(climber.climbDownCommand())
                                .onFalse(climber.stopClimbingMotorsCommand());
                DriverController.a().and(DriverController.rightBumper()).whileTrue(climber.climbUpCommand())
                                .onFalse(climber.stopClimbingMotorsCommand());

        }

        public void bindOperatorControls(ElevatorSubsystem elevatorSubsystem,
                        EndEffectorSubsystem endEffectorSubsystem) {

                elevatorSubsystem.setDefaultCommand(elevatorSubsystem
                                .ElevatorDefaultCommand(OperatorController.getTriggerSupplier(0.06, 0)));

                endEffectorSubsystem.setDefaultCommand(
                                endEffectorSubsystem.defaultCommand(
                                                OperatorController.rightBumper(),
                                                OperatorController.leftBumper(),
                                                OperatorController.getLeftStickYSupplier(0.3, 0)));

                // Elevator and Wrist Combined Controls                              
                OperatorController.povDown().onTrue(Container.setCombinedHeightAndAngle(ElevatorPosition.kTrough));
                OperatorController.a().onTrue(Container.setCombinedHeightAndAngle(ElevatorPosition.kLow));
                OperatorController.x().onTrue(Container.setCombinedHeightAndAngle(ElevatorPosition.kMid));
                OperatorController.povUp().onTrue(Container.setCombinedHeightAndAngle(ElevatorPosition.kHigh));
                OperatorController.start().onTrue(Container.setCombinedHeightAndAngle(ElevatorPosition.kSource));
                OperatorController.b().onTrue(Container.setCombinedHeightAndAngle(ElevatorPosition.kAbsoluteMinimum));

                OperatorController.rightBumper()
                                .whileTrue(endEffectorSubsystem.setIntakeSpeedCommand(EndEffectorMap.EjectSpeed))
                                .onFalse(endEffectorSubsystem.stopIntakeMotorCommand());
                OperatorController.leftBumper()
                                .whileTrue(endEffectorSubsystem.setIntakeSpeedCommand(EndEffectorMap.IntakeSpeed))
                                .onFalse(endEffectorSubsystem.stopIntakeMotorCommand());

                OperatorController.leftStick().onTrue(endEffectorSubsystem.disabletWristManualControlCommand());
                OperatorController.rightStick().onTrue(elevatorSubsystem.disableElevatorManualControlCommand());

        }

        public void setDriverRumbleIntensity(double intensity) {
                DriverController.setRumble(RumbleType.kBothRumble, intensity);
        }

}
