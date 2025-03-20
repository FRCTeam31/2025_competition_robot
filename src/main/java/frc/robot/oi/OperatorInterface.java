package frc.robot.oi;

import java.util.Map;

import org.prime.control.Controls;
import org.prime.control.HolonomicControlStyle;
import org.prime.control.SupplierXboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Container;
import frc.robot.game.ReefPegSide;
import frc.robot.subsystems.climbing.ClimberSubsystem;
import frc.robot.subsystems.drivetrain.SwerveMap;
import frc.robot.subsystems.drivetrain.SwerveSubsystem;
import frc.robot.subsystems.elevator.ElevatorPosition;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endEffector.EndEffectorSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class OperatorInterface {
        public static class OIMap {
                public static final HolonomicControlStyle DefaultDriveControlStyle = HolonomicControlStyle.Standard;
        }

        private SupplierXboxController DriverController;
        private SupplierXboxController OperatorController;
        private Map<String, Command> _containerNamedCommands = Container.getNamedCommands();

        public OperatorInterface() {
                DriverController = new SupplierXboxController(Controls.DRIVER_PORT);
                OperatorController = new SupplierXboxController(Controls.OPERATOR_PORT);
        }

        public void bindDriverControls(SwerveSubsystem swerveSubsystem, ClimberSubsystem climber,
                        VisionSubsystem visionSubsystem) {
                var controlProfile = DriverController.getSwerveControlProfile(
                                OIMap.DefaultDriveControlStyle,
                                SwerveMap.Control.DriveDeadband,
                                SwerveMap.Control.DeadbandCurveWeight);

                swerveSubsystem.setDefaultCommand(swerveSubsystem.driveFieldRelativeCommand(controlProfile));

                // While holding leftStick, auto-aim the robot to an apriltag target using snap angle
                DriverController.leftStick().whileTrue(swerveSubsystem.enableLockOnCommand())
                                .onFalse(swerveSubsystem.disableAutoAlignCommand());

                DriverController.x().onTrue(swerveSubsystem.disableAutoAlignCommand());
                DriverController.a().onTrue(swerveSubsystem.resetGyroCommand());
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

                // Climber Controls
                // Climber in will only go in until it hits the artifical stop measured by the encoder
                DriverController.y().and(DriverController.leftBumper()).whileTrue(climber.setClimberInCommand())
                                .onFalse(climber.stopAllMotors());
                DriverController.y().and(DriverController.rightBumper()).whileTrue(climber.setClimberOutCommand())
                                .onFalse(climber.stopClimbingMotorsCommand());

                // Hooks Controls
                DriverController.b().and(DriverController.leftBumper())
                                .whileTrue(climber.setHooksOpenCommand()).onFalse(climber.stopHooksMotorsCommand());
                DriverController.b().and(DriverController.rightBumper())
                                .whileTrue(climber.setHooksClosedCommand()).onFalse(climber.stopHooksMotorsCommand());

                // Manual Control for setting the climber all the way in                
                DriverController.back().whileTrue(climber.fullyClimbInManual())
                                .onFalse(climber.stopClimbingMotorsCommand());

                // Changes the vision mode for the rear limelight. 
                OperatorController.start().onTrue(visionSubsystem.setRearCameraMode(true))
                                .onFalse(visionSubsystem.setRearCameraMode(false));
        }

        public void bindOperatorControls(ElevatorSubsystem elevatorSubsystem,
                        EndEffectorSubsystem endEffectorSubsystem, VisionSubsystem visionSubsystem,
                        SwerveSubsystem swerveSubsystem) {

                elevatorSubsystem.setDefaultCommand(elevatorSubsystem
                                .elevatorDefaultCommand(OperatorController.getTriggerSupplier(0.06, 0)));

                endEffectorSubsystem.setDefaultCommand(
                                endEffectorSubsystem.defaultCommand(
                                                OperatorController.leftBumper(),
                                                OperatorController.getLeftStickYSupplier(0.3, 0)));

                // Elevator and Wrist Combined Controls                              
                OperatorController.povDown().onTrue(Container.setCombinedHeightAndAngle(ElevatorPosition.kTrough));
                OperatorController.a().onTrue(Container.setCombinedHeightAndAngle(ElevatorPosition.kLow));
                OperatorController.x().onTrue(Container.setCombinedHeightAndAngle(ElevatorPosition.kMid));
                OperatorController.povUp().onTrue(Container.setCombinedHeightAndAngle(ElevatorPosition.kHigh));
                OperatorController.start().onTrue(Container.setCombinedHeightAndAngle(ElevatorPosition.kSource));
                OperatorController.b().onTrue(Container.setCombinedHeightAndAngle(ElevatorPosition.kAbsoluteMinimum));

                // OperatorController.povDown().onTrue(_containerNamedCommands.get("Score-Trough"));
                // OperatorController.a().onTrue(_containerNamedCommands.get("Score-L2-L"));
                // OperatorController.x().onTrue(_containerNamedCommands.get("Score-L3-L"));
                // OperatorController.povUp().onTrue(_containerNamedCommands.get("Score-L4-L"));
                // OperatorController.start().onTrue(_containerNamedCommands.get("Pickup-Source"));

                OperatorController.leftStick().onTrue(endEffectorSubsystem.disableWristManualControlCommand());
                OperatorController.rightStick().onTrue(elevatorSubsystem.disableElevatorManualControlCommand());

                // OperatorController.back().whileTrue(Commands.runOnce(() -> {
                //         Container.Vision.setLedMode(0, 2);
                //         Container.Vision.setLedMode(1, 2);
                // })).onFalse(Commands.runOnce(() -> {
                //         Container.Vision.setLedMode(0, 0);
                //         Container.Vision.setLedMode(1, 0);
                // }));

                OperatorController.back().onTrue(visionSubsystem.setRearCameraPipeline(1).ignoringDisable(true))
                                .onFalse(visionSubsystem.setRearCameraPipeline(0).ignoringDisable(true));
                OperatorController.povLeft().onTrue(swerveSubsystem.pathfindToReefPegSide(ReefPegSide.kLeft));
                OperatorController.povRight().onTrue(swerveSubsystem.pathfindToReefPegSide(ReefPegSide.kRight));
        }

        public void setDriverRumbleIntensity(double intensity) {
                DriverController.setRumble(RumbleType.kBothRumble, intensity);
        }

}
