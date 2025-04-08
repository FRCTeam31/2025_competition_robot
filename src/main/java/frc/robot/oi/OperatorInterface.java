package frc.robot.oi;

import org.prime.control.Controls;
import org.prime.control.HolonomicControlStyle;
import org.prime.control.SupplierXboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.Container;
import frc.robot.game.ReefBranchSide;
import frc.robot.subsystems.swerve.SwerveMap;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.elevator.ElevatorPosition;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.vision.LimelightNameEnum;
import frc.robot.subsystems.vision.Vision;

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

        public void bindDriverControls(Swerve swerve, Climber climber, Vision vision) {
                var controlProfile = DriverController.getSwerveControlProfile(
                                OIMap.DefaultDriveControlStyle,
                                SwerveMap.Control.DriveDeadband,
                                SwerveMap.Control.DeadbandCurveWeight);

                swerve.setDefaultCommand(swerve.driveFieldRelativeCommand(controlProfile));

                DriverController.x()
                                .onTrue(swerve.disableAutoAlignCommand());
                DriverController.a()
                                .onTrue(swerve.resetGyroCommand());

                // While holding POV up, auto-align the robot to the in-view apriltag target's rotation
                DriverController.pov(Controls.up)
                                .onTrue(swerve.disableAutoAlignCommand());

                // When L or R bumper is pressed, and Y is unpressed, drive to the in-view reef target branch
                DriverController.leftBumper().and(DriverController.y().negate()).and(DriverController.b().negate())
                                .onTrue(swerve.driveToReefTargetBranch(ReefBranchSide.kLeft, controlProfile));
                DriverController.rightBumper().and(DriverController.y().negate()).and(DriverController.b().negate())
                                .onTrue(swerve.driveToReefTargetBranch(ReefBranchSide.kRight, controlProfile));

                // Climber Controls
                // Climber in will only go in until it hits the artifical stop measured by the encoder
                DriverController.y().and(DriverController.leftBumper())
                                .onTrue(climber.retractWinchAuto());
                DriverController.y().and(DriverController.rightBumper())
                                .onTrue(climber.extendWinchAuto());

                // Hooks Controls
                DriverController.b().and(DriverController.leftBumper())
                                .onTrue(climber.setHooksOpenAuto());
                DriverController.b().and(DriverController.rightBumper())
                                .onTrue(climber.setHooksClosedAuto());

                // Manual Control for setting the climber all the way in                
                DriverController.back()
                                .whileTrue(climber.retractWinchCommand())
                                .onFalse(climber.stopAllMotors());

                // Changes the vision mode for the rear limelight. 
                OperatorController.start()
                                .onTrue(vision.setLimelightPipeline(LimelightNameEnum.kRear, 1))
                                .onFalse(vision.setLimelightPipeline(LimelightNameEnum.kRear, 0));
        }

        public void bindOperatorControls(Elevator elevatorSubsystem,
                        EndEffector endEffectorSubsystem, Vision visionSubsystem,
                        Swerve swerveSubsystem) {

                elevatorSubsystem.setDefaultCommand(elevatorSubsystem
                                .elevatorDefaultCommand(OperatorController.getTriggerSupplier(0.06, 0)));

                endEffectorSubsystem.setDefaultCommand(endEffectorSubsystem.defaultCommand(
                                OperatorController.leftBumper(),
                                OperatorController.getLeftStickYSupplier(0.3, 0)));
                OperatorController.rightBumper()
                                .onTrue(endEffectorSubsystem.scoreCoral());

                // Elevator and Wrist Combined Controls                              
                OperatorController.b().onTrue(Container.setCombinedHeightAndAngle(ElevatorPosition.kAbsoluteMinimum));
                OperatorController.povDown().onTrue(Container.setCombinedHeightAndAngle(ElevatorPosition.kTrough));
                OperatorController.a().onTrue(Container.setCombinedHeightAndAngle(ElevatorPosition.kL2));
                OperatorController.x().onTrue(Container.setCombinedHeightAndAngle(ElevatorPosition.kL3));
                OperatorController.povUp().onTrue(Container.setCombinedHeightAndAngle(ElevatorPosition.kL4));
                OperatorController.start().onTrue(Container.pickupFromSourceAndLower());

                OperatorController.rightStick().onTrue(elevatorSubsystem.disableElevatorManualControlCommand());
        }

        public void setDriverRumbleIntensity(double intensity) {
                DriverController.setRumble(RumbleType.kBothRumble, intensity);
        }

}
