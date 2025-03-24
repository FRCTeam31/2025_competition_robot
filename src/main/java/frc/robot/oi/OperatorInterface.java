package frc.robot.oi;

import org.prime.control.Controls;
import org.prime.control.HolonomicControlStyle;
import org.prime.control.SupplierXboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.Container;
import frc.robot.subsystems.climbing.ClimberSubsystem;
import frc.robot.subsystems.swerve.SwerveMap;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.elevator.ElevatorPosition;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endEffector.EndEffectorSubsystem;
import frc.robot.subsystems.vision.LimelightNameEnum;
import frc.robot.subsystems.vision.VisionSubsystem;

public class OperatorInterface {
        public static class OIMap {
                public static final HolonomicControlStyle DefaultDriveControlStyle = HolonomicControlStyle.Drone;
        }

        private SupplierXboxController DriverController;
        private SupplierXboxController OperatorController;

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

                DriverController.x()
                                .onTrue(swerveSubsystem.disableAutoAlignCommand());
                DriverController.a()
                                .onTrue(swerveSubsystem.resetGyroCommand());

                // While holding POV up, auto-align the robot to the in-view apriltag target's rotation
                DriverController.pov(Controls.up)
                                .whileTrue(swerveSubsystem.enableReefAutoAlignCommand())
                                .onFalse(swerveSubsystem.disableAutoAlignCommand());

                // Climber Controls
                // Climber in will only go in until it hits the artifical stop measured by the encoder
                DriverController.y().and(DriverController.leftBumper())
                                .whileTrue(climber.setClimberInCommand())
                                .onFalse(climber.stopAllMotors());
                DriverController.y().and(DriverController.rightBumper())
                                .whileTrue(climber.setClimberOutCommand())
                                .onFalse(climber.stopClimbingMotorsCommand());

                // Hooks Controls
                DriverController.b().and(DriverController.leftBumper())
                                .whileTrue(climber.setHooksOpenCommand())
                                .onFalse(climber.stopHooksMotorsCommand());
                DriverController.b().and(DriverController.rightBumper())
                                .whileTrue(climber.setHooksClosedCommand())
                                .onFalse(climber.stopHooksMotorsCommand());

                // Manual Control for setting the climber all the way in                
                DriverController.back()
                                .whileTrue(climber.fullyClimbInManual())
                                .onFalse(climber.stopClimbingMotorsCommand());

                // Changes the vision mode for the rear limelight. 
                OperatorController.start()
                                .onTrue(visionSubsystem.setLimelightPipeline(LimelightNameEnum.kRear, 1))
                                .onFalse(visionSubsystem.setLimelightPipeline(LimelightNameEnum.kRear, 0));
        }

        public void bindOperatorControls(ElevatorSubsystem elevatorSubsystem,
                        EndEffectorSubsystem endEffectorSubsystem, VisionSubsystem visionSubsystem,
                        SwerveSubsystem swerveSubsystem) {

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
