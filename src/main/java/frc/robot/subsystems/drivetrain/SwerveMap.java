package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MomentOfInertia;

import frc.robot.subsystems.drivetrain.swervemodule.SwerveModuleMap;

import org.prime.control.ExtendedPIDConstants;

public class SwerveMap {
        public class Chassis {
                // public static final double TrackWidthMeters = Units.inchesToMeters(24.75);
                // public static final double WheelBaseMeters = Units.inchesToMeters(24.75);
                public static final double TrackWidthMeters = Units.inchesToMeters(21.25);
                public static final double WheelBaseMeters = Units.inchesToMeters(21.25);
                public static final double MaxSpeedMetersPerSecond = 5.4;
                public static final double MaxAngularSpeedRadians = Math.PI * 2;
        }

        public class Control {
                public static final double DriveDeadband = 0.15;
                public static final double DeadbandCurveWeight = 0.5;
        }

        // PID Constants
        public static final ExtendedPIDConstants DrivePID = new ExtendedPIDConstants(0.0075, 0, 0.000, 0.0, 0.099, 0.12,
                        0.14);
        public static final ExtendedPIDConstants SteeringPID = new ExtendedPIDConstants(4.1, 0, 0.04);
        public static final ExtendedPIDConstants AutoAlignPID = new ExtendedPIDConstants(4, 0, 0.08);
        // public static final ExtendedPIDConstants PathPlannerTranslationPID = new ExtendedPIDConstants(3, 0, 0);
        // public static final ExtendedPIDConstants PathPlannerRotationPID = new ExtendedPIDConstants(2, 0, 0);
        public static final ExtendedPIDConstants PathPlannerTranslationPID = new ExtendedPIDConstants(4, 0, 0);
        public static final ExtendedPIDConstants PathPlannerRotationPID = new ExtendedPIDConstants(2, 0, 0);

        // Uniform Drive Constants
        public static final double DriveGearRatio = 5.9;
        public static final double DriveWheelDiameterMeters = Units.inchesToMeters(3.875);
        public static final double DriveWheelCircumferenceMeters = Math.PI * DriveWheelDiameterMeters;
        public static final int DriveStallCurrentLimit = 40;
        public static final int DriveFreeCurrentLimit = 30;
        public static final int DriveSupplyCurrentLimitDuration = 100;

        public static final int PigeonId = 10;
        public static final SwerveModuleMap FrontLeftSwerveModule = new SwerveModuleMap(
                        1,
                        2,
                        14,
                        47 * 0.0025,
                        false,
                        false,
                        new Translation2d(Chassis.TrackWidthMeters / 2, Chassis.WheelBaseMeters / 2));
        public static final SwerveModuleMap FrontRightSwerveModule = new SwerveModuleMap(
                        7,
                        8,
                        13,
                        272 * 0.0025,
                        true,
                        false,
                        new Translation2d(Chassis.TrackWidthMeters / 2, -(Chassis.WheelBaseMeters / 2)));
        public static final SwerveModuleMap RearRightSwerveModule = new SwerveModuleMap(
                        5,
                        6,
                        12,
                        278.75 * 0.0025,
                        true,
                        false,
                        new Translation2d(-(Chassis.TrackWidthMeters / 2), -(Chassis.WheelBaseMeters / 2)));
        public static final SwerveModuleMap RearLeftSwerveModule = new SwerveModuleMap(
                        3,
                        4,
                        11,
                        55 * 0.0025,
                        false,
                        false,
                        new Translation2d(-Chassis.TrackWidthMeters / 2, Chassis.WheelBaseMeters / 2));

        public static final RobotConfig PathPlannerRobotConfiguration = new RobotConfig(
                        Units.lbsToKilograms(50),
                        MomentOfInertia.ofBaseUnits(3, edu.wpi.first.units.Units.KilogramSquareMeters)
                                        .baseUnitMagnitude(),
                        new ModuleConfig(
                                        DriveWheelDiameterMeters / 2,
                                        Chassis.MaxSpeedMetersPerSecond / 10,
                                        1.0,
                                        DCMotor.getNeoVortex(1).withReduction(DriveGearRatio),
                                        DriveStallCurrentLimit,
                                        1),
                        FrontLeftSwerveModule.ModuleLocation,
                        FrontRightSwerveModule.ModuleLocation,
                        RearLeftSwerveModule.ModuleLocation,
                        RearRightSwerveModule.ModuleLocation);
}
