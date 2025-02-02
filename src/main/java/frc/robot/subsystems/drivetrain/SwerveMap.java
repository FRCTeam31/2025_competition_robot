package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MomentOfInertia;

import frc.robot.subsystems.drivetrain.swervemodule.SwerveModuleMap;

import org.prime.control.PrimePIDConstants;

public class SwerveMap {
        public static class Chassis {
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
        public static final PrimePIDConstants DrivePID = new PrimePIDConstants(0.01, 0, 0.000, 0.0, 0.096, 0, 0.14);
        public static final PrimePIDConstants SteeringPID = new PrimePIDConstants(4, 0, 0.04);
        public static final PrimePIDConstants AutoAlignPID = new PrimePIDConstants(3.75, 0, 0);
        public static final PrimePIDConstants PathPlannerTranslationPID = new PrimePIDConstants(3, 0, 0);
        public static final PrimePIDConstants PathPlannerRotationPID = new PrimePIDConstants(2, 0, 0);

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
                        0.673828,
                        true,
                        true,
                        new Translation2d(Chassis.TrackWidthMeters / 2, Chassis.WheelBaseMeters / 2));
        public static final SwerveModuleMap FrontRightSwerveModule = new SwerveModuleMap(
                        7,
                        8,
                        13,
                        0.113770,
                        true,
                        true,
                        new Translation2d(Chassis.TrackWidthMeters / 2, -(Chassis.WheelBaseMeters / 2)));
        public static final SwerveModuleMap RearRightSwerveModule = new SwerveModuleMap(
                        5,
                        6,
                        12,
                        0.699951,
                        true,
                        true,
                        new Translation2d(-(Chassis.TrackWidthMeters / 2), -(Chassis.WheelBaseMeters / 2)));
        public static final SwerveModuleMap RearLeftSwerveModule = new SwerveModuleMap(
                        3,
                        4,
                        11,
                        0.136963,
                        true,
                        true,
                        new Translation2d(-Chassis.TrackWidthMeters / 2, Chassis.WheelBaseMeters / 2));

        public static final RobotConfig PathPlannerRobotConfiguration = new RobotConfig(
                        Units.lbsToKilograms(120),
                        MomentOfInertia.ofBaseUnits(6, edu.wpi.first.units.Units.KilogramSquareMeters)
                                        .baseUnitMagnitude(),
                        new ModuleConfig(
                                        Units.inchesToMeters(4),
                                        Chassis.MaxSpeedMetersPerSecond,
                                        1.0,
                                        DCMotor.getNeoVortex(1),
                                        DriveStallCurrentLimit,
                                        1),
                        FrontLeftSwerveModule.ModuleLocation,
                        FrontRightSwerveModule.ModuleLocation,
                        RearLeftSwerveModule.ModuleLocation,
                        RearRightSwerveModule.ModuleLocation);
}
