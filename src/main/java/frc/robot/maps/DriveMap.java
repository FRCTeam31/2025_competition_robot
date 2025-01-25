package frc.robot.maps;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MomentOfInertia;
import org.prime.control.PrimePIDConstants;

public class DriveMap {

        public static final double TrackWidthMeters = 0.51181;
        public static final double WheelBaseMeters = 0.67945;
        public static final double WheelBaseCircumferenceMeters = Math.PI * 0.7778174593052;
        public static final double MaxSpeedMetersPerSecond = 5.6;
        public static final double MaxAccelerationMetersPerSecondSquared = Units.feetToMeters(15);
        public static final double MaxAngularSpeedRadians = Math.PI * 3;
        public static final int PigeonId = 10;
        public static final double DriveDeadband = 0.15;
        public static final double DeadbandCurveWeight = 0.5;
        // public static final PrimePIDConstants DrivePID = new PrimePIDConstants(0.24339 / 2, 0, 0, 0,
        //                 2.022, 0.26097, 0.016668);

        public static final PrimePIDConstants DrivePID = new PrimePIDConstants(0, 0, 0, 0,
                        0.105, 0.025, 0.13);

        public static final PrimePIDConstants SteeringPID = new PrimePIDConstants(4.5, 0, 0.04);
        public static final PrimePIDConstants SnapToPID = new PrimePIDConstants(6, 0, 0);
        public static final PrimePIDConstants PathingTranslationPid = new PrimePIDConstants(3, 0, 0);
        public static final PrimePIDConstants PathingRotationPid = new PrimePIDConstants(2, 0, 0);
        public static final double DriveGearRatio = 5.9;
        public static final double DriveWheelDiameterMeters = Units.inchesToMeters(3.875);
        public static final double DriveWheelCircumferenceMeters = Math.PI * DriveWheelDiameterMeters;
        public static final int DriveSupplyCurrentLimit = 40;
        public static final int DriveSupplyCurrentLimitThreshold = 50;
        public static final int DriveSupplyCurrentLimitDuration = 100;
        public static final String LimelightRearName = "limelight-rear";
        public static final String LimelightFrontName = "limelight-front";

        public static final SwerveModuleMap FrontLeftSwerveModule = new SwerveModuleMap(1, 2, 14,
                        47 * 0.0025, false, false, new Translation2d(TrackWidthMeters / 2, WheelBaseMeters / 2));
        public static final SwerveModuleMap FrontRightSwerveModule = new SwerveModuleMap(7, 8, 13,
                        272 * 0.0025, true, false, new Translation2d(TrackWidthMeters / 2, -(WheelBaseMeters / 2)));
        public static final SwerveModuleMap RearRightSwerveModule = new SwerveModuleMap(5, 6, 12,
                        278.75 * 0.0025, true, false,
                        new Translation2d(-(TrackWidthMeters / 2), -(WheelBaseMeters / 2)));
        public static final SwerveModuleMap RearLeftSwerveModule = new SwerveModuleMap(3, 4, 11,
                        55 * 0.0025, false, false, new Translation2d(-TrackWidthMeters / 2, WheelBaseMeters / 2));

        public static final RobotConfig PathPlannerRobotConfiguration = new RobotConfig(Units.lbsToKilograms(120),
                        MomentOfInertia.ofBaseUnits(6, edu.wpi.first.units.Units.KilogramSquareMeters)
                                        .baseUnitMagnitude(),
                        new ModuleConfig(Units.inchesToMeters(4), MaxSpeedMetersPerSecond, 1.0,
                                        DCMotor.getNeoVortex(1), DriveSupplyCurrentLimit, 1),
                        FrontLeftSwerveModule.ModuleLocation, FrontRightSwerveModule.ModuleLocation,
                        RearLeftSwerveModule.ModuleLocation, RearRightSwerveModule.ModuleLocation);
}
