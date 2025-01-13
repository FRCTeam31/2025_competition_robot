package frc.robot.maps;

import edu.wpi.first.math.geometry.Translation2d;

public class SwerveModuleMap {

  public int DriveMotorCanId;
  public int SteeringMotorCanId;
  public int CANCoderCanId;
  public double CanCoderStartingOffset;
  public Translation2d ModuleLocation;
  public boolean DriveInverted;
  public boolean SteerInverted;
  public double DriveMotorRampRate = 0.5;

  public SwerveModuleMap(
    int driveMotorCanId,
    int steeringMotorCanId,
    int canCoderCanId,
    double canCoderStartingOffset,
    boolean driveInverted,
    boolean steerInverted,
    Translation2d location
  ) {
    DriveMotorCanId = driveMotorCanId;
    SteeringMotorCanId = steeringMotorCanId;

    CANCoderCanId = canCoderCanId;
    CanCoderStartingOffset = canCoderStartingOffset;

    DriveInverted = driveInverted;
    SteerInverted = steerInverted;

    ModuleLocation = location;
  }
}
