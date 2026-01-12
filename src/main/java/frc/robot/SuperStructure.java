package frc.robot;

import java.util.HashMap;
import java.util.Map;

import frc.robot.subsystems.climber.ClimberInputsAutoLogged;
import frc.robot.subsystems.elevator.ElevatorInputsAutoLogged;
import frc.robot.subsystems.endEffector.EndEffectorInputsAutoLogged;
import frc.robot.subsystems.swerve.SwerveSubsystemInputsAutoLogged;
import frc.robot.subsystems.vision.LimelightInputsAutoLogged;
import frc.robot.subsystems.vision.LimelightNameEnum;

public class SuperStructure {
    public static ClimberInputsAutoLogged Climber = new ClimberInputsAutoLogged();
    public static ElevatorInputsAutoLogged Elevator = new ElevatorInputsAutoLogged();
    public static EndEffectorInputsAutoLogged EndEffector = new EndEffectorInputsAutoLogged();
    public static SwerveSubsystemInputsAutoLogged Swerve = new SwerveSubsystemInputsAutoLogged();
    public static Map<LimelightNameEnum, LimelightInputsAutoLogged> Limelights = new HashMap<>();
}
