package frc.robot;

import java.util.HashMap;
import java.util.Map;

import org.prime.vision.LimelightInputs;

import frc.robot.subsystems.climber.ClimberInputsAutoLogged;
import frc.robot.subsystems.elevator.ElevatorInputsAutoLogged;
import frc.robot.subsystems.endEffector.EndEffectorInputsAutoLogged;
import frc.robot.subsystems.swerve.SwerveSubsystemInputsAutoLogged;
import frc.robot.subsystems.vision.LimelightNameEnum;

public class SuperStructure {
    public static ClimberInputsAutoLogged ClimberState = new ClimberInputsAutoLogged();
    public static ElevatorInputsAutoLogged ElevatorState = new ElevatorInputsAutoLogged();
    public static EndEffectorInputsAutoLogged EndEffectorState = new EndEffectorInputsAutoLogged();
    public static SwerveSubsystemInputsAutoLogged SwerveState = new SwerveSubsystemInputsAutoLogged();
    public static Map<LimelightNameEnum, LimelightInputs> LimelightStates = new HashMap<>();
}
