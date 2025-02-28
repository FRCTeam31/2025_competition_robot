package frc.robot.subsystems.climbing;

import edu.wpi.first.units.measure.Time;

public class ClimberMap {
    public static final byte ClimberLeftMotorCANID = 100;
    public static final byte ClimberRightMotorCANID = 69;
    public static final byte ClimberGearRatio = 60;
    public static final double ClimberInSpeed = 0.7;
    public static final double ClimberOutSpeed = -0.5;
    public static final int ClimberOutLimitSwitchChannel = 0;
    public static final int ClimberInLimitSwitchChannel = 0;
    public static final double MaxMotorPercentOutput = 1;
    public static final double ClimberMotorsReelingInSpeed = 0.5;
    public static final double ClimberMotorsReelingOutSpeed = -0.5;
    public static final double MaxChangeClimberStateTime = 5;
    public static final int ClimberHookMotorCANID = 0;
    public static final int HooksOutLimitSwitchChannel = 0;
    public static final int HooksInLimitSwitchChannel = 0;
    public static final double HooksOpenSpeed = 0.5;
    public static final double HooksCloseSpeed = -0.5;
    public static final double MaxChangeHookStateTime = 5;

}
