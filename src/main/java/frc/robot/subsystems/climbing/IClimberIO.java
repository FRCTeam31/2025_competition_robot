package frc.robot.subsystems.climbing;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public interface IClimberIO {

    public ClimberInputs updateInputs();

    public void setMotorSpeed(double Speed);

    public void stopMotors();

    public void setClimbersState(Value climberState);

}
