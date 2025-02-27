package frc.robot.subsystems.climbing;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import frc.robot.subsystems.climbing.ClimberInputs.ServoPosition;

@Logged
public class ClimberIOSim implements IClimberIO {
    private DCMotorSim _climberMotorSim;
    // private DoubleSolenoidSim climbSolenoidSim;
    private double _climbServoSimValue = ClimberMap.ClimberServoClosedValue;
    private DIOSim _climbOutLimitSwitchSim;
    private DIOSim _climbInLimitSwitchSim;
    private ClimberInputs _inputs = new ClimberInputs();

    public ClimberIOSim() {

        _climberMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.001, ClimberMap.ClimberGearRatio),
                DCMotor.getNeoVortex(1));

        _climbOutLimitSwitchSim = new DIOSim(ClimberMap.ClimberOutLimitSwitchChannel);
        _climbInLimitSwitchSim = new DIOSim(ClimberMap.ClimberInLimitSwitchChannel);
    }

    public ClimberInputs updateInputs() {
        double ClimbMotorSpeed = _climberMotorSim.getAngularVelocity().in(Units.RotationsPerSecond);
        _inputs.ClimbMotorSpeed = ClimbMotorSpeed;
        _inputs.OutLimitSwitch = _climbOutLimitSwitchSim.getValue();
        _inputs.InLimitSwitch = _climbInLimitSwitchSim.getValue();
        return _inputs;

    }

    public void setMotorSpeed(double speed) {
        _climberMotorSim.setAngularVelocity(speed);
    }

    public void stopMotors() {
        _climberMotorSim.setAngularVelocity(0);
    }

    public void setHooksState(ServoPosition hooksCommmandedPosition) {
        _climbServoSimValue = (hooksCommmandedPosition == ServoPosition.CLOSED ? ClimberMap.ClimberServoClosedValue
                : ClimberMap.ClimberServoOpenValue);
    }
}
