package frc.robot.subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;

public class ClimberSim implements IClimber {
    private DCMotorSim _climbWinchMotorsSim;
    private DCMotorSim _climbHooksMotorSim;

    private DIOSim _climbOutLimitSwitchSim;
    private DIOSim _climbInLimitSwitchSim;
    private DIOSim _hooksOutLimitSwitchSim;
    private DIOSim _hooksInLimitSwitchSim;

    public ClimberSim() {
        _climbWinchMotorsSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.001, ClimberMap.ClimberGearRatio),
                DCMotor.getNeoVortex(2));

        _climbHooksMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.001, ClimberMap.ClimberGearRatio),
                DCMotor.getNeoVortex(1));

        _climbOutLimitSwitchSim = new DIOSim(ClimberMap.ClimberOutLimitSwitchChannel);
        _climbInLimitSwitchSim = new DIOSim(ClimberMap.ClimberInLimitSwitchChannel);
        _hooksOutLimitSwitchSim = new DIOSim(ClimberMap.HooksOpenLimitSwitchChannel);
        _hooksInLimitSwitchSim = new DIOSim(ClimberMap.HooksClosedLimitSwitchChannel);
    }

    @Override
    public void updateInputs(ClimberInputsAutoLogged inputs) {
        inputs.WinchMotorSpeed = _climbWinchMotorsSim.getAngularVelocity().magnitude();
        inputs.HooksMotorSpeed = _climbHooksMotorSim.getAngularVelocity().magnitude();
        inputs.WinchOuterLimitSwitch = _climbOutLimitSwitchSim.getValue();
        inputs.WinchInnerLimitSwitch = _climbInLimitSwitchSim.getValue();
        inputs.HooksClosedLimitSwitch = _hooksOutLimitSwitchSim.getValue();
        inputs.HooksOpenLimitSwitch = _hooksInLimitSwitchSim.getValue();
    }

    @Override
    public void setWinchSpeed(double speed) {
        _climbWinchMotorsSim.setAngularVelocity(speed);
    }

    @Override
    public void setHookMotorSpeed(double speed) {
        _climbHooksMotorSim.setAngularVelocity(speed);
    }

    @Override
    public void stopWinchMotors() {
        _climbWinchMotorsSim.setAngularVelocity(0);
    }

    @Override
    public void stopHooksMotors() {
        _climbHooksMotorSim.setAngularVelocity(0);
    }

    @Override
    public void resetClimberAngle() {
        // Not implemented
    }

}
