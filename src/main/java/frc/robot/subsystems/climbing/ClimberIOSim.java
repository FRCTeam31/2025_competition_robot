package frc.robot.subsystems.climbing;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import frc.robot.subsystems.climbing.ClimberSubsystem.ClimberMap;

public class ClimberIOSim implements IClimberIO {
    private DCMotorSim _climbWenchMotorsSim;

    private DCMotorSim _climbHooksMotorSim;

    // private DoubleSolenoidSim climbSolenoidSim;

    private DIOSim _climbOutLimitSwitchSim;
    private DIOSim _climbInLimitSwitchSim;
    private DIOSim _hooksOutLimitSwitchSim;
    private DIOSim _hooksInLimitSwitchSim;
    private ClimberInputs _inputs = new ClimberInputs();

    public ClimberIOSim() {

        _climbWenchMotorsSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.001, ClimberMap.ClimberGearRatio),
                DCMotor.getNeoVortex(2));

        _climbHooksMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.001, ClimberMap.ClimberGearRatio),
                DCMotor.getNeoVortex(1));

        _climbOutLimitSwitchSim = new DIOSim(ClimberMap.ClimberOutLimitSwitchChannel);
        _climbInLimitSwitchSim = new DIOSim(ClimberMap.ClimberInLimitSwitchChannel);
        _hooksOutLimitSwitchSim = new DIOSim(ClimberMap.HooksOutLimitSwitchChannel);
        _hooksInLimitSwitchSim = new DIOSim(ClimberMap.HooksInLimitSwitchChannel);
    }

    public void updateInputs(ClimberInputsAutoLogged inputs) {
        double climbWenchMotorSpeed = _climbWenchMotorsSim.getAngularVelocity().magnitude();
        double climbHookMotorSpeed = _climbHooksMotorSim.getAngularVelocity().magnitude();

        _inputs.ClimbWenchMotorSpeed = climbWenchMotorSpeed;
        _inputs.HooksMotorSpeed = climbHookMotorSpeed;
        _inputs.ClimbWenchOutLimitSwitch = _climbOutLimitSwitchSim.getValue();
        _inputs.ClimbWenchInLimitSwitch = _climbInLimitSwitchSim.getValue();
        _inputs.HooksClosedLimitSwitch = _hooksOutLimitSwitchSim.getValue();
        _inputs.HooksOpenLimitSwitch = _hooksInLimitSwitchSim.getValue();
    }

    public void setClimbingWenchSpeed(double speed) {
        _climbWenchMotorsSim.setAngularVelocity(speed);
    }

    public void setHookMotorSpeed(double speed) {
        _climbHooksMotorSim.setAngularVelocity(speed);
    }

    public void stopWenchMotors() {
        _climbWenchMotorsSim.setAngularVelocity(0);
    }

    public void stopHooksMotors() {
        _climbHooksMotorSim.setAngularVelocity(0);
    }

}
