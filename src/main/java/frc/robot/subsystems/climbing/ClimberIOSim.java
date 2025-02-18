package frc.robot.subsystems.climbing;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import edu.wpi.first.wpilibj.simulation.PneumaticsBaseSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;

@Logged
public class ClimberIOSim implements IClimberIO {
    private DCMotorSim climberMotorSim;
    private DoubleSolenoidSim climbSolenoidSim;
    private DIOSim climbOutLimitSwitchSim;
    private DIOSim climbInLimitSwitchSim;
    private ClimberInputs m_Inputs = new ClimberInputs();

    public ClimberIOSim() {

        climberMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.001, ClimberMap.climberGearRatio),
                DCMotor.getNeoVortex(1));
        climbSolenoidSim = new DoubleSolenoidSim(
                PneumaticsBaseSim.getForType(ClimberMap.climbPneumaticsCANID, PneumaticsModuleType.CTREPCM),
                ClimberMap.climberForwardChannel,
                ClimberMap.climberReverseChannel);
        climbOutLimitSwitchSim = new DIOSim(ClimberMap.climberOutLimitSwitchChannel);
        climbInLimitSwitchSim = new DIOSim(ClimberMap.climberInLimitSwitchChannel);
    }

    public ClimberInputs updateInputs() {
        double ClimbMotorSpeed = climberMotorSim.getAngularVelocity().in(Units.RotationsPerSecond);
        Value ClimbSolenoidPosition = climbSolenoidSim.get();
        m_Inputs.ClimbMotorSpeed = ClimbMotorSpeed;
        m_Inputs.ClimbSolenoidPosition = ClimbSolenoidPosition;
        m_Inputs.OutLimitSwitch = climbOutLimitSwitchSim.getValue();
        m_Inputs.InLimitSwitch = climbInLimitSwitchSim.getValue();
        return m_Inputs;

    }

    public void setMotorSpeed(double speed) {
        climberMotorSim.setAngularVelocity(speed);
    }

    public void stopMotors() {
        climberMotorSim.setAngularVelocity(0);
    }

    public void setClimbersState(Value climberState) {
        climbSolenoidSim.set(climberState);
    }
}
