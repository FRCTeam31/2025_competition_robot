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

@Logged
public class ClimberIOSim implements IClimberIO {
    private DCMotorSim climberMotorSim;
    private DoubleSolenoidSim climbSolenoidSim;
    private ClimberInputs m_Inputs = new ClimberInputs();

    public ClimberIOSim() {

        climberMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 0.001, ClimberMap.climberGearRatio),
                DCMotor.getNeoVortex(1));
        climbSolenoidSim = new DoubleSolenoidSim(
                PneumaticsBaseSim.getForType(ClimberMap.climbPneumaticsCANID, PneumaticsModuleType.CTREPCM),
                ClimberMap.climberForwardChannel,
                ClimberMap.climberReverseChannel);
    }

    public ClimberInputs getInputs() {
        double ClimbMotorSpeed = climberMotorSim.getAngularVelocity().in(Units.RotationsPerSecond);
        Value ClimbSolenoidPosition = climbSolenoidSim.get();
        m_Inputs.ClimbMotorSpeed = ClimbMotorSpeed;
        m_Inputs.ClimbSolenoidPosition = ClimbSolenoidPosition;
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
