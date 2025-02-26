package frc.robot.subsystems.climbing;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class ClimberSubsystem extends SubsystemBase {
    private IClimberIO _climber;
    private ClimberInputs _inputs;

    public enum ClimberPosition {
        /** The starting position of the climber (Up) */
        IN,
        /** 𝓣𝓱𝓮 𝓮𝓷𝓭𝓲𝓷𝓰 𝓹𝓸𝓼𝓲𝓽𝓲𝓸𝓷 𝓸𝓯 𝓽𝓱𝓮 𝓬𝓵𝓲𝓶𝓫𝓮𝓻 (Down) */
        OUT
    }

    public enum ServoPosition {
        /** The starting position of the climber (Up) */
        IN,
        /** 𝓣𝓱𝓮 𝓮𝓷𝓭𝓲𝓷𝓰 𝓹𝓸𝓼𝓲𝓽𝓲𝓸𝓷 𝓸𝓯 𝓽𝓱𝓮 𝓬𝓵𝓲𝓶𝓫𝓮𝓻 (Down) */
        OUT
    }

    private ClimberPosition _climberPosition = ClimberPosition.IN;
    private ServoPosition _servoPosition = ServoPosition.IN;

    public ClimberSubsystem(Boolean isReal) {
        if (isReal) {
            _climber = new ClimberIOReal();
        } else {
            _climber = new ClimberIOSim();
        }

        _inputs = _climber.updateInputs();
    }

    @Override
    public void periodic() {
        _inputs = _climber.updateInputs();
    }

    public Command stopClimbingMotorsCommand() {
        return Commands.runOnce(() -> {
            _climber.stopMotors();
        }, this);
    }

    public Command setHooksStateCommand(ServoPosition servoPosition) {
        return Commands.runOnce(() -> {
            _climber.setHooksState(servoPosition);
        }, this);

    }

    public Command toggleHooksStateCommand(ServoPosition servoPosition) {
        return Commands.runOnce(() -> {
            _climber.setHooksState(_servoPosition == ServoPosition.IN ? ServoPosition.OUT : ServoPosition.IN);
        }, this);

    }

    public void setClimberSpeedCommand(double speed) {

        if (_inputs.InLimitSwitch && speed < 0) {
            MathUtil.clamp(speed, 0, ClimberMap.maxMotorPercentOutput);
        } else if (_inputs.OutLimitSwitch && speed > 0) {
            MathUtil.clamp(speed, -ClimberMap.maxMotorPercentOutput, 0);
        }
        _climber.setMotorSpeed(speed);

    }

    public Command toggleClimbersCommand() {
        return Commands.runOnce(() -> {
            if (_inputs.OutLimitSwitch || _climberPosition == ClimberPosition.OUT) {
                setClimberSpeedCommand(ClimberMap.climberInSpeed);
                _climberPosition = ClimberPosition.IN;
            } else if (_inputs.InLimitSwitch || _climberPosition == ClimberPosition.IN) {
                setClimberSpeedCommand(ClimberMap.climberOutSpeed);
                _climberPosition = ClimberPosition.OUT;
            }
        }, this).withTimeout(5).andThen(stopClimbingMotorsCommand());
    }

}