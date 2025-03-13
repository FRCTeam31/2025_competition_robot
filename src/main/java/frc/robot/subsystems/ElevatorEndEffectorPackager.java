package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Container;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.endEffector.EndEffectorSubsystem;

public class ElevatorEndEffectorPackager {
    private ElevatorSubsystem _elevatorSubsystem = Container.Elevator;
    private EndEffectorSubsystem _endEffectorSubsystem = Container.EndEffector;

    public Command setWristSetpointCommand(ElevatorPosition position) {
        return Commands.parallel(
                _elevatorSubsystem.goToElevatorPositionCommand(position),
                _endEffectorSubsystem.setWristSetpointCommand(position));
    }
}
