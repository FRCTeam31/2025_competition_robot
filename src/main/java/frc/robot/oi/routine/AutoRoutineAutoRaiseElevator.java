package frc.robot.oi.routine;

import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Container;
import frc.robot.subsystems.elevator.ElevatorPosition;

public class AutoRoutineAutoRaiseElevator {
    private Pattern _filterPattern = Pattern.compile("^Score|^Pickup");
    private Pattern _removeJunkPattern = Pattern.compile("Score-(L[2-4])-[LR]|Score-(Trough)|Pickup-(Source)");
    public List<ElevatorPosition> AutoElevatorPositionsList = new ArrayList<>();

    public AutoRoutineAutoRaiseElevator() {
        // Automatically raise the elevator based on the next elevator height.
    }

    /**
     * Construct the {@code AutoElevatorPositionsList} from a given routine.
     */
    public void constructAutoElevatorPositionsList(List<String> routine) {
        List<String> _stringList = new ArrayList<>();
        _stringList.addAll(routine);

        _stringList.removeIf(_filterPattern.asPredicate().negate());

        List<String> _processedList = _stringList.stream()
                .map(item -> {
                    Matcher _matcher = _removeJunkPattern.matcher(item);
                    if (_matcher.matches()) {
                        return _matcher.group(1) != null ? _matcher.group(1)
                                : _matcher.group(2) != null ? _matcher.group(2) : _matcher.group(3);
                    }
                    return null;
                })
                .filter(s -> s != null)
                .collect(Collectors.toList());

        for (var item : _processedList) {
            AutoElevatorPositionsList.add(ElevatorPosition.getFromRawName(item));
        }
    }

    /**
     * A command that raises the elevator based on the current auto routine.
     * @implNote Must be called each time you want to update the height of the
     * elevator.
     * @return
     */
    public Command automaticallyRaiseElevatorBasedOnAutoRoutine() {
        return Container.setCombinedHeightAndAngle(AutoElevatorPositionsList.get(0))
                .andThen(() -> AutoElevatorPositionsList.remove(0));
    }

}