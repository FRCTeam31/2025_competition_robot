package org.prime.dashboard;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import frc.robot.Robot;

public class SendableButton implements Sendable {
    private String _name;
    private boolean _isPressed = false;
    private BooleanEvent _pressedEvent;

    public SendableButton(String name, Runnable action) {
        _name = name;
        _pressedEvent = new BooleanEvent(Robot.EventLoop, () -> _isPressed)
                .debounce(0.1);

        _pressedEvent.ifHigh(() -> {
            action.run();
            _isPressed = false;
        });
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Toggle Button");
        builder.addBooleanProperty(_name, () -> _isPressed, value -> _isPressed = value);
    }
}
