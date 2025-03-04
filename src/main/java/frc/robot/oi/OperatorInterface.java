package frc.robot.oi;

import java.util.function.Consumer;
import java.util.function.Function;

import org.prime.control.Controls;
import org.prime.control.HolonomicControlStyle;
import org.prime.control.SupplierXboxController;
import org.prime.control.SwerveControlSuppliers;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.drivetrain.SwerveMap;

public class OperatorInterface {
    public static class OIMap {
        public static final HolonomicControlStyle DefaultDriveControlStyle = HolonomicControlStyle.Drone;
    }

    public SupplierXboxController DriverController;
    public SupplierXboxController OperatorController;

    public OperatorInterface() {
        DriverController = new SupplierXboxController(Controls.DRIVER_PORT);
        OperatorController = new SupplierXboxController(Controls.OPERATOR_PORT);
    }

    /**
     * Binds the driver controls to the input commands
     * @param resetGyroCommand
     * @param enableLockOnCommand
     * @param disableSnapAngleCommand
     * @param setSnapToSetpointCommandFunc
     * @param setDefaultDrivetrainCommandConsumer
     * @param getDriveCommandFunc
     */
    public void bindDriverControls(Command resetGyroCommand,
            Command enableLockOnCommand,
            Command disableSnapAngleCommand,
            Function<Integer, Command> setSnapToSetpointCommandFunc,
            Consumer<Command> setDefaultDrivetrainCommandConsumer,
            Function<SwerveControlSuppliers, Command> getDriveCommandFunc) {

        var controlProfile = DriverController.getSwerveControlProfile(
                OIMap.DefaultDriveControlStyle,
                SwerveMap.Control.DriveDeadband,
                SwerveMap.Control.DeadbandCurveWeight);

        setDefaultDrivetrainCommandConsumer.accept(getDriveCommandFunc.apply(controlProfile));

        DriverController.a().onTrue(resetGyroCommand);

        // While holding b, auto-aim the robot to an apriltag target using snap angle
        DriverController.leftStick().whileTrue(enableLockOnCommand).onFalse(disableSnapAngleCommand);

        DriverController.x().onTrue(disableSnapAngleCommand);
        DriverController.pov(Controls.up).onTrue(setSnapToSetpointCommandFunc.apply(Controls.up));
        DriverController.pov(Controls.upRight).onTrue(setSnapToSetpointCommandFunc.apply(Controls.upRight - 90));
        DriverController.pov(Controls.right).onTrue(setSnapToSetpointCommandFunc.apply(Controls.right - 180));
        DriverController.pov(Controls.downRight).onTrue(setSnapToSetpointCommandFunc.apply(Controls.downRight + 90));
        DriverController.pov(Controls.down).onTrue(setSnapToSetpointCommandFunc.apply(Controls.down));
        DriverController.pov(Controls.downLeft).onTrue(setSnapToSetpointCommandFunc.apply(Controls.downLeft - 90));
        DriverController.pov(Controls.left).onTrue(setSnapToSetpointCommandFunc.apply(Controls.left - 180));
        DriverController.pov(Controls.upLeft).onTrue(setSnapToSetpointCommandFunc.apply(Controls.upLeft + 90));
    }

    public void bindOperatorControls(Command elevatorUpCommand, Command elevatorDownCommand, Command stopMotors) {
        OperatorController.a().onTrue(elevatorUpCommand).onFalse(stopMotors);
        OperatorController.b().onTrue(elevatorDownCommand).onFalse(stopMotors);
    }
}
