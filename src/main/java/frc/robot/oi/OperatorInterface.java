package frc.robot.oi;

import java.util.function.Consumer;
import java.util.function.Function;

import org.prime.control.Controls;
import org.prime.control.HolonomicControlStyle;
import org.prime.control.PrimeXboxController;
import org.prime.control.SwerveControlSuppliers;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.drivetrain.DriveMap;

public class OperatorInterface {
    public static class OIMap {
        public static final HolonomicControlStyle DefaultDriveControlStyle = HolonomicControlStyle.Drone;
    }

    public PrimeXboxController DriverController;

    public OperatorInterface() {
        DriverController = new PrimeXboxController(Controls.DRIVER_PORT);
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
                DriveMap.Control.DriveDeadband,
                DriveMap.Control.DeadbandCurveWeight);

        setDefaultDrivetrainCommandConsumer.accept(getDriveCommandFunc.apply(controlProfile));

        DriverController.a().onTrue(resetGyroCommand);

        // While holding b, auto-aim the robot to an apriltag target using snap angle
        DriverController.leftStick().whileTrue(enableLockOnCommand).onFalse(disableSnapAngleCommand);

        DriverController.x().onTrue(disableSnapAngleCommand);
        DriverController.pov(Controls.up).onTrue(setSnapToSetpointCommandFunc.apply(0));
        DriverController.pov(Controls.left).onTrue(setSnapToSetpointCommandFunc.apply(270));
        DriverController.pov(Controls.down).onTrue(setSnapToSetpointCommandFunc.apply(180));
        DriverController.pov(Controls.right).onTrue(setSnapToSetpointCommandFunc.apply(90));
    }

    public void bindOperatorControls() {
        // Not implemented yet
    }
}
