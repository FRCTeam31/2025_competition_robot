package frc.robot.subsystems.endEffector;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.BeforeEach;
import org.mockito.ArgumentCaptor;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.SuperStructure;
import frc.robot.subsystems.elevator.ElevatorMap;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

public class EndEffectorTests {
    private EndEffector endEffector;
    private IEndEffector mockHardware;

    @BeforeEach
    public void setUp() {
        mockHardware = mock(IEndEffector.class);

        endEffector = new EndEffector(false) {
            {
                this.EndEffectorIO = mockHardware; // Inject the mock
            }
        };

        // "Enables" the robot simulation
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
    }

    @Test
    public void runWristManual_respectsLimits() {
        // Arrange
        SuperStructure.EndEffector.EndEffectorAngleDegrees = EndEffectorMap.WristMinAngle - 5;

        // Act
        endEffector.runWristManual(-1.0);

        // Assert
        // Verify that the wrist speed is set to 0 when below the minimum angle
        ArgumentCaptor<Double> captor = ArgumentCaptor.forClass(Double.class);
        verify(mockHardware).setWristSpeed(captor.capture());

        double speed = captor.getValue();
        assertTrue(speed >= 0, "Wrist speed should not be negative when at min angle");
    }

    @Test
    public void seekWristAnglePID_inDangerZone_stopsMotors() {
        // Arrange current angle and elevator distance
        SuperStructure.Elevator.DistanceMeters = 0.0;
        SuperStructure.EndEffector.EndEffectorAngleDegrees = 0;

        // Attempt to seek a wrist angle while in the danger zone
        endEffector.setWristSetpoint(90);
        endEffector.seekWristAnglePID(true);

        ArgumentCaptor<Double> captor = ArgumentCaptor.forClass(Double.class);
        verify(mockHardware).setWristSpeed(captor.capture());

        // Assert that the wrist speed is set to 0 when in the danger zone
        assertEquals(0.0, captor.getValue(), "Wrist speed should be zero or near-zero in danger zone");
    }

    @Test
    public void setWristSetpointCommand_setsCorrectSetpoint() {
        // Fails due to CommandScheduler not being something that you can interact with in tests
        SuperStructure.Elevator.DistanceMeters = ElevatorMap.MaxHeight;

        CommandScheduler.getInstance().enable();
        CommandScheduler.getInstance().schedule(endEffector.setWristSetpointCommand(-40.0));
        CommandScheduler.getInstance().run();

        assertEquals(-40.0, endEffector.PIDController.getSetpoint());
    }

    @Test
    public void setWristSetpointCommand_angleGreaterThanMax_setsToMax() {
        // Fails due to CommandScheduler not being something that you can interact with in tests
        SuperStructure.Elevator.DistanceMeters = ElevatorMap.MaxHeight;

        CommandScheduler.getInstance().enable();
        CommandScheduler.getInstance().schedule(endEffector.setWristSetpointCommand(EndEffectorMap.WristMaxAngle + 10));
        CommandScheduler.getInstance().run();

        assertEquals(EndEffectorMap.WristMaxAngle, endEffector.PIDController.getSetpoint());
    }

    @Test
    public void ejectCommands_setStateProperly() {
        CommandScheduler.getInstance().enable();
        endEffector.enableEjectCommand().schedule();
        CommandScheduler.getInstance().run();

        verify(mockHardware).setIntakeSpeed(EndEffectorMap.EjectSpeed);
        assertTrue(SuperStructure.EndEffector.ManuallyEjecting);

        endEffector.disableEjectCommand().schedule();
        CommandScheduler.getInstance().run();

        verify(mockHardware).stopIntakeMotor();
        assertFalse(SuperStructure.EndEffector.ManuallyEjecting);
    }
}
