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
                this._endEffector = mockHardware; // Inject the mock
            }
        };

        // "Enables" the robot simulation
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
    }

    @Test
    public void testRunWristManualLimitsAtMinAngle() {
        SuperStructure.EndEffectorState.EndEffectorAngleDegrees = EndEffectorMap.WristMinAngle - 5;
        endEffector.runWristManual(-1.0);

        ArgumentCaptor<Double> captor = ArgumentCaptor.forClass(Double.class);
        verify(mockHardware).setWristSpeed(captor.capture());

        double speed = captor.getValue();
        assertTrue(speed >= 0, "Wrist speed should not be negative when at min angle");
    }

    @Test
    public void testSeekWristPIDInDangerZoneSetsZero() {
        SuperStructure.ElevatorState.DistanceMeters = 0.0;
        SuperStructure.EndEffectorState.EndEffectorAngleDegrees = 45.0;
        endEffector._wristSetpoint = 90.0;

        endEffector.seekWristAnglePID(true);

        ArgumentCaptor<Double> captor = ArgumentCaptor.forClass(Double.class);
        verify(mockHardware).setWristSpeed(captor.capture());

        assertEquals(0.0, captor.getValue(), 0.5, "Wrist speed should be zero or near-zero in danger zone");
    }

    @Test
    public void testWristSetpointCommandChangesSetpoint() {
        // Fails due to CommandScheduler not being something that you can interact with in tests
        SuperStructure.ElevatorState.DistanceMeters = ElevatorMap.MaxElevatorHeight;

        CommandScheduler.getInstance().enable();
        CommandScheduler.getInstance().schedule(endEffector.setWristSetpointCommand(30.0));
        CommandScheduler.getInstance().run();

        assertEquals(30.0, endEffector._wristSetpoint);
    }

    @Test
    public void testIntakeEjectToggle() {
        CommandScheduler.getInstance().enable();
        endEffector.enableEjectCommand().schedule();
        CommandScheduler.getInstance().run();

        verify(mockHardware).setIntakeSpeed(EndEffectorMap.EjectSpeed);
        assertTrue(endEffector._intakeIsEjecting);

        endEffector.disableEjectCommand().schedule();
        CommandScheduler.getInstance().run();

        verify(mockHardware).stopIntakeMotor();
        assertFalse(endEffector._intakeIsEjecting);
    }
}
