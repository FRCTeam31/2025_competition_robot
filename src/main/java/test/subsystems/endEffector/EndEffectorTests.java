package test.subsystems.endEffector;

import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.endEffector.IEndEffector;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;

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
    }

    @Test
    public void testRunWristManualLimitsAtMinAngle() {
        endEffector._inputs.EndEffectorAngleDegrees = EndEffectorMap.WristMinAngle - 5;
        endEffector.runWristManual(-1.0);

        ArgumentCaptor<Double> captor = ArgumentCaptor.forClass(Double.class);
        verify(mockHardware).setWristSpeed(captor.capture());

        double speed = captor.getValue();
        assertTrue(speed >= 0, "Wrist speed should not be negative when at min angle");
    }

    @Test
    public void testSeekWristPIDInDangerZoneSetsZero() {
        endEffector._inputs.EndEffectorAngleDegrees = 45.0;
        endEffector._wristSetpoint = 90.0;

        endEffector.seekWristAnglePID(true);

        ArgumentCaptor<Double> captor = ArgumentCaptor.forClass(Double.class);
        verify(mockHardware).setWristSpeed(captor.capture());

        assertEquals(0.0, captor.getValue(), 0.5, "Wrist speed should be zero or near-zero in danger zone");
    }

    @Test
    public void testWristSetpointCommandChangesSetpoint() {
        CommandScheduler.getInstance().schedule(endEffector.setWristSetpointCommand(30.0));
        CommandScheduler.getInstance().run();

        assertEquals(30.0, endEffector._wristSetpoint, 0.001);
    }

    @Test
    public void testIntakeEjectToggle() {
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
