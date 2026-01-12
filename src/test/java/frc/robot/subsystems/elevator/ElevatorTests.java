package frc.robot.subsystems.elevator;

import static org.mockito.Mockito.*;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.*;
import org.mockito.*;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.SuperStructure;

public class ElevatorTests {
    @Mock
    private IElevator mockElevatorIO;

    private Elevator elevator;

    @BeforeEach
    void setUp() {
        MockitoAnnotations.openMocks(this);
        elevator = new Elevator(false); // Assuming 'false' initializes ElevatorSim
        elevator._elevatorIO = mockElevatorIO; // Injecting mock
    }

    @Test
    void testAtSetpoint() {
        elevator.setPositionSetpoint(ElevatorPosition.kL2); // Set the desired position
        // Simulate the elevator reaching the setpoint
        SuperStructure.Elevator.DistanceMeters = ElevatorMap.PositionMap.get(ElevatorPosition.kL2);
        SuperStructure.Elevator.SpeedMPS = 0.0;
        elevator.periodic(); // Call periodic to update the state (simulate one tick of the control loop)

        assertTrue(elevator.atSetpoint(), "Elevator should be at setpoint.");
    }

    @Test
    void testPositionIsNear() {
        // Set the elevator's current position
        SuperStructure.Elevator.DistanceMeters = 0.16; // Corresponds to kSource

        assertTrue(elevator.positionIsNear(ElevatorPosition.kSource, 1.0),
                "Elevator should be within 1 cm of kSource position.");
    }

    @Test
    void testGetElevatorPositionAtLocation() {
        var position = elevator.getElevatorPositionAtLocation(ElevatorPosition.kL2);
        assertEquals(0.28, position.in(Units.Meters), 1e-3,
                "kL2 position should be 0.28 meters.");
    }

    @Test
    void testGetElevatorPositionPercent() {
        SuperStructure.Elevator.DistanceMeters = ElevatorMap.MaxHeight / 2; // Assuming MaxElevatorHeight is 0.627
        assertEquals(0.5, elevator.getElevatorPositionPercent(), 1e-3,
                "Elevator should be at 50% of its maximum height.");
    }

    @Test
    void testSetElevatorSetpointCommand() {
        // "Enables" the robot simulation
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        CommandScheduler.getInstance().enable();

        var command = elevator.setElevatorSetpointCommand(ElevatorPosition.kL3);
        command.schedule();
        CommandScheduler.getInstance().run();

        assertEquals(ElevatorMap.PositionMap.get(ElevatorPosition.kL3),
                elevator._elevatorController.getSetpoint(),
                1e-3,
                "Setpoint should be set to 0.432 meters for kL3.");
    }

    @Test
    void testStopMotorsCommand() {
        var command = elevator.stopMotorsCommand();
        command.initialize(); // Simulate command initialization

        verify(mockElevatorIO, times(1)).stopMotors();
    }

    @Test
    void testGoToElevatorBottomCommand() {
        // Simulate the bottom limit switch being triggered
        SuperStructure.Elevator.BottomLimitSwitch = true;

        // "Enables" the robot simulation
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        CommandScheduler.getInstance().enable();

        var command = elevator.homeLiftCommand();
        command.schedule();
        CommandScheduler.getInstance().run();

        verify(mockElevatorIO, times(1)).setMotorSpeeds(-ElevatorMap.MaxSpeedCoefficient / 2);
        verify(mockElevatorIO, times(1)).stopMotors();
        assertEquals(0.0, elevator._elevatorController.getSetpoint(), 1e-3,
                "Setpoint should be reset to 0 after reaching the bottom.");
    }
}
