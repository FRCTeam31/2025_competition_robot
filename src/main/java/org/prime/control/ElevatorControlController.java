package org.prime.control;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Commands;

public class ElevatorControlController {
    private double _multiplier;
    private double _ramp;

    private double _staticFriction;
    private double _gravity;

    // Used for finding constants
    private boolean constantsFound = false;
    private boolean shouldCheck = true;
    private boolean shouldCheckForS = true;
    private boolean shouldCheckForG = false;
    private double voltageOut = 0;

    private boolean readyToFindUpwardsVoltage = false;
    private boolean upwardsVoltageFound = false;
    private double upwardsVoltage = 0;

    private boolean readyToFindDownwardsVoltage = false;
    private boolean downwardsVoltageFound = false;
    private double downwardsVoltage = 0;

    private boolean readyToFindUpwardVolcity = false;
    private boolean upwardVelocityFound = false;
    private double upwardVelocity = 0;
    private double upperTravelPosition = 0;

    private boolean readyToFindDownwardVelocity = false;
    private boolean downwardVelocityFound = false;
    private double downwardVelocity = 0;
    private double lowerTravelPosition = 0;

    private double foundS = 0;
    private double foundG = 0;

    public ElevatorControlController(double M, double R, double S, double G) {
        _multiplier = M;
        _ramp = R;
        _staticFriction = S;
        _gravity = G;
    }

    public ElevatorControlController(ElevatorControlConstants constants) {
        _multiplier = constants.M;
        _ramp = constants.R;
        _staticFriction = constants.S;
        _gravity = constants.G;
    }

    public double calculate(double setpoint, double position, double velocity) {
        double error = setpoint - position;
        double ramped = error * _ramp;
        double directional = (velocity > 0 ? ramped * _gravity : ramped) + _staticFriction;
        double output = directional * _multiplier;

        return output;
    }

    public void findConstants(
            double lowerPositionBound,
            double upperPositionBound,
            double safeZones,
            double setpointDeadband,
            double safeMovementVoltage,
            double voltageIncreasePerTick,
            double targetVelocity,
            double safeTravelDistance,
            Supplier<Double> positionSupplier,
            Supplier<Double> velocitySupplier,
            Consumer<Double> voltageControlConsumer) {

        double position = positionSupplier.get();
        double velocity = velocitySupplier.get();

        Commands.run(() -> {

            var belowLowerBound = position <= lowerPositionBound + safeZones;
            var aboveUpperBound = position >= upperPositionBound - safeZones;

            if (belowLowerBound) {
                voltageControlConsumer.accept(safeMovementVoltage);
                voltageOut = 0;
                shouldCheck = false;
            } else if (aboveUpperBound) {
                voltageControlConsumer.accept(-safeMovementVoltage);
                voltageOut = 0;
                shouldCheck = false;
            } else {
                readyToFindUpwardsVoltage = true;
                shouldCheck = true;
            }

            if (shouldCheck) {
                if (shouldCheckForS) {
                    if (readyToFindUpwardsVoltage && !upwardsVoltageFound) {
                        voltageOut += voltageIncreasePerTick;
                        voltageControlConsumer.accept(voltageOut);

                        if (velocity >= targetVelocity) {
                            upwardsVoltageFound = true;
                            readyToFindUpwardsVoltage = false;
                            upwardsVoltage = voltageOut;
                            voltageOut = 0;
                        }
                    }

                    if (readyToFindDownwardsVoltage && !downwardsVoltageFound) {
                        voltageOut -= voltageIncreasePerTick;
                        voltageControlConsumer.accept(voltageOut);

                        if (velocity <= -targetVelocity) {
                            downwardsVoltageFound = true;
                            readyToFindDownwardsVoltage = false;
                            downwardsVoltage = voltageOut;
                            voltageOut = 0;
                        }
                    }

                    if (upwardsVoltageFound && downwardsVoltageFound) {
                        foundS = (upwardsVoltage + downwardsVoltage) / 2;
                        readyToFindUpwardVolcity = true;

                        shouldCheckForS = false;
                        shouldCheckForG = true;

                        upperTravelPosition = position + safeTravelDistance;
                        lowerTravelPosition = position - safeTravelDistance;
                    }

                }

                if (shouldCheckForG) {
                    if (readyToFindUpwardVolcity && !upwardVelocityFound) {
                        if (position < upperTravelPosition) {
                            voltageControlConsumer.accept(safeMovementVoltage);
                        } else if (position >= upperTravelPosition) {
                            upwardVelocity = velocity;
                            readyToFindUpwardVolcity = false;
                            readyToFindDownwardVelocity = true;
                            upwardVelocityFound = true;
                        }
                    }

                    if (readyToFindDownwardVelocity && !downwardVelocityFound) {
                        if (position > lowerTravelPosition) {
                            voltageControlConsumer.accept(-safeMovementVoltage);
                        } else if (position <= lowerTravelPosition) {
                            downwardVelocity = velocity;
                            readyToFindDownwardVelocity = false;
                            downwardVelocityFound = true;
                        }
                    }

                    if (upwardVelocityFound && downwardVelocityFound) {
                        foundG = downwardVelocity / upwardVelocity;

                        shouldCheckForG = false;
                    }
                }
            }

        }).until(() -> constantsFound).schedule();

        _multiplier = 1;
        _ramp = 1;
        _staticFriction = foundS;
        _gravity = foundG;
    }
}
