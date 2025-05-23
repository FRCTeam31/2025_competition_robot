package org.prime.control;

import edu.wpi.first.math.MathUtil;

public class MRSGController {
    private double _multiplier;
    private double _ramp;

    private double _staticFriction;
    private double _gravity;

    private double _setpoint = -0.01;
    private double _frictionDeadband = 0.1;

    // Slowdown Constants
    private double _slowdownStart = 0.5;
    private double _slowestSlowdownPercent = 0.05;

    // Physical Variables
    private double _position;
    private double _velocity;
    private double _error;

    // Physical Constants
    private double _maxTravel;

    public MRSGController(double M, double R, double S, double G) {
        _multiplier = M;
        _ramp = R;
        _staticFriction = S;
        _gravity = G;
    }

    public MRSGController(MRSGConstants constants) {
        _multiplier = constants.M;
        _ramp = constants.R;
        _staticFriction = constants.S;
        _gravity = constants.G;
    }

    public MRSGController(MRSGConstants constants, double maxTravel) {
        _multiplier = constants.M;
        _ramp = constants.R;
        _staticFriction = constants.S;
        _gravity = constants.G;

        _maxTravel = maxTravel;
    }

    // public MRSGController(double M, double R, double S, double G, double frictionDeadband) {
    //     _multiplier = M;
    //     _ramp = R;
    //     _staticFriction = S;
    //     _gravity = G;

    //     _frictionDeadband = frictionDeadband;
    // }

    // public MRSGController(MRSGConstants constants, double frictionDeadband) {
    //     _multiplier = constants.M;
    //     _ramp = constants.R;
    //     _staticFriction = constants.S;
    //     _gravity = constants.G;

    //     _frictionDeadband = frictionDeadband;
    // }

    public double calculate(double setpoint, double position, double velocity) {
        _position = position;
        _velocity = velocity;

        double error = setpoint - position;
        // double percentTravel = error / _maxTravel;
        // double clampedPercentTravel = Math.min(_slowestSlowdownPercent, Math.max(_slowdownStart, percentTravel));
        // double alteredRamp = (_ramp / clampedPercentTravel) * _slowdownStart;

        // double ramped = error * alteredRamp;
        double ramped = error * _ramp;

        double directional = velocity >= 0
                ? (ramped * _gravity) + _staticFriction
                : ramped - _staticFriction;
        double scaled = directional * _multiplier;
        // double dampeningFactor = Math.tanh(error * 0.5);

        // double output = (scaled <= _staticFriction) || scaled - _staticFriction < _frictionDeadband ? 0 : scaled;
        double output = scaled;

        double clamped = MathUtil.clamp(output, -12, 12);

        _error = error;
        return clamped;
    }

    public double calculate(double position, double velocity) {
        if (_setpoint < 0) {
            _setpoint = position;
        }

        return calculate(_setpoint, position, velocity);
    }

    public void setSetpoint(double setpoint) {
        _setpoint = setpoint;
    }

    public double getSetpoint() {
        return _setpoint;
    }

    public double getError() {
        return _error;
    }

    public boolean atSetpoint(double deadband) {
        return Math.abs(_setpoint - _position) < deadband;
    }

    public boolean atSetpoint() {
        return atSetpoint(0.01);
    }

    public void setM(double M) {
        _multiplier = M;
    }

    public void setR(double R) {
        _ramp = R;
    }

    public void setS(double S) {
        _staticFriction = S;
    }

    public void setG(double G) {
        _gravity = G;
    }

    public double getM() {
        return _multiplier;
    }

    public double getR() {
        return _ramp;
    }

    public double getS() {
        return _staticFriction;
    }

    public double getG() {
        return _gravity;
    }
}
