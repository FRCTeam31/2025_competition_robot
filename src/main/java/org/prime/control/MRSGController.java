package org.prime.control;

import edu.wpi.first.math.MathUtil;

public class MRSGController {
    private double _multiplier;
    private double _ramp;
    private double _staticFriction;
    private double _gravity;

    private double _setpoint = -0.01;

    // Physical Variables
    private double _position;
    private double _error;

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

    public double calculate(double setpoint, double position, double velocity) {
        _position = position;

        _error = setpoint - position;
        double ramped = _error * _ramp;

        double directional = velocity >= 0
                ? (ramped * _gravity) + _staticFriction
                : ramped - _staticFriction;
        double scaled = directional * _multiplier;
        double clamped = MathUtil.clamp(scaled, -12, 12);

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
