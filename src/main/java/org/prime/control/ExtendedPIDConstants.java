package org.prime.control;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class ExtendedPIDConstants {

  public double kP;
  public double kI;
  public double kD;
  public double kF;

  // See https://docs.wpilib.org/en/latest/docs/software/advanced-controls/introduction/introduction-to-feedforward.html#the-permanent-magnet-dc-motor-feedforward-equation
  public double kV; // Voltage needed to hold a constant velocity while overcoming any additional friction that increases with speed
  public double kA; // Voltage needed to induce a given acceleration in the system
  public double kS; // Voltage needed to overcome the system's static friction

  public ExtendedPIDConstants(double kP, double kI, double kD, double kF, double kV, double kA, double kS) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kF = kF;
    this.kV = kV;
    this.kA = kA;
    this.kS = kS;
  }

  public ExtendedPIDConstants() {
    this(0, 0, 0, 0, 0, 0, 0);
  }

  public ExtendedPIDConstants(double kP, double kI, double kD) {
    this(kP, kI, kD, 0, 0, 0, 0);
  }

  public ExtendedPIDConstants(double kP, double kI, double kD, double kF) {
    this(kP, kI, kD, kF, 0, 0, 0);
  }

  public ExtendedPIDConstants(double kP, double kI, double kD, double kF, double kV) {
    this(kP, kI, kD, kF, kV, 0, 0);
  }

  public PIDConstants toPIDConstants() {
    return new PIDConstants(kP, kI, kD);
  }

  public PIDController createPIDController(double controllerPeriod) {
    return new PIDController(kP, kI, kD, controllerPeriod);
  }

  public SimpleMotorFeedforward createSimpleMotorFeedForward() {
    return new SimpleMotorFeedforward(kS, kV, kA);
  }
}
