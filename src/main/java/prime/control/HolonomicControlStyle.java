package prime.control;

public enum HolonomicControlStyle {
  /**
   * Controls a swerve drive with the left stick controlling forward and strafe, and the triggers controlling rotation
   */
  Standard,

  /**
   * Controls a swerve drive with the left stick's X-axis controlling rotation and the right stick controlling forward and strafe
   */
  Drone,
}
