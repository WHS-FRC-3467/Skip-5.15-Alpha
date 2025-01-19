package frc.robot.subsystems.drive;

public final class DriveConstants {
  public static final double headingAngleTolerance = 5.0;
  public static final double MaxSpeed =
      4.73; // desired top speed //  public static final LinearVelocity kSpeedAt12Volts =
  // MetersPerSecond.of(4.73)
  public static final double MaxAngularRate =
      1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  public static final double driverSpeed = 0.75; // Multiplier to the controller input
}
