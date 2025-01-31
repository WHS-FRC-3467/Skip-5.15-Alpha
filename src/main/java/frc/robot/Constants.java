// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
// import frc.robot.generated.TunerConstants;

public class Constants {

  public static double loopPeriodSecs = 0.02;

  // Use LoggedTunableNumbers
  public static final boolean tuningMode = true;

  /**
   * This enum defines the runtime mode used by AdvantageKit. The mode is always "real" when running
   * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
   * (log replay from a file).
   */
  public static final Mode simMode = Mode.SIM;

  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,
    /** Running a physics simulator. */
    SIM,
    /** Replaying from a log file. */
    REPLAY
  }

  public static final class ExampleSimpleSubsystemConstants {
    public static final int ID_Motor = 10;

    public static TalonFXConfiguration motorConfig() {
      TalonFXConfiguration m_configuration = new TalonFXConfiguration();

      m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      m_configuration.Voltage.PeakForwardVoltage = 12.0;
      m_configuration.Voltage.PeakReverseVoltage = -12.0;

      m_configuration.CurrentLimits.SupplyCurrentLimit = 20;
      m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
      m_configuration.CurrentLimits.StatorCurrentLimit = 70;
      m_configuration.CurrentLimits.StatorCurrentLimitEnable = true;

      return m_configuration;
    }
  }

  public static final class ExampleComplexSubsystemConstants {
    public static final int ID_Motor = 11;
    public static final double upperLimit = Units.degreesToRadians(180);
    public static final double lowerLimit = Units.degreesToRadians(0);
    public static final double tolerance = Units.degreesToRadians(1);

    public static TalonFXConfiguration motorConfig() {
      TalonFXConfiguration m_configuration = new TalonFXConfiguration();

      m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      m_configuration.Voltage.PeakForwardVoltage = 12.0;
      m_configuration.Voltage.PeakReverseVoltage = -12.0;

      m_configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
      m_configuration.Feedback.SensorToMechanismRatio = 1;

      m_configuration.Slot0.kP = 1; // output per unit of error in position (output/rotation)
      m_configuration.Slot0.kI =
          0; // output per unit of integrated error in position (output/(rotation*s))
      m_configuration.Slot0.kD = 0; // output per unit of error derivative in position (output/rps)

      m_configuration.Slot1.kG = 0; // output to overcome gravity (output)
      m_configuration.Slot1.kS = 0; // output to overcome static friction (output)
      m_configuration.Slot1.kV = 0; // output per unit of requested velocity (output/rps)
      m_configuration.Slot1.kA = 0; // unused, as there is no target acceleration
      m_configuration.Slot1.kP = 1; // output per unit of error in position (output/rotation)
      m_configuration.Slot1.kI =
          0; // output per unit of integrated error in position (output/(rotation*s))
      m_configuration.Slot1.kD = 0; // output per unit of error derivative in position (output/rps)

      m_configuration.MotionMagic.MotionMagicCruiseVelocity = 10;
      m_configuration.MotionMagic.MotionMagicAcceleration = 10;
      m_configuration.MotionMagic.MotionMagicJerk = 10;

      m_configuration.CurrentLimits.SupplyCurrentLimit = 20;
      m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
      m_configuration.CurrentLimits.StatorCurrentLimit = 70;
      m_configuration.CurrentLimits.StatorCurrentLimitEnable = true;

      return m_configuration;
    }
  }

  /*
   * public static final class DriveConstants {
   * public static final double headingAngleTolerance = 2.0;
   * public static final double MaxSpeed = 4.70;
   * // TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
   * public static final double MaxAngularRate =
   * 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
   *
   * public static TalonFXConfiguration motorConfig() {
   * TalonFXConfiguration m_configuration = new TalonFXConfiguration();
   *
   * m_configuration.CurrentLimits.SupplyCurrentLimit = 30;
   * m_configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
   * m_configuration.CurrentLimits.StatorCurrentLimit = 80;
   * m_configuration.CurrentLimits.StatorCurrentLimitEnable = true;
   *
   * return m_configuration;
   * }
   *
   * public static TalonFXConfiguration motor2Config() {
   * TalonFXConfiguration m_configuration = motorConfig();
   * m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
   * return m_configuration;
   * }
   * }
   */

  public static class FieldConstants {

    // BLUE SIDE
    public static final Pose2d BLUE_REEF_AB =
        new Pose2d(
            Units.inchesToMeters(144.00), // X Pose
            Units.inchesToMeters(158.50), // Y Pose
            // Rotation - Do a 180 from the direction the april tag faces to get the direction the
            // robot should face
            new Rotation2d(Units.degreesToRadians(0))); // Rotation (degrees to radians)
    public static final Pose2d BLUE_REEF_CD =
        new Pose2d(
            Units.inchesToMeters(160.39), // X Pose
            Units.inchesToMeters(130.17), // Y Pose
            new Rotation2d(Units.degreesToRadians(60))); // Rotation
    public static final Pose2d BLUE_REEF_EF =
        new Pose2d(
            Units.inchesToMeters(193.1), // X Pose
            Units.inchesToMeters(130.17), // Y Pose
            new Rotation2d(Units.degreesToRadians(120))); // Rotation
    public static final Pose2d BLUE_REEF_GH =
        new Pose2d(
            Units.inchesToMeters(209.49), // X Pose
            Units.inchesToMeters(158.50), // Y Pose
            new Rotation2d(Units.degreesToRadians(180))); // Rotation
    public static final Pose2d BLUE_REEF_IJ =
        new Pose2d(
            Units.inchesToMeters(193.1), // X Pose
            Units.inchesToMeters(180.83), // Y Pose
            new Rotation2d(Units.degreesToRadians(-120))); // Rotation
    public static final Pose2d BLUE_REEF_KL =
        new Pose2d(
            Units.inchesToMeters(160.39), // X Pose
            Units.inchesToMeters(186.83), // Y Pose
            new Rotation2d(Units.degreesToRadians(-60))); // Rotation
    public static final Pose2d BLUE_SUBSTATION_LEFT =
        new Pose2d(
            Units.inchesToMeters(0.0), // X Pose
            Units.inchesToMeters(0.0), // Y Pose
            new Rotation2d()); // Rotation
    public static final Pose2d BLUE_SUBSTATION_RIGHT =
        new Pose2d(
            Units.inchesToMeters(0.0), // X Pose
            Units.inchesToMeters(0.0), // Y Pose
            new Rotation2d()); // Rotation
    public static final Pose2d BLUE_PROCESSOR =
        new Pose2d(
            Units.inchesToMeters(235.726), // X Pose
            Units.inchesToMeters(0.0), // Y Pose
            Rotation2d.fromDegrees(90)); // Rotation
    public static final Pose2d BLUE_NET =
        new Pose2d(
            Units.inchesToMeters(0.0), // X Pose
            Units.inchesToMeters(0.0), // Y Pose
            new Rotation2d()); // Rotation
    public static final Pose2d BLUE_BARGE =
        new Pose2d(
            Units.inchesToMeters(0.0), // X Pose
            Units.inchesToMeters(0.0), // Y Pose
            new Rotation2d()); // Rotation

    // RED SIDE
    public static final Pose2d RED_REEF_AB =
        new Pose2d(
            Units.inchesToMeters(546.87), // X Pose
            Units.inchesToMeters(158.50), // Y Pose
            new Rotation2d(Units.degreesToRadians(180))); // Rotation (degrees to radians).
    // Rotation - Do a 180 from the direction the april tag faces to get the direction the robot
    // should face
    public static final Pose2d RED_REEF_CD =
        new Pose2d(
            Units.inchesToMeters(530.49), // X Pose
            Units.inchesToMeters(186.83), // Y Pose
            new Rotation2d(Units.degreesToRadians(-120))); // Rotation
    public static final Pose2d RED_REEF_EF =
        new Pose2d(
            Units.inchesToMeters(497.77), // X Pose
            Units.inchesToMeters(186.83), // Y Pose
            new Rotation2d(Units.degreesToRadians(-60))); // Rotation
    public static final Pose2d RED_REEF_GH =
        new Pose2d(
            Units.inchesToMeters(481.39), // X Pose
            Units.inchesToMeters(158.50), // Y Pose
            new Rotation2d(Units.degreesToRadians(0))); // Rotation
    public static final Pose2d RED_REEF_IJ =
        new Pose2d(
            Units.inchesToMeters(497.77), // X Pose
            Units.inchesToMeters(130.37), // Y Pose
            new Rotation2d(Units.degreesToRadians(60))); // Rotation
    public static final Pose2d RED_REEF_KL =
        new Pose2d(
            Units.inchesToMeters(530.49), // X Pose
            Units.inchesToMeters(130.17), // Y Pose
            new Rotation2d(Units.degreesToRadians(120))); // Rotation
    public static final Pose2d RED_SUBSTATION_LEFT =
        new Pose2d(
            Units.inchesToMeters(0.0), // X Pose
            Units.inchesToMeters(0.0), // Y Pose
            new Rotation2d()); // Rotation
    public static final Pose2d RED_SUBSTATION_RIGHT =
        new Pose2d(
            Units.inchesToMeters(0.0), // X Pose
            Units.inchesToMeters(0.0), // Y Pose
            new Rotation2d()); // Rotation
    public static final Pose2d RED_PROCESSOR =
        new Pose2d(
            Units.inchesToMeters(690.875 - 235.726), // X Pose
            Units.inchesToMeters(0.0), // Y Pose
            Rotation2d.fromDegrees(-90)); // Rotation
    public static final Pose2d RED_NET =
        new Pose2d(
            Units.inchesToMeters(0.0), // X Pose
            Units.inchesToMeters(0.0), // Y Pose
            new Rotation2d()); // Rotation
    public static final Pose2d RED_BARGE =
        new Pose2d(
            Units.inchesToMeters(0.0), // X Pose
            Units.inchesToMeters(0.0), // Y Pose
            new Rotation2d()); // Rotation

    public static final Pose2d BLUE_REEF_CENTER =
        new Pose2d(Units.inchesToMeters(196.75), Units.inchesToMeters(158.5), new Rotation2d());

    public static final Pose2d RED_REEF_CENTER =
        new Pose2d(Units.inchesToMeters(514.13), Units.inchesToMeters(158.5), new Rotation2d());

    public static final double FIELD_X_LENGTH = Units.inchesToMeters(690.875);
    public static final double FIELD_Y_LENGTH = Units.inchesToMeters(317.0);
    // public static final Pose2d BLUE_SPEAKER =
    // new Pose2d(
    // Units.inchesToMeters(-1.5 + 12), Units.inchesToMeters(218.42), new
    // Rotation2d(0));
    // public static final Pose2d RED_SPEAKER =
    // new Pose2d(
    // Units.inchesToMeters(652.73 - 12),
    // Units.inchesToMeters(218.42),
    // new Rotation2d(Math.PI));
    // public static final Pose2d BLUE_FEED = new Pose2d(1.25, 6.62, new
    // Rotation2d(0));
    // public static final Pose2d RED_FEED = new Pose2d(15.250, 6.62, new
    // Rotation2d(0));
    // public static final Pose2d BLUE_AMP =
    // new Pose2d(
    // Units.inchesToMeters(72.5), Units.inchesToMeters(323.00), new
    // Rotation2d(Math.PI / 2));
    // public static final Pose2d RED_AMP =
    // new Pose2d(
    // Units.inchesToMeters(578.77),
    // Units.inchesToMeters(323.00),
    // new Rotation2d(-Math.PI / 2));
    // public static final double BLUE_AUTO_PENALTY_LINE =
    // 9; // X distance from origin to center of the robot almost fully crossing the
    // midline
    // public static final double RED_AUTO_PENALTY_LINE =
    // 7.4; // X distance from origin to center of the robot almost fully crossing
    // the midline

    // public static final Rotation2d ampAngle = new Rotation2d(Math.PI / 2);
  }
}
