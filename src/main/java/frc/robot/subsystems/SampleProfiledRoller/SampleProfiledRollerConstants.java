package frc.robot.subsystems.SampleProfiledRoller;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Ports;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemConstants;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemConstants.simType;

/** Add your docs here. */
public final class SampleProfiledRollerConstants {

  public static final GenericMotionProfiledSubsystemConstants kSubSysConstants =
      new GenericMotionProfiledSubsystemConstants();

  static {
    kSubSysConstants.kName = "SampleProfiledRoller";

    kSubSysConstants.kLeaderMotor = Ports.PROFROLLER_MAIN;
    kSubSysConstants.kFollowMotor = null;
    kSubSysConstants.kFollowerOpposesMain = true;

    // Using TalonFX internal encoder

    kSubSysConstants.kCANcoder = null;
    kSubSysConstants.kMotorConfig.Feedback.FeedbackSensorSource =
        FeedbackSensorSourceValue.RotorSensor;
    kSubSysConstants.kMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
    kSubSysConstants.kMotorConfig.Feedback.RotorToSensorRatio = 1.0;

    // Using a remote CANcoder
    /*
     * kSubSysConstants.kCANcoder = Ports.ARM_CANCODER;
     * kSubSysConstants.kMotorConfig.Feedback.FeedbackSensorSource =
     * FeedbackSensorSourceValue.FusedCANcoder;
     * kSubSysConstants.kMotorConfig.Feedback.SensorToMechanismRatio = 7.04;
     * kSubSysConstants.kMotorConfig.Feedback.RotorToSensorRatio = 54.4/7.04;
     * kSubSysConstants.kEncoderConfig.MagnetSensor.MagnetOffset = 0.3467;
     * kSubSysConstants.kEncoderConfig.MagnetSensor.SensorDirection =
     * SensorDirectionValue.Clockwise_Positive;
     * kSubSysConstants.kEncoderConfig.MagnetSensor.AbsoluteSensorRange =
     * AbsoluteSensorRangeValue.Unsigned_0To1;
     */

    kSubSysConstants.kMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    kSubSysConstants.kMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    kSubSysConstants.kMotorConfig.Voltage.PeakForwardVoltage = 12.0;
    kSubSysConstants.kMotorConfig.Voltage.PeakReverseVoltage = -12.0;

    kSubSysConstants.kMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    kSubSysConstants.kMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kSubSysConstants.kMotorConfig.CurrentLimits.StatorCurrentLimit = 70;
    kSubSysConstants.kMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    /* REAL system profile constants */
    kSubSysConstants.kMotorConfig.Slot0.kP = 0;
    kSubSysConstants.kMotorConfig.Slot0.kI = 0;
    kSubSysConstants.kMotorConfig.Slot0.kD = 0;
    kSubSysConstants.kMotorConfig.Slot0.kG = 0;
    kSubSysConstants.kMotorConfig.Slot0.kS = 0;
    kSubSysConstants.kMotorConfig.Slot0.kV = 0;
    kSubSysConstants.kMotorConfig.Slot0.kA = 0;
    kSubSysConstants.kMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 0;
    kSubSysConstants.kMotorConfig.MotionMagic.MotionMagicAcceleration = 0;
    kSubSysConstants.kMotorConfig.MotionMagic.MotionMagicJerk = 0;

    /* SIM system profile constants */
    kSubSysConstants.kSimMotorConfig.Slot0.kP = 500;
    kSubSysConstants.kSimMotorConfig.Slot0.kI = 0;
    kSubSysConstants.kSimMotorConfig.Slot0.kD = 20;
    kSubSysConstants.kSimMotorConfig.Slot0.kG = 0;
    kSubSysConstants.kSimMotorConfig.Slot0.kS = 0;
    kSubSysConstants.kSimMotorConfig.Slot0.kV = 0.19;
    kSubSysConstants.kSimMotorConfig.Slot0.kA = 0;
    kSubSysConstants.kSimMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 500;
    kSubSysConstants.kSimMotorConfig.MotionMagic.MotionMagicAcceleration = 50;
    kSubSysConstants.kSimMotorConfig.MotionMagic.MotionMagicJerk = 0;

    // Simulation Type
    kSubSysConstants.SimType = simType.ROLLER;

    // Motor simulation
    kSubSysConstants.kMotorSimConfig.simMotorModelSupplier = () -> DCMotor.getKrakenX60Foc(1);
    kSubSysConstants.kMotorSimConfig.simReduction = 1;
  }
}
