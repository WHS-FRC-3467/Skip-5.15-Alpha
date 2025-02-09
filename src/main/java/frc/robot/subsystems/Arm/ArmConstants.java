package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Ports;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemConstants;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemConstants.simType;

/** Add your docs here. */
public final class ArmConstants {

    public static final GenericMotionProfiledSubsystemConstants kSubSysConstants =
        new GenericMotionProfiledSubsystemConstants();

    public static final double kHomingCurrent = 2.0;

    static {
        kSubSysConstants.kName = "Arm";

        // This is the minimum tolerance that will be used by the atPosition() method.
        // It will be used even if you pass a smaller value into atPosition().
        // If you want to specify a larger value on an individual call basis, then you
        // should pass that value into atPosition()
        kSubSysConstants.kminTolerance = 0.01;

        kSubSysConstants.kLeaderMotor = Ports.ARM_MAIN;

        // Using TalonFX internal encoder
        /* 
        kSubSysConstants.kCANcoder = null;
        kSubSysConstants.kMotorConfig.Feedback.FeedbackSensorSource =
            FeedbackSensorSourceValue.RotorSensor;
        kSubSysConstants.kMotorConfig.Feedback.SensorToMechanismRatio = 54.4;
        kSubSysConstants.kMotorConfig.Feedback.RotorToSensorRatio = 1.0; */

        // Using a remote CANcoder
        kSubSysConstants.kCANcoder = Ports.ARM_CANCODER;
        kSubSysConstants.kMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        kSubSysConstants.kMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
        kSubSysConstants.kMotorConfig.Feedback.RotorToSensorRatio = 104.1322; //Gearbox (15/1)*big gears(70/22)*chain(48/22)
        kSubSysConstants.kEncoderConfig.MagnetSensor.MagnetOffset = 0.0; // TODO: Tune arm encoder offset
        kSubSysConstants.kEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        // Should be the center of the positions where the arm cannot go, in rotations
        kSubSysConstants.kEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.58; 
        

        kSubSysConstants.kMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kSubSysConstants.kMotorConfig.MotorOutput.Inverted =
            InvertedValue.CounterClockwise_Positive;
        kSubSysConstants.kMotorConfig.Voltage.PeakForwardVoltage = 12.0;
        kSubSysConstants.kMotorConfig.Voltage.PeakReverseVoltage = -12.0;

        kSubSysConstants.kMotorConfig.CurrentLimits.SupplyCurrentLimit = 20;
        kSubSysConstants.kMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        kSubSysConstants.kMotorConfig.CurrentLimits.StatorCurrentLimit = 70;
        kSubSysConstants.kMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        /* REAL system profile constants */
        kSubSysConstants.kMotorConfig.Slot0.kP = 0;
        kSubSysConstants.kMotorConfig.Slot0.kI = 0;
        kSubSysConstants.kMotorConfig.Slot0.kD = 0;
        kSubSysConstants.kMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        kSubSysConstants.kMotorConfig.Slot0.kG = 0;
        kSubSysConstants.kMotorConfig.Slot0.kS = 0;
        kSubSysConstants.kMotorConfig.Slot0.kV = 0;
        kSubSysConstants.kMotorConfig.Slot0.kA = 0;
        kSubSysConstants.kMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 0;
        kSubSysConstants.kMotorConfig.MotionMagic.MotionMagicAcceleration = 0;
        kSubSysConstants.kMotorConfig.MotionMagic.MotionMagicJerk = 0;

        /* SIM system profile constants */
        kSubSysConstants.kSimMotorConfig.Slot0.kP = 700;
        kSubSysConstants.kSimMotorConfig.Slot0.kI = 0;
        kSubSysConstants.kSimMotorConfig.Slot0.kD = 100;
        kSubSysConstants.kSimMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        kSubSysConstants.kSimMotorConfig.Slot0.kG = 13;
        kSubSysConstants.kSimMotorConfig.Slot0.kS = 0;
        kSubSysConstants.kSimMotorConfig.Slot0.kV = 0.19;
        kSubSysConstants.kSimMotorConfig.Slot0.kA = 0;
        kSubSysConstants.kSimMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 500;
        kSubSysConstants.kSimMotorConfig.MotionMagic.MotionMagicAcceleration = 50;
        kSubSysConstants.kSimMotorConfig.MotionMagic.MotionMagicJerk = 0;

        // Simulation Type
        kSubSysConstants.SimType = simType.ARM;

        // Motor simulation
        kSubSysConstants.kMotorSimConfig.simMotorModelSupplier = () -> DCMotor.getKrakenX60Foc(2);

        // Arm Simulation
        kSubSysConstants.kArmSimConfig.kIsComboSim = true;
        kSubSysConstants.kArmSimConfig.kArmMass = 8.0; // Kilograms
        kSubSysConstants.kArmSimConfig.kArmLength = Units.inchesToMeters(23.188);
        kSubSysConstants.kArmSimConfig.kDefaultArmSetpointDegrees = 75.0;
        kSubSysConstants.kArmSimConfig.kMinAngleDegrees = -75.0;
        kSubSysConstants.kArmSimConfig.kMaxAngleDegrees = 255.0;
        kSubSysConstants.kArmSimConfig.kArmReduction =
            104.1322; // RotorToSensorRatio * SensorToMechanismRatio
        kSubSysConstants.kArmSimConfig.kSensorReduction = 1.0; // SensorToMechanismRatio
    }
}
