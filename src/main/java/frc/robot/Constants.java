// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {

    public static double loopPeriodSecs = 0.02;

    // Use LoggedTunableNumbers
    public static final boolean tuningMode = true;

    /**
     * This enum defines the runtime mode used by AdvantageKit. The mode is always "real" when
     * running on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and
     * "replay" (log replay from a file).
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

        public static TalonFXConfiguration motorConfig()
        {
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

        public static TalonFXConfiguration motorConfig()
        {
            TalonFXConfiguration m_configuration = new TalonFXConfiguration();

            m_configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            m_configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            m_configuration.Voltage.PeakForwardVoltage = 12.0;
            m_configuration.Voltage.PeakReverseVoltage = -12.0;

            m_configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
            m_configuration.Feedback.SensorToMechanismRatio = 1;

            m_configuration.Slot0.kP = 1; // output per unit of error in position (output/rotation)
            m_configuration.Slot0.kI = 0; // output per unit of integrated error in position
            // (output/(rotation*s))
            m_configuration.Slot0.kD = 0; // output per unit of error derivative in position
            // (output/rps)

            m_configuration.Slot1.kG = 0; // output to overcome gravity (output)
            m_configuration.Slot1.kS = 0; // output to overcome static friction (output)
            m_configuration.Slot1.kV = 0; // output per unit of requested velocity (output/rps)
            m_configuration.Slot1.kA = 0; // unused, as there is no target acceleration
            m_configuration.Slot1.kP = 1; // output per unit of error in position (output/rotation)
            m_configuration.Slot1.kI = 0; // output per unit of integrated error in position
            // (output/(rotation*s))
            m_configuration.Slot1.kD = 0; // output per unit of error derivative in position
            // (output/rps)

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
}


