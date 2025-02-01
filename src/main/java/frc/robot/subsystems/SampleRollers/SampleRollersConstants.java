package frc.robot.subsystems.SampleRollers;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Ports;
import frc.robot.subsystems.GenericRollerSubsystem.GenericRollerSubsystemConstants;
import java.util.List;

/** Add your docs here. */
public final class SampleRollersConstants {

    public static final GenericRollerSubsystemConstants kSubSysConstants =
        new GenericRollerSubsystemConstants();

    static {
        kSubSysConstants.kName = "SampleRollers";

        // kMotorConstants.kMotorIDs = List.of(Ports.SAMPLE_ROLLER);
        kSubSysConstants.kMotorIDs = List.of(Ports.TWO_ROLLER_1, Ports.TWO_ROLLER_2);

        kSubSysConstants.kMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kSubSysConstants.kMotorConfig.MotorOutput.Inverted =
            InvertedValue.CounterClockwise_Positive;
        kSubSysConstants.kMotorConfig.Voltage.PeakForwardVoltage = 12.0;
        kSubSysConstants.kMotorConfig.Voltage.PeakReverseVoltage = -12.0;

        kSubSysConstants.kMotorConfig.CurrentLimits.SupplyCurrentLimit = 20;
        kSubSysConstants.kMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        kSubSysConstants.kMotorConfig.CurrentLimits.StatorCurrentLimit = 70;
        kSubSysConstants.kMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // Motor simulation
        kSubSysConstants.simMotorModelSupplier = () -> DCMotor.getKrakenX60Foc(1);
        kSubSysConstants.simReduction = (18.0 / 12.0);
        kSubSysConstants.simMOI = 0.001;
    }
}
