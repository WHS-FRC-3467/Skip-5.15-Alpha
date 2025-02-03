package frc.robot.subsystems.GenericMotionProfiledSubsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public abstract class GenericMotionProfiledSubsystem<G extends GenericMotionProfiledSubsystem.TargetState>
    extends SubsystemBase {

    // Tunable numbers
    private LoggedTunableNumber kP, kI, kD, kG, kS, kV, kA, kCruiseVelocity, kAcceleration, kJerk;

    public enum ProfileType {
        POSITION,
        VELOCITY,
        MM_POSITION,
        MM_VELOCITY,
        OPEN_VOLTAGE,
        OPEN_CURRENT
    }

    public interface TargetState {

        public double getOutput();

        public double getFeedFwd();

        public ProfileType getProfileType();
    }

    public abstract G getState();

    private final String m_name;
    private final GenericMotionProfiledSubsystemConstants m_constants;
    protected final GenericMotionProfiledSubsystemIO io;
    private boolean mIsSim = false;
    private ProfileType m_proType;

    protected final GenericMotionProfiledIOInputsAutoLogged inputs =
        new GenericMotionProfiledIOInputsAutoLogged();
    private final Alert leaderMotorDisconnected;
    private final Alert followerMotorDisconnected;
    private final Alert CANcoderDisconnected;

    public GenericMotionProfiledSubsystem(
        ProfileType defaultProfileType,
        GenericMotionProfiledSubsystemConstants constants,
        GenericMotionProfiledSubsystemIO io,
        boolean isSim)
    {

        this.m_proType = defaultProfileType;
        this.m_constants = constants;
        this.io = io;
        this.mIsSim = isSim;
        this.m_name = m_constants.kName;

        this.leaderMotorDisconnected =
            new Alert(m_name + " Leader motor disconnected!", Alert.AlertType.kWarning);
        this.followerMotorDisconnected =
            new Alert(m_name + " Follower motor disconnected!", Alert.AlertType.kWarning);
        this.CANcoderDisconnected =
            new Alert(m_name + " CANcoder disconnected!", Alert.AlertType.kWarning);

        // Make sure we use the correct profiling configs
        TalonFXConfiguration fxConfig =
            mIsSim ? m_constants.kSimMotorConfig : m_constants.kMotorConfig;

        // Tunable numbers for PID and motion gain constants
        kP = new LoggedTunableNumber(m_name + "/Gains/kP", fxConfig.Slot0.kP);
        kI = new LoggedTunableNumber(m_name + "/Gains/kI", fxConfig.Slot0.kI);
        kD = new LoggedTunableNumber(m_name + "/Gains/kD", fxConfig.Slot0.kD);

        kG = new LoggedTunableNumber(m_name + "/Gains/kG", fxConfig.Slot0.kG);
        kS = new LoggedTunableNumber(m_name + "/Gains/kS", fxConfig.Slot0.kS);
        kV = new LoggedTunableNumber(m_name + "/Gains/kV", fxConfig.Slot0.kV);
        kA = new LoggedTunableNumber(m_name + "/Gains/kA", fxConfig.Slot0.kA);

        kCruiseVelocity =
            new LoggedTunableNumber(
                m_name + "/CruiseVelocity", fxConfig.MotionMagic.MotionMagicCruiseVelocity);
        kAcceleration =
            new LoggedTunableNumber(
                m_name + "/Acceleration", fxConfig.MotionMagic.MotionMagicAcceleration);
        kJerk = new LoggedTunableNumber(m_name + "/Jerk", fxConfig.MotionMagic.MotionMagicJerk);

        io.configurePID(kP.get(), kI.get(), kD.get(), true);
        io.configureGSVA(kG.get(), kS.get(), kV.get(), kA.get(), true);
        io.configureMotion(kCruiseVelocity.get(), kAcceleration.get(), kJerk.get(), true);
    }

    public void periodic()
    {
        // If Profile Type has changed, reset the encoder(s)
        ProfileType newProfType = getState().getProfileType();
        if (m_proType != newProfType) {
            io.zeroSensors();
            m_proType = newProfType;
        }

        io.updateInputs(inputs);
        Logger.processInputs(m_name, inputs);

        // Check for disconnections
        leaderMotorDisconnected.set(!inputs.leaderMotorConnected);
        followerMotorDisconnected.set(
            m_constants.kFollowMotor != null && !inputs.followerMotorConnected);
        CANcoderDisconnected.set(m_constants.kCANcoder != null && !inputs.CANcoderConnected);

        // If changed, update controller constants from Tuneable Numbers
        if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
            io.configurePID(kP.get(), kI.get(), kD.get(), true);
        }

        if (kG.hasChanged(hashCode())
            || kS.hasChanged(hashCode())
            || kV.hasChanged(hashCode())
            || kA.hasChanged(hashCode())) {
            io.configureGSVA(kG.get(), kS.get(), kV.get(), kA.get(), true);
        }

        if (kCruiseVelocity.hasChanged(hashCode())
            || kAcceleration.hasChanged(hashCode())
            || kJerk.hasChanged(hashCode())) {
            io.configureMotion(kCruiseVelocity.get(), kAcceleration.get(), kJerk.get(), true);
        }

        // Run system based on Profile Type
        switch (m_proType) {
            default:
            case POSITION:
                /* Run Closed Loop to position in rotations */
                io.runToPosition(getState().getOutput(), getState().getFeedFwd());
                break;
            case VELOCITY:
                /* Run Closed Loop to velocity in rotations/second */
                io.runToVelocity(getState().getOutput(), getState().getFeedFwd());
                break;
            case MM_POSITION:
                /* Run Motion Magic to the specified position setpoint (in rotations) */
                io.runMotionMagicPosition(getState().getOutput(), getState().getFeedFwd());
                break;
            case MM_VELOCITY:
                /* Run Motion Magic to the specified velocity setpoint (in rotations/second) */
                io.runMotionMagicVelocity(getState().getOutput(), getState().getFeedFwd());
                break;
            case OPEN_VOLTAGE:
                /* Run Open Loop using specified voltage in volts */
                io.runVoltage(getState().getOutput());
                break;
            case OPEN_CURRENT:
                /* Run Open Loop using specified current in amps */
                io.runCurrent(getState().getOutput());
                break;
        }

        displayInfo();
    }

    private void displayInfo()
    {

        Logger.recordOutput(m_name + "/Goal State", getState().toString());
        Logger.recordOutput(m_name + "/Profile Type", getState().getProfileType().toString());

        if (Constants.tuningMode) {
            Logger.recordOutput(m_name + "/Setpoint", io.getSetpoint());
            Logger.recordOutput(m_name + "/Position(Rotations)", io.getPosition());
            Logger.recordOutput(m_name + "/Position(Degrees)", Math.toDegrees(io.getPosition()* 2 * Math.PI));
            Logger.recordOutput(m_name + "/Velocity", io.getVelocity());
            Logger.recordOutput(m_name + "/CurrTrajPos", io.getCurrTrajPos());
            Logger.recordOutput(m_name + "/hasFinishedTrajectory", io.hasFinishedTrajectory(0.1));

            Logger.recordOutput(m_name + "/Appl Volt", inputs.appliedVoltage[0]);
            Logger.recordOutput(m_name + "/Supply Current", inputs.supplyCurrentAmps[0]);
        }
    }
}
