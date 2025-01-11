package frc.robot.subsystems.GenericMotionProfiledSubsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public abstract class GenericMotionProfiledSubsystem<
        G extends GenericMotionProfiledSubsystem.TargetState>
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
  }

  public abstract G getState();

  private final String mName;
  private final ProfileType mProType;
  private final GenericMotionProfiledSubsystemConstants mConstants;
  private final GenericMotionProfiledSubsystemIO io;
  private boolean mIsSim = false;

  protected final GenericMotionProfiledIOInputsAutoLogged inputs =
      new GenericMotionProfiledIOInputsAutoLogged();
  private final Alert leaderMotorDisconnected;
  private final Alert followerMotorDisconnected;
  private final Alert CANcoderDisconnected;

  public GenericMotionProfiledSubsystem(
      ProfileType pType,
      GenericMotionProfiledSubsystemConstants constants,
      GenericMotionProfiledSubsystemIO io,
      boolean isSim) {

    this.mProType = pType;
    this.mConstants = constants;
    this.io = io;
    this.mIsSim = isSim;
    this.mName = mConstants.kName;

    this.leaderMotorDisconnected =
        new Alert(mName + " Leader motor disconnected!", Alert.AlertType.kWarning);
    this.followerMotorDisconnected =
        new Alert(mName + " Follower motor disconnected!", Alert.AlertType.kWarning);
    this.CANcoderDisconnected =
        new Alert(mName + " CANcoder disconnected!", Alert.AlertType.kWarning);

    // Make sure we use the correct profiling configs
    TalonFXConfiguration fxConfig = mIsSim ? mConstants.kSimMotorConfig : mConstants.kMotorConfig;

    // Tunable numbers for PID and motion gain constants
    kP = new LoggedTunableNumber(mName + "/Gains/kP", fxConfig.Slot0.kP);
    kI = new LoggedTunableNumber(mName + "/Gains/kI", fxConfig.Slot0.kI);
    kD = new LoggedTunableNumber(mName + "/Gains/kD", fxConfig.Slot0.kD);

    kG = new LoggedTunableNumber(mName + "/Gains/kG", fxConfig.Slot0.kG);
    kS = new LoggedTunableNumber(mName + "/Gains/kS", fxConfig.Slot0.kS);
    kV = new LoggedTunableNumber(mName + "/Gains/kV", fxConfig.Slot0.kV);
    kA = new LoggedTunableNumber(mName + "/Gains/kA", fxConfig.Slot0.kA);

    kCruiseVelocity =
        new LoggedTunableNumber(
            mName + "/CruiseVelocity", fxConfig.MotionMagic.MotionMagicCruiseVelocity);
    kAcceleration =
        new LoggedTunableNumber(
            mName + "/Acceleration", fxConfig.MotionMagic.MotionMagicAcceleration);
    kJerk = new LoggedTunableNumber(mName + "/Jerk", fxConfig.MotionMagic.MotionMagicJerk);

    io.configurePID(kP.get(), kI.get(), kD.get(), true);
    io.configureGSVA(kG.get(), kS.get(), kV.get(), kA.get(), true);
    io.configureMotion(kCruiseVelocity.get(), kAcceleration.get(), kJerk.get(), true);
  }

  public void periodic() {

    io.updateInputs(inputs);
    Logger.processInputs(mName, inputs);

    // Check for disconnections
    leaderMotorDisconnected.set(!inputs.leaderMotorConnected);
    followerMotorDisconnected.set(!inputs.followerMotorConnected);
    CANcoderDisconnected.set(mConstants.kCANcoder != null && !inputs.CANcoderConnected);

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
    switch (mProType) {
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
        /* Run Open Loop to velocity in rotations/second */
        io.runVoltage(getState().getOutput());
        break;
      case OPEN_CURRENT:
        /* Run Open Loop to current in amps */
        io.runCurrent(getState().getOutput());
        break;
    }

    displayInfo();
  }

  private void displayInfo() {

    Logger.recordOutput(mName + "/Goal State", getState().toString());

    if (Constants.tuningMode) {
      Logger.recordOutput(mName + "/Setpoint", io.getSetpoint());
      Logger.recordOutput(mName + "/Position", io.getPosition());
      Logger.recordOutput(mName + "/CurrTrajPos", io.getCurrTrajPos());

      Logger.recordOutput(mName + "/Appl Volt", inputs.appliedVoltage[0]);
      Logger.recordOutput(mName + "/Supply Current", inputs.supplyCurrentAmps[0]);
    }
  }
}
