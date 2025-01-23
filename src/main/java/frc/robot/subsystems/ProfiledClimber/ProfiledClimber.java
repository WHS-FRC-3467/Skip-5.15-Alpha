package frc.robot.subsystems.ProfiledClimber;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem.TargetState;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class ProfiledClimber extends GenericMotionProfiledSubsystem<ProfiledClimber.State> {

  @RequiredArgsConstructor
  @Getter
  public enum State implements TargetState {
    HOME(Units.degreesToRadians(90), 0.0, ProfileType.MM_POSITION),
    CLIMB(
        Units.degreesToRotations(0.0),
        0.0,
        ProfileType.MM_POSITION); // Make the horizontal position the zero point for gravity feedfwd
    // LEVEL_2(Units.degreesToRotations(135.0), 0.0, ProfileType.MM_POSITION),
    // LEVEL_3(Units.degreesToRotations(200.0), 0.0, ProfileType.MM_POSITION);

    private final double output;
    private final double feedFwd;
    private final ProfileType profileType;
  }

  @Getter @Setter private State state = State.HOME;

  private final boolean debug = true;

  private double torqueCurrent = io.getTorqueCurrent();
  private BooleanSupplier isTorqueCurrentLimitBreached =
      () -> (torqueCurrent > ProfiledClimberConstants.kTorqueCurrentLimit);

  /** Constructor */
  public ProfiledClimber(ProfiledClimberIO io, boolean isSim) {
    super(ProfileType.MM_POSITION, ProfiledClimberConstants.kSubSysConstants, io, isSim);
  }

  public Command setStateCommand(State state) {
    return startEnd(() -> this.state = state, () -> this.state = State.HOME);
  }

  // TODO: Code Review this Command that stops climber if current exceeds a certain threshold
  public Command setStateUntilCurrentLimit(State state) {
    return run(() -> this.state = state)
        .until(isTorqueCurrentLimitBreached)
        .andThen(run(() -> this.state = State.HOME));
  }
}
