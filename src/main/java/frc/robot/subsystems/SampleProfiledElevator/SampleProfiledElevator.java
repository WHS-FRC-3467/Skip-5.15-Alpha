package frc.robot.subsystems.SampleProfiledElevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem.TargetState;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class SampleProfiledElevator
    extends GenericMotionProfiledSubsystem<SampleProfiledElevator.State> {

  @RequiredArgsConstructor
  @Getter
  public enum State implements TargetState {
    HOME(0.0, 0.0, ProfileType.MM_POSITION),
    LEVEL_1(0.2, 0.0, ProfileType.MM_POSITION),
    LEVEL_2(0.6, 0.0, ProfileType.MM_POSITION),
    LEVEL_3(1.0, 0.0, ProfileType.MM_POSITION);

    private final double output;
    private final double feedFwd;
    private final ProfileType profileType;
  }

  @Getter
  @Setter
  private State state = State.HOME;

  /** Constructor */
  public SampleProfiledElevator(SampleProfiledElevatorIO io, boolean isSim) {
    super(ProfileType.MM_POSITION, SampleProfiledElevatorConstants.kSubSysConstants, io, isSim);
  }

  public Command setStateCommand(State state) {
    return startEnd(() -> this.state = state, () -> this.state = State.HOME);
  }
}
