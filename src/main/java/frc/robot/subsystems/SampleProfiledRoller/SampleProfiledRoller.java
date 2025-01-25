package frc.robot.subsystems.SampleProfiledRoller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem.TargetState;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class SampleProfiledRoller
    extends GenericMotionProfiledSubsystem<SampleProfiledRoller.State> {

  @RequiredArgsConstructor
  @Getter
  public enum State implements TargetState {
    OFF(0.0, 0.0, ProfileType.OPEN_VOLTAGE),
    INTAKE(6.0, 0.0, ProfileType.OPEN_VOLTAGE),
    EJECT(-6.0, 0.0, ProfileType.OPEN_VOLTAGE),
    POSITION(90.0, 0.0, ProfileType.MM_POSITION);

    private final double output;
    private final double feedFwd;
    private final ProfileType profileType;
  }

  @Getter @Setter private State state = State.OFF;

  private final boolean debug = true;

  /** Constructor */
  public SampleProfiledRoller(SampleProfiledRollerIO io, boolean isSim) {
    super(ProfileType.OPEN_VOLTAGE, SampleProfiledRollerConstants.kSubSysConstants, io, isSim);
  }

  public Command setStateCommand(State state) {
    return startEnd(() -> this.state = state, () -> this.state = State.OFF);
  }
}
