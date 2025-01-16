package frc.robot.subsystems.SampleProfiledArm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem.TargetState;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class SampleProfiledArm extends GenericMotionProfiledSubsystem<SampleProfiledArm.State> {

  @RequiredArgsConstructor
  @Getter
  public enum State implements TargetState {
    HOME(0.0, 0.0, ProfileType.MM_POSITION),
    LEVEL_1(Units.degreesToRotations(90.0), 0.0, ProfileType.MM_POSITION),
    LEVEL_2(Units.degreesToRotations(135.0), 0.0, ProfileType.MM_POSITION),
    LEVEL_3(Units.degreesToRotations(200.0), 0.0, ProfileType.MM_POSITION);

    private final double output;
    private final double feedFwd;
    private final ProfileType profileType;
  }

  @Getter
  @Setter
  private State state = State.HOME;

  private final boolean debug = true;

  /** Constructor */
  public SampleProfiledArm(SampleProfiledArmIO io, boolean isSim) {
    super(ProfileType.MM_POSITION, SampleProfiledArmConstants.kSubSysConstants, io, isSim);
  }

  public Command setStateCommand(State state) {
    return startEnd(() -> this.state = state, () -> this.state = State.HOME);
  }
}
