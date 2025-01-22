package frc.robot.subsystems.ProfiledCoralRoller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem.TargetState;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class ProfiledCoralRoller extends GenericMotionProfiledSubsystem<ProfiledCoralRoller.State> {

  @RequiredArgsConstructor
  @Getter
  public enum State implements TargetState {
    OFF(0.0, 0.0, ProfileType.OPEN_VOLTAGE), INTAKE(6.0, 0.0, ProfileType.OPEN_VOLTAGE), EJECT(6.0,
        0.0, ProfileType.OPEN_VOLTAGE);

    private final double output;
    private final double feedFwd;
    private final ProfileType profileType;
  }

  @Getter
  @Setter
  private State state = State.OFF;

  private final boolean debug = true;

  /** Constructor */
  public ProfiledCoralRoller(ProfiledCoralRollerIO io, boolean isSim) {
    super(ProfileType.OPEN_VOLTAGE, ProfiledCoralRollerConstants.kSubSysConstants, io, isSim);
  }

  public Command setStateCommand(State state) {
    return startEnd(() -> this.state = state, () -> this.state = State.OFF);
  }
}
