package frc.robot.subsystems.ProfiledElevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem.TargetState;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class ProfiledElevator extends GenericMotionProfiledSubsystem<ProfiledElevator.State> {

  @RequiredArgsConstructor
  @Getter
  public enum State implements TargetState {
    HOMING(0.0, 0.0, ProfileType.OPEN_VOLTAGE),
    HOME(0.0, 0.0, ProfileType.MM_POSITION),
    LEVEL_1(0.2, 0.0, ProfileType.MM_POSITION),
    LEVEL_2(0.6, 0.0, ProfileType.MM_POSITION),
    LEVEL_3(1.0, 0.0, ProfileType.MM_POSITION),
    LEVEL_4(1.4, 0.0, ProfileType.MM_POSITION),
    CORAL_STATION(0.3, 0.0, ProfileType.MM_POSITION),
    ALGAE_LOWER(0.5, 0.0, ProfileType.MM_POSITION),
    ALGAE_UPPER(0.7, 0.0, ProfileType.MM_POSITION),
    NET(0.9, 0.0, ProfileType.MM_POSITION);


    private final double output;
    private final double feedFwd;
    private final ProfileType profileType;

  }

  @Getter @Setter private State state = State.HOME;

  /** Constructor */
  public ProfiledElevator(ElevatorIO io, boolean isSim) {
    super(ProfileType.MM_POSITION, ElevatorConstants.kSubSysConstants, io, isSim);
  }

  public Command setStateCommand(State state) {
    return startEnd(() -> this.state = state, () -> this.state = State.HOME);
  }

  // TODO: Create command that calls zeroSensor()?
}
