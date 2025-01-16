package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem.TargetState;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class Elevator extends GenericMotionProfiledSubsystem<Elevator.State> {

  @RequiredArgsConstructor
  @Getter
  public enum State implements TargetState {
    HOME(0.0, 0.0),
    LEVEL_1(0.2, 0.0),
    LEVEL_2(0.6, 0.0),
    LEVEL_3(1.0, 0.0),
    LEVEL_4(1.4, 0.0),
    CORAL_STATION(0.3, 0.0),
    ALGAE_LOWER(0.5, 0.0),
    ALGAE_UPPER(0.7, 0.0),
    NET(0.9, 0.0),
    ;

    private final double output;
    private final double feedFwd;
  }

  @Getter @Setter private State state = State.HOME;

  /** Constructor */
  public Elevator(ElevatorIO io, boolean isSim) {
    super(ProfileType.MM_POSITION, ElevatorConstants.kSubSysConstants, io, isSim);
  }

  public Command setStateCommand(State state) {
    return startEnd(() -> this.state = state, () -> this.state = State.HOME);
  }
}
