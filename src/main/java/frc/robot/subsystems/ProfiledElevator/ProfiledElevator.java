package frc.robot.subsystems.ProfiledElevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    HOMING(
        -0.2,
        0.0,
        ProfileType.OPEN_VOLTAGE), // TODO: Test Voltage and position values (rotations) (elevator)
    HOME(0.15, 0.0, ProfileType.MM_POSITION),
    LEVEL_1(0.5, 0.0, ProfileType.MM_POSITION),
    LEVEL_2(3.0, 0.0, ProfileType.MM_POSITION),
    LEVEL_3(7.0, 0.0, ProfileType.MM_POSITION),
    LEVEL_4(9.0, 0.0, ProfileType.MM_POSITION),
    CORAL_STATION(0.6, 0.0, ProfileType.MM_POSITION),
    ALGAE_LOWER(0.5, 0.0, ProfileType.MM_POSITION),
    ALGAE_UPPER(0.8, 0.0, ProfileType.MM_POSITION),
    NET(9.0, 0.0, ProfileType.MM_POSITION);

    private final double output;
    private final double feedFwd;
    private final ProfileType profileType;
  }

  @Getter @Setter private State state = State.HOME;

  // We are in the correct position for homing if we are moving down and the current is above a
  // certain threshold
  public Trigger homedTrigger =
      new Trigger(
          () ->
              (this.state == State.HOMING
                  && io.getSupplyCurrent() > ElevatorConstants.kHomingCurrent));

  /** Constructor */
  public ProfiledElevator(ElevatorIO io, boolean isSim) {
    super(ProfileType.MM_POSITION, ElevatorConstants.kSubSysConstants, io, isSim);
  }

  public Command setStateCommand(State state) {
    return startEnd(() -> this.state = state, () -> this.state = State.HOME);
  }

  // Command that calls zeroSensors(), part of the homing sequence
  public Command zeroSensorCommand() {
    return new InstantCommand(() -> io.zeroSensors());
  }
}
