package frc.robot.subsystems.Climber;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem.TargetState;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class Climber extends GenericMotionProfiledSubsystem<Climber.State> {

    static LoggedTunableNumber positionTuning =
        new LoggedTunableNumber("Climber/PositionTuningSP", 0.0);

    @RequiredArgsConstructor
    @Getter
    public enum State implements TargetState {
        // HOME is climber upright, Prep - Assuming that PREP position is parallel to the x axis,
        // CLIMB is inwards
        HOME(new ProfileType.MM_POSITION(() -> Units.degreesToRotations(90.0))),
        PREP(new ProfileType.MM_POSITION(() -> Units.degreesToRotations(0.0))),
        CLIMB(new ProfileType.MM_POSITION(() -> Units.degreesToRotations(110.0))),
        TUNING(new ProfileType.MM_POSITION(
            () -> Units.degreesToRotations(positionTuning.getAsDouble())));

        private final ProfileType profileType;
    }

    @Getter
    @Setter
    private State state = State.HOME;

    @Getter
    public final Alert climbedAlert = new Alert("CLIMB COMPLETE", Alert.AlertType.kInfo);

    /** Constructor */
    public Climber(ClimberIO io, boolean isSim)
    {
        super(State.HOME.profileType, ClimberConstants.kSubSysConstants, io, isSim);
    }

    public Command setStateCommand(State state)
    {
        return startEnd(() -> this.state = state, () -> this.state = State.HOME);
    }

    // Climbing Triggers
    public boolean climbRequested = false; // Whether or not a climb request is active
    private Trigger climbRequest = new Trigger(() -> climbRequested); // Trigger for climb request
    public int climbStep = 0; // Tracking what step in the climb sequence we are on, is at zero when
                              // not climbing

    // Triggers for each step of the climb sequence
    private Trigger climbStep1 = new Trigger(() -> climbStep == 1);
    private Trigger climbStep2 = new Trigger(() -> climbStep == 2);

    // Debouncer and trigger checks to see if the climber has finished climbing
    private Debouncer climbedDebouncer = new Debouncer(.25, DebounceType.kRising);

    public Trigger climbedTrigger =
        new Trigger(
            () -> climbedDebouncer.calculate(
                this.state == State.CLIMB
                    && (Math.abs(io.getSupplyCurrent()) > ClimberConstants.kSupplyCurrentLimit)));

    public Command climbedAlertCommand()
    {
        return new SequentialCommandGroup(
            new InstantCommand(() -> climbedAlert.set(true)),
            Commands.waitSeconds(1),
            new InstantCommand(() -> climbedAlert.set(false)));
    }

    public boolean atPosition(double tolerance)
    {
        return io.atPosition(state.profileType, tolerance);
    }

}
