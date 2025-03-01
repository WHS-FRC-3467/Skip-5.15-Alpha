package frc.robot.subsystems.Climber;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        HOME(new ProfileType.MM_POSITION(() -> 0)),
        PREP(new ProfileType.MM_POSITION(() -> -180)),
        CLIMB(new ProfileType.MM_POSITION(() -> 15)),
        ClIMB_MORE(new ProfileType.MM_POSITION(() -> 20)),
        HOMING(new ProfileType.OPEN_VOLTAGE(() -> 3));

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
        SmartDashboard.putData("Home Climber Command", getHomeCommand());
    }

    public Command setStateCommand(State state)
    {
        return this.runOnce(() -> this.state = state);
    }

    // Climbing Triggers
    public boolean climbRequested = false; // Whether or not a climb request is active
    private Trigger climbRequest = new Trigger(() -> climbRequested); // Trigger for climb request
    public int climbStep = 0; // Tracking what step in the climb sequence we are on, is at zero when
                              // not climbing

    // Triggers for each step of the climb sequence
    private Trigger climbStep1 = new Trigger(() -> climbStep == 1);
    private Trigger climbStep2 = new Trigger(() -> climbStep == 2);
    private Trigger climbStep3 = new Trigger(() -> climbStep == 3);

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

    private Debouncer homedDebouncer = new Debouncer(0.1, DebounceType.kRising);
    private Debouncer stateDebouncer = new Debouncer(2, DebounceType.kRising);

    private Trigger homedTrigger =
        new Trigger(() -> homedDebouncer
            .calculate(Math.abs(io.getSupplyCurrent()) > 3.5)
            && stateDebouncer.calculate(this.state == State.HOMING));

    private Command getHomeCommand()
    {
        return this.setStateCommand(State.HOMING)
            .andThen(Commands.waitUntil(homedTrigger))
            .andThen(Commands.runOnce(() -> io.zeroSensors()))
            .andThen(this.setStateCommand(State.HOME));
    }
}
