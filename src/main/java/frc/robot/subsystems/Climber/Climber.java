package frc.robot.subsystems.Climber;

import java.util.function.BooleanSupplier;
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
import frc.robot.util.Util;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class Climber extends GenericMotionProfiledSubsystem<Climber.State> {

    @RequiredArgsConstructor
    @Getter
    public enum State implements TargetState {
        // HOME is climber upright, Prep - Assuming that PREP position is parallel to the x axis, CLIMB is inwards
        HOME(Units.degreesToRotations(90), 0.0, ProfileType.MM_POSITION),
        PREP(Units.degreesToRotations(0.0), 0.0, ProfileType.MM_POSITION),
        CLIMB(Units.degreesToRotations(110.0), 0.0, ProfileType.MM_POSITION),;

        private final double output;
        private final double feedFwd;
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
        super(ProfileType.MM_POSITION, ClimberConstants.kSubSysConstants, io, isSim);
    }

    public Command setStateCommand(State state)
    {
        return startEnd(() -> this.state = state, () -> this.state = State.HOME);
    }

    private Debouncer climbedDebouncer = new Debouncer(.25, DebounceType.kRising);

    public Trigger climbedTrigger =
        new Trigger(
            () -> climbedDebouncer.calculate(
                this.state == State.CLIMB && (Math.abs(io.getSupplyCurrent()) > ClimberConstants.kSupplyCurrentLimit)));

    public Command climbedAlertCommand()
    {
        return new SequentialCommandGroup(
            new InstantCommand(() -> climbedAlert.set(true)),
            Commands.waitSeconds(1),
            new InstantCommand(() -> climbedAlert.set(false)));
    }
}
