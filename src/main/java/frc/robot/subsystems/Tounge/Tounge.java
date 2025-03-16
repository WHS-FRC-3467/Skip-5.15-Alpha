package frc.robot.subsystems.Tounge;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem.TargetState;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class Tounge extends GenericMotionProfiledSubsystem<Tounge.State> {

    static LoggedTunableNumber positionTuning =
        new LoggedTunableNumber("Tounge/PositionTuningSP", 10.0);

    static LoggedTunableNumber homingTuning =
        new LoggedTunableNumber("Tounge/HomingVoltageSP", -1);

    @RequiredArgsConstructor
    @Getter
    public enum State implements TargetState {
        HOMING(new ProfileType.OPEN_VOLTAGE(() -> homingTuning.getAsDouble())),
        STOW(new ProfileType.MM_POSITION(() -> Units.degreesToRotations(0), 0)),
        RAISED(new ProfileType.MM_POSITION(
            () -> Units.degreesToRotations(positionTuning.getAsDouble()), 0));

        private final ProfileType profileType;
    }

    @Getter
    @Setter
    private State state = State.STOW;

    public Tounge(ToungeIO io, boolean isSim)
    {
        super(State.STOW.profileType, ToungeConstants.kSubSysConstants, io, isSim);
    }

    public Command setStateCommand(State state)
    {
        return this.runOnce(() -> this.state = state);
    }

    public boolean atPosition(double tolerance)
    {
        return io.atPosition(state.profileType, tolerance);
    }

    private Debouncer homedDebouncer = new Debouncer(0.1, DebounceType.kRising);

    public Trigger homedTrigger = new Trigger(
        () -> homedDebouncer.calculate(
            (this.state == State.HOMING && Math.abs(io.getSupplyCurrent()) > 3)));

    public Trigger coralContactTrigger = new Trigger(
        () -> atPosition(Units.degreesToRotations(5)) && Math.abs(io.getSupplyCurrent()) > 3);

}
