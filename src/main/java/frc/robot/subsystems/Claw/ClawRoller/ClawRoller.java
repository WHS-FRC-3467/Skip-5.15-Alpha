package frc.robot.subsystems.Claw.ClawRoller;

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
public class ClawRoller
    extends GenericMotionProfiledSubsystem<ClawRoller.State> {

    public final Trigger stalled = new Trigger(() -> super.inputs.torqueCurrentAmps[0] <= -60);

    static LoggedTunableNumber intakeSpeed =
        new LoggedTunableNumber("ClawRoller/IntakeDutyCycle", 0.12);
    static LoggedTunableNumber holdPosition =
        new LoggedTunableNumber("ClawRoller/holdPosition", 0.00);

    @RequiredArgsConstructor
    @Getter
    public enum State implements TargetState {
        OFF(new ProfileType.OPEN_VOLTAGE(() -> 0.0)),
        INTAKE(new ProfileType.OPEN_CURRENT(() -> 80.0, intakeSpeed)),
        EJECT(new ProfileType.OPEN_VOLTAGE(() -> 10.0)),
        SCORE(new ProfileType.OPEN_VOLTAGE(() -> 4.0)),
        SCORE_L1(new ProfileType.OPEN_VOLTAGE(() -> 1.5)),
        HOLDCORAL(new ProfileType.POSITION(holdPosition)),
        ALGAE_INTAKE(new ProfileType.OPEN_CURRENT(() -> -90, () -> 0.6));

        private final ProfileType profileType;
    }

    @Getter
    private State state = State.OFF;

    /** Constructor */
    public ClawRoller(ClawRollerIO io, boolean isSim)
    {
        super(State.OFF.profileType, ClawRollerConstants.kSubSysConstants, io, isSim);
    }

    public Command setStateCommand(State state)
    {
        return runOnce(() -> this.state = state);
    }

    public boolean atPosition(double tolerance)
    {
        return io.atPosition(state.profileType, tolerance);
    }
}
