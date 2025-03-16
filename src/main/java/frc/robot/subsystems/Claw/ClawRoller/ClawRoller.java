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

    public final Trigger algaeStalledTrigger =
        new Trigger(() -> super.inputs.torqueCurrentAmps[0] >= 69);

    public final Trigger coralStalledTrigger =
        new Trigger(() -> super.inputs.supplyCurrentAmps[0] > 10);

    public final Trigger coralStoppedTrigger =
        new Trigger(() -> Math.abs(super.inputs.velocityRps) < 0.2);


    static LoggedTunableNumber intakeSpeed =
        new LoggedTunableNumber("ClawRoller/IntakeDutyCycle", .4);

    @RequiredArgsConstructor
    @Getter
    public enum State implements TargetState {
        OFF(new ProfileType.DISABLED_BRAKE()),
        INTAKE(new ProfileType.OPEN_CURRENT(() -> 80,
            () -> .5)),
        SLOW_INTAKE(new ProfileType.OPEN_CURRENT(() -> 20,
            () -> .2)),
        GORT_INTAKE(new ProfileType.OPEN_CURRENT(() -> 80,
            () -> 0.06)),
        SCORE(new ProfileType.OPEN_VOLTAGE(() -> 4.0)),
        ALGAE_INTAKE(new ProfileType.OPEN_CURRENT(() -> 90, () -> 0.6)),
        ALGAE_SCORE(new ProfileType.OPEN_CURRENT(() -> -90, () -> 0.6));

        private final ProfileType profileType;
    }

    @Getter
    private State state = State.OFF;

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
