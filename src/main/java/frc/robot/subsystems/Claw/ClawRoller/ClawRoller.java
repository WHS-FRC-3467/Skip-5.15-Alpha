package frc.robot.subsystems.Claw.ClawRoller;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem.TargetState;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.Util;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class ClawRoller
    extends GenericMotionProfiledSubsystem<ClawRoller.State> {

    public final Trigger stalled = new Trigger(() -> super.inputs.torqueCurrentAmps[0] >= 30);

    static LoggedTunableNumber holdCoralSP = new LoggedTunableNumber("ClawRoller/HoldCoralSP", 0.0);
    static LoggedTunableNumber algaeIntakeSP =
        new LoggedTunableNumber("ClawRoller/AlgaeIntakeSP", -15.0);

    @RequiredArgsConstructor
    @Getter
    public enum State implements TargetState {
        OFF(() -> 0.0, ProfileType.OPEN_VOLTAGE),
        INTAKE(() -> 2.0, ProfileType.OPEN_VOLTAGE),
        INTAKESLOW(() -> 1.5, ProfileType.OPEN_VOLTAGE),
        SHUFFLE(() -> holdCoralSP.get(), ProfileType.OPEN_VOLTAGE),
        EJECT(() -> 10.0, ProfileType.OPEN_VOLTAGE),
        SCORE(() -> 8.0, ProfileType.OPEN_VOLTAGE),
        HOLDCORAL(() -> 0.5, ProfileType.MM_POSITION),
        ALGAE_INTAKE(() -> algaeIntakeSP.getAsDouble(), ProfileType.OPEN_CURRENT);

        private final DoubleSupplier output;
        private final ProfileType profileType;
    }

    @Getter
    private State state = State.OFF;

    /** Constructor */
    public ClawRoller(ClawRollerIO io, boolean isSim)
    {
        super(ProfileType.OPEN_VOLTAGE, ClawRollerConstants.kSubSysConstants, io, isSim);
    }

    public Command setStateCommand(State state)
    {
        return runOnce(() -> this.state = state);
    }

    public boolean atPosition(double tolerance)
    {
        return Util.epsilonEquals(io.getPosition(), state.output.getAsDouble(),
            Math.max(0.0001, tolerance));
    }
}
