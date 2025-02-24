package frc.robot.subsystems.Claw.ClawRoller;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    public final Trigger stalled = new Trigger(() -> super.inputs.torqueCurrentAmps[0] <= -60);

    @RequiredArgsConstructor
    @Getter
    public enum State implements TargetState {
        OFF(() -> 0.0, ProfileType.OPEN_VOLTAGE),
        INTAKE(() -> 2.0, ProfileType.OPEN_VOLTAGE),
        INTAKESLOW(() -> 5, ProfileType.VELOCITY),
        EJECT(() -> 10.0, ProfileType.OPEN_VOLTAGE),
        SCORE(() -> 8.0, ProfileType.OPEN_VOLTAGE),
        SHUFFLE(() -> -1, ProfileType.VELOCITY),
        HOLDCORAL(() -> 0, ProfileType.DISABLED_BRAKE),
        ALGAE_INTAKE(() -> -100, ProfileType.OPEN_CURRENT);

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

    public Command holdCoralCommand(Trigger clawTriggered)
    {
        return this.setStateCommand(State.HOLDCORAL).andThen(
            this.run(() -> {
                clawTriggered.negate().whileTrue(
                    Commands.sequence(
                        this.setStateCommand(State.SHUFFLE),
                        Commands.waitSeconds(0.2),
                        this.setStateCommand(State.INTAKESLOW),
                        Commands.waitSeconds(0.2),
                        this.setStateCommand(State.OFF)))
                    .onFalse(
                        this.setStateCommand(State.HOLDCORAL));
            }));
    }
}
