package frc.robot.subsystems.Claw.ClawRoller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem.TargetState;
import frc.robot.util.Util;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class ClawRoller
    extends GenericMotionProfiledSubsystem<ClawRoller.State> {

    public final Trigger stalled = new Trigger(() -> super.inputs.supplyCurrentAmps[0] < 1); // TODO:
                                                                                             // real
                                                                                             // numbers

    @RequiredArgsConstructor
    @Getter
    public enum State implements TargetState {
        OFF(0.0, 0.0, ProfileType.OPEN_VOLTAGE), // TODO: tune on real robot
        INTAKE(2.0, 0.0, ProfileType.OPEN_VOLTAGE),
        EJECT(6.0, 0.0, ProfileType.OPEN_VOLTAGE),
        SCORE(8.0, 0.0, ProfileType.OPEN_VOLTAGE),
        HOLDCORAL(0.0, 0.0, ProfileType.MM_POSITION); // One rotation after detecting coral, switch
                                                      // to HOLDCORAL to tell the motors to go one
                                                      // rotaiton

        private final double output;
        private final double feedFwd;
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
        return startEnd(() -> this.state = state, () -> this.state = State.OFF);
    }

    public Command setStateCommandNoEnd(State state)
    {
        return runOnce(() -> this.state = state);
    }

    public boolean atPosition(double tolerance)
    {
        return Util.epsilonEquals(io.getPosition(), state.output, Math.max(0.0001, tolerance));
    }
}
