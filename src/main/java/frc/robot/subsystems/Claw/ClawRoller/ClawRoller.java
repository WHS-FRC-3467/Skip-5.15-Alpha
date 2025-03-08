package frc.robot.subsystems.Claw.ClawRoller;

import java.util.function.BooleanSupplier;
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

    static LoggedTunableNumber holdCoralSP = new LoggedTunableNumber("ClawRoller/HoldCoralSP", 0.0);
    static LoggedTunableNumber algaeIntakeSP =
       
        new LoggedTunableNumber("ClawRoller/AlgaeIntakeSP", -15.0);

    public static boolean isCurrentDipped;

    @RequiredArgsConstructor
    @Getter
    public enum State implements TargetState {
        OFF(new ProfileType.OPEN_VOLTAGE(() -> 0.0)),
        INTAKE(new ProfileType.OPEN_CURRENT(() -> 80.0, () -> 0.06)),
        EJECT(new ProfileType.OPEN_VOLTAGE(() -> 10.0)),
        SCORE(new ProfileType.OPEN_VOLTAGE(() -> 4.0)),
        SCORE_L1(new ProfileType.OPEN_VOLTAGE(() -> 1.5)),
        SHUFFLE(new ProfileType.VELOCITY(() -> -1)),
        HOLDCORAL(new ProfileType.DISABLED_BRAKE()),
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

    public BooleanSupplier isCurrentSpiked(double spikedCurrentThreshold)
    {
        return () -> spikedCurrentThreshold > io.getSupplyCurrent();
    }

    public BooleanSupplier isCurrentDipped(double dippedCurrentThreshold)
    {
        return () -> dippedCurrentThreshold < io.getSupplyCurrent();
    }


}
