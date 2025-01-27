package frc.robot.subsystems.Arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem.TargetState;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class Arm extends GenericMotionProfiledSubsystem<Arm.State> {

    @RequiredArgsConstructor
    @Getter
    public enum State implements TargetState {
        HOME(0.0, 0.0, ProfileType.MM_POSITION),
        HOMING(0.0, 0.0, ProfileType.MM_POSITION),
        LEVEL_1(Units.degreesToRotations(90.0), 0.0, ProfileType.MM_POSITION),
        LEVEL_2(Units.degreesToRotations(135.0), 0.0, ProfileType.MM_POSITION),
        LEVEL_3(Units.degreesToRotations(135.0), 0.0, ProfileType.MM_POSITION),
        LEVEL_4(Units.degreesToRotations(200.0), 0.0, ProfileType.MM_POSITION);

        private final double output;
        private final double feedFwd;

        private final ProfileType profileType;
    }

    @Getter
    @Setter
    private State state = State.HOME;

    private final boolean debug = true;

    public Arm(ArmIO io, boolean isSim)
    {
        super(ProfileType.MM_POSITION, ArmConstants.kSubSysConstants, io, isSim);
    }

    public Trigger homedTrigger =
        new Trigger(
            () -> (this.state == State.HOMING
                && io.getSupplyCurrent() > ArmConstants.kHomingCurrent));

    /** Constructor */
    public Command setStateCommand(State state)
    {
        return startEnd(() -> this.state = state, () -> this.state = State.HOME);
    }

    public Command zeroSensorCommand()
    {
        return new InstantCommand(() -> io.zeroSensors());
    }
}
