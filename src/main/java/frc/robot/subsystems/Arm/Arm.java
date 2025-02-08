package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem.TargetState;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class Arm extends GenericMotionProfiledSubsystem<Arm.State> {

    @RequiredArgsConstructor
    @Getter
    public enum State implements TargetState {
        // HOMING(0.0, 0.0, ProfileType.MM_POSITION),
        STOW(Units.degreesToRotations(20.0), 0.0, ProfileType.MM_POSITION),
        CORAL_INTAKE(Units.degreesToRotations(0.0), 0.0, ProfileType.MM_POSITION),
        LEVEL_1(Units.degreesToRotations(20.0), 0.0, ProfileType.MM_POSITION),
        LEVEL_2(Units.degreesToRotations(30.0), 0.0, ProfileType.MM_POSITION),
        LEVEL_3(Units.degreesToRotations(30.0), 0.0, ProfileType.MM_POSITION),
        LEVEL_4(Units.degreesToRotations(60.0), 0.0, ProfileType.MM_POSITION),
        CLIMB(Units.degreesToRotations(95.0), 0.0, ProfileType.MM_POSITION),
        ALGAE_LOW(Units.degreesToRotations(30.0), 0.0, ProfileType.MM_POSITION),
        ALGAE_HIGH(Units.degreesToRotations(30.0), 0.0, ProfileType.MM_POSITION),
        ALGAE_GROUND(Units.degreesToRotations(105.0), 0.0, ProfileType.MM_POSITION),
        ALGAE_SCORE(Units.degreesToRotations(30.0), 0.0, ProfileType.MM_POSITION),
        BARGE(Units.degreesToRotations(30.0), 0.0, ProfileType.MM_POSITION),
        CHARACTERIZATION(0.0, 0.0, ProfileType.CHARACTERIZATION);

        private final double output;
        private final double feedFwd;

        private final ProfileType profileType;
    }

    @Getter
    @Setter
    private State state = State.STOW;

    private final boolean debug = true;

    /* For adjusting the Arm's static characterization velocity threshold */
    private static final LoggedTunableNumber staticCharacterizationVelocityThresh =
        new LoggedTunableNumber("Arm/StaticCharacterizationVelocityThresh", 0.1);

    public Arm(ArmIO io, boolean isSim)
    {
        super(ProfileType.MM_POSITION, ArmConstants.kSubSysConstants, io, isSim);
    }

    /** Constructor */
    public Command setStateCommand(State state)
    {
        return startEnd(() -> this.state = state, () -> this.state = State.STOW);
    }

    public boolean atPosition(double tolerance)
    {
        return io.atPosition(tolerance);
    }

    public Command staticCharacterization(double outputRampRate)
    {
        final StaticCharacterizationState characterizationState = new StaticCharacterizationState();
        Timer timer = new Timer();
        return Commands.startRun(
            () -> {
                this.state = State.CHARACTERIZATION;
                timer.restart(); // Starts the timer that tracks the time of the characterization
            },
            () -> {
                characterizationState.characterizationOutput = outputRampRate * timer.get();
                io.runCurrent(characterizationState.characterizationOutput);
                Logger.recordOutput(
                    "Arm/StaticCharacterizationOutput",
                    characterizationState.characterizationOutput);
            })
            .until(() -> inputs.velocityRps * 2 * Math.PI >= staticCharacterizationVelocityThresh
                .get())
            .finallyDo(
                () -> {
                    timer.stop();
                    Logger.recordOutput("Arm/CharacterizationOutput",
                        characterizationState.characterizationOutput);
                    this.state = State.STOW;
                });
    }

    private static class StaticCharacterizationState {
        public double characterizationOutput = 0.0;
    }
}
