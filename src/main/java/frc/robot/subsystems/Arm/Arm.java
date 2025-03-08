package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    static LoggedTunableNumber positionTuning =
        new LoggedTunableNumber("Arm/PositionTuningSP", 124.0);

    // .14 rot is the max extension

    @RequiredArgsConstructor
    @Getter
    public enum State implements TargetState {
        // HOMING(0.0, 0.0, ProfileType.MM_POSITION),
        STOW(new ProfileType.MM_POSITION(() -> Units.degreesToRotations(125.18))),
        // CORAL_INTAKE(() -> 0.42, ProfileType.MM_POSITION),
        CORAL_INTAKE(new ProfileType.MM_POSITION(() -> Units.degreesToRotations(137.7))),
        LEVEL_1(new ProfileType.MM_POSITION(() -> Units.degreesToRotations(94.13))),
        LEVEL_2(new ProfileType.MM_POSITION(() -> Units.degreesToRotations(94.48))),
        LEVEL_3(new ProfileType.MM_POSITION(() -> Units.degreesToRotations(94.48))),
        LEVEL_4(new ProfileType.MM_POSITION(() -> Units.degreesToRotations(101.33))),
        CLIMB(new ProfileType.MM_POSITION(() -> Units.degreesToRotations(50.4))),
        ALGAE_LOW(new ProfileType.MM_POSITION(() -> .2377)),
        ALGAE_HIGH(new ProfileType.MM_POSITION(() -> .2446)),
        ALGAE_GROUND(new ProfileType.MM_POSITION(() -> Units.degreesToRotations(70.0))),
        ALGAE_SCORE(new ProfileType.MM_POSITION(() -> Units.degreesToRotations(120.0))),
        BARGE(new ProfileType.MM_POSITION(() -> Units.degreesToRotations(140.0))),
        TUNING(new ProfileType.MM_POSITION(
            () -> Units.degreesToRotations(positionTuning.getAsDouble()))),
        CHARACTERIZATION(new ProfileType.CHARACTERIZATION()),
        COAST(new ProfileType.DISABLED_COAST()),
        BRAKE(new ProfileType.DISABLED_BRAKE());

        private final DoubleSupplier output;
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
        super(State.STOW.profileType, ArmConstants.kSubSysConstants, io, isSim);
        SmartDashboard.putData("Arm Coast Command", setCoastStateCommand());
        SmartDashboard.putData("Arm Brake Command", setBrakeStateCommand());
    }

    /** Constructor */
    public Command setStateCommand(State state)
    {
        return this.runOnce(() -> this.state = state);
    }

    public Command setCoastStateCommand()
    {
        return this.runOnce(() -> this.state = State.COAST);
    }

    public Command setBrakeStateCommand()
    {
        return this.runOnce(() -> this.state = State.BRAKE);
    }

    public boolean atPosition(double tolerance)
    {
        return io.atPosition(state.profileType, tolerance);
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
                io.runCurrent(characterizationState.characterizationOutput, 1);
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
