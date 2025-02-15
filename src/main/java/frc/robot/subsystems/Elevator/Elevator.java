package frc.robot.subsystems.Elevator;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem.TargetState;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class Elevator extends GenericMotionProfiledSubsystem<Elevator.State> {

    @RequiredArgsConstructor
    @Getter
    public enum State implements TargetState {
        HOMING(() -> new LoggedTunableNumber("Elevator/HomingVoltageSP", -0.2).getAsDouble(), ProfileType.OPEN_VOLTAGE),
        // TODO: Test Voltage and position values (rotations)
        STOW(() -> new LoggedTunableNumber("Elevator/StowSP", 0.15).getAsDouble(), ProfileType.MM_POSITION),
        CORAL_INTAKE(() -> new LoggedTunableNumber("Elevator/CoralIntakeSP", 0.05).getAsDouble(), ProfileType.MM_POSITION),
        LEVEL_1(() -> new LoggedTunableNumber("Elevator/L1SP", 0.2).getAsDouble(), ProfileType.MM_POSITION),
        LEVEL_2(() -> new LoggedTunableNumber("Elevator/L2SP", 1.4).getAsDouble(), ProfileType.MM_POSITION),
        LEVEL_3(() -> new LoggedTunableNumber("Elevator/L3SP", 2.85).getAsDouble(), ProfileType.MM_POSITION),
        LEVEL_4(() -> new LoggedTunableNumber("Elevator/L4SP", 5.2).getAsDouble(), ProfileType.MM_POSITION),
        CLIMB(() -> new LoggedTunableNumber("Elevator/ClimbSP", 0.05).getAsDouble(), ProfileType.MM_POSITION),
        ALGAE_LOW(() -> new LoggedTunableNumber("Elevator/AlgaeLowSP", 0.8).getAsDouble(), ProfileType.MM_POSITION),
        ALGAE_HIGH(() -> new LoggedTunableNumber("Elevator/AlgaeHighSP", 1.5).getAsDouble(), ProfileType.MM_POSITION),
        ALGAE_GROUND(() -> new LoggedTunableNumber("Elevator/AlgaeGroundSP", 0.05).getAsDouble(), ProfileType.MM_POSITION),
        ALGAE_SCORE(() -> new LoggedTunableNumber("Elevator/AlgaeScoreSP", 0.05).getAsDouble(), ProfileType.MM_POSITION),
        BARGE(() -> new LoggedTunableNumber("Elevator/BargeSP", 4.0).getAsDouble(), ProfileType.MM_POSITION),
        CHARACTERIZATION(() -> 0.0, ProfileType.CHARACTERIZATION);

        private final DoubleSupplier output;
        private final ProfileType profileType;
    }

    @Getter
    @Setter
    private State state = State.STOW;

    @Getter
    public final Alert homedAlert = new Alert("NEW HOME SET", Alert.AlertType.kInfo);

    /* For adjusting the Arm's static characterization velocity threshold */
    private static final LoggedTunableNumber staticCharacterizationVelocityThresh =
        new LoggedTunableNumber("Elevator/StaticCharacterizationVelocityThresh", 0.1);

    /** Constructor */
    public Elevator(ElevatorIO io, boolean isSim)
    {
        super(ProfileType.MM_POSITION, ElevatorConstants.kSubSysConstants, io, isSim);
    }

    public Command setStateCommand(State state)
    {
        return runOnce(() -> this.state = state);
    }

    private Debouncer homedDebouncer = new Debouncer(.25, DebounceType.kRising);

    public Trigger homedTrigger =
        new Trigger(
            () -> homedDebouncer.calculate(
                (this.state == State.HOMING && Math.abs(io.getVelocity()) < .001)));

    public Command zeroSensorCommand()
    {
        return new InstantCommand(() -> io.zeroSensors());
    }

    public boolean atPosition(double tolerance)
    {
        return io.atPosition(tolerance);
    }

    public Command homedAlertCommand()
    {
        return new SequentialCommandGroup(
            new InstantCommand(() -> homedAlert.set(true)),
            Commands.waitSeconds(1),
            new InstantCommand(() -> homedAlert.set(false)));
    }

    public Command staticCharacterization(double outputRampRate)
    {
        final StaticCharacterizationState state = new StaticCharacterizationState();
        Timer timer = new Timer();
        return Commands.startRun(
            () -> {
                this.state = State.CHARACTERIZATION;
                timer.restart(); // Starts the timer that tracks the time of the characterization
            },
            () -> {
                state.characterizationOutput = outputRampRate * timer.get();
                io.runCurrent(state.characterizationOutput);
                Logger.recordOutput(
                    "Elevator/StaticCharacterizationOutput", state.characterizationOutput);
            })
            .until(() -> inputs.velocityRps * 2 * Math.PI >= staticCharacterizationVelocityThresh
                .get())
            .finallyDo(
                () -> {
                    timer.stop();
                    Logger.recordOutput("Elevator/CharacterizationOutput",
                        state.characterizationOutput);
                    this.state = State.STOW;
                });
    }

    private static class StaticCharacterizationState {
        public double characterizationOutput = 0.0;
    }
}
