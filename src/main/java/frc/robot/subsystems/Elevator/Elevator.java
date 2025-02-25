package frc.robot.subsystems.Elevator;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    static LoggedTunableNumber homingTuning =
        new LoggedTunableNumber("Elevator/HomingVoltageSP", -0.2);
    static LoggedTunableNumber positionTuning =
        new LoggedTunableNumber("Elevator/PositionTuningSP", 0.05);

    @RequiredArgsConstructor
    @Getter
    public enum State implements TargetState {
        HOMING(() -> homingTuning.getAsDouble(), ProfileType.OPEN_VOLTAGE),
        STOW(() -> 0, ProfileType.MM_POSITION),
        CORAL_INTAKE(() -> 0.06, ProfileType.MM_POSITION),
        LEVEL_1(() -> 0.913, ProfileType.MM_POSITION),
        LEVEL_2(() -> 1.1281, ProfileType.MM_POSITION),
        LEVEL_3(() -> 2.558, ProfileType.MM_POSITION),
        LEVEL_4(() -> 5.1, ProfileType.MM_POSITION),
        CLIMB(() -> 0.05, ProfileType.MM_POSITION),
        ALGAE_LOW(() -> 1.903, ProfileType.MM_POSITION),
        ALGAE_HIGH(() -> 3.406, ProfileType.MM_POSITION),
        ALGAE_GROUND(() -> 0.05, ProfileType.MM_POSITION),
        ALGAE_SCORE(() -> 0.05, ProfileType.MM_POSITION),
        BARGE(() -> 5.3, ProfileType.MM_POSITION),
        TUNING(() -> positionTuning.getAsDouble(), ProfileType.MM_POSITION),
        CHARACTERIZATION(() -> 0.0, ProfileType.CHARACTERIZATION),
        COAST(() -> 0.0, ProfileType.DISABLED_COAST),
        BRAKE(() -> 0.0, ProfileType.DISABLED_BRAKE);

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
        SmartDashboard.putData("Elevator Coast Command", setCoastStateCommand());
        SmartDashboard.putData("Elevator Brake Command", setBrakeStateCommand());
    }

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

    private Debouncer homedDebouncer = new Debouncer(.01, DebounceType.kRising);

    public Trigger homedTrigger =
        new Trigger(
            () -> homedDebouncer.calculate(
                (this.state == State.HOMING && Math.abs(io.getVelocity()) < .01)));

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
