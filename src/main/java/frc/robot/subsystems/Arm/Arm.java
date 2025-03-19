package frc.robot.subsystems.Arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem.TargetState;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class Arm extends GenericMotionProfiledSubsystem<Arm.State> {

    @RequiredArgsConstructor
    public enum Setpoints {
        STOW(Units.degreesToRotations(125.18), Units.degreesToRotations(125.18)),
        CORAL_INTAKE(Units.degreesToRotations(137.7), Units.degreesToRotations(137.7)),
        // CORAL_CLEARANCE(Units.degreesToRotations(133.5), Units.degreesToRotations(133.5)),
        LEVEL_1(Units.degreesToRotations(94.13), Units.degreesToRotations(94.13)),
        LEVEL_2(Units.degreesToRotations(94.48), Units.degreesToRotations(94.48)),
        LEVEL_3(Units.degreesToRotations(104.48), Units.degreesToRotations(104.48)),
        LEVEL_4(Units.degreesToRotations(101.33), Units.degreesToRotations(101.33)),
        CLIMB(Units.degreesToRotations(82.4), Units.degreesToRotations(82.4)),
        ALGAE_LOW(Units.degreesToRotations(103.3), Units.degreesToRotations(103.3)),
        ALGAE_HIGH(Units.degreesToRotations(103.3), Units.degreesToRotations(103.3)),
        ALGAE_GROUND(Units.degreesToRotations(70.0), Units.degreesToRotations(70.0)),
        PROCESSOR_SCORE(Units.degreesToRotations(120.0), Units.degreesToRotations(120.0)),
        BARGE(Units.degreesToRotations(130.0), Units.degreesToRotations(130.0));

        private final double gortSetpoint;
        private final double bajaSetpoint;

        public double getSetpoint()
        {
            if (Constants.getRobot() == RobotType.GORT) {
                return gortSetpoint;
            } else {
                return bajaSetpoint;
            }
        }
    }

    @RequiredArgsConstructor
    @Getter
    public enum State implements TargetState {
        // HOMING(0.0, 0.0, ProfileType.MM_POSITION),
        STOW(new ProfileType.MM_POSITION(() -> Setpoints.STOW.getSetpoint(), 0)),
        // CORAL_INTAKE(() -> 0.42, ProfileType.MM_POSITION),
        CORAL_INTAKE(new ProfileType.MM_POSITION(() -> Setpoints.CORAL_INTAKE.getSetpoint(), 0)),
        LEVEL_1(new ProfileType.MM_POSITION(() -> Setpoints.LEVEL_1.getSetpoint(), 0)),
        LEVEL_2(new ProfileType.MM_POSITION(() -> Setpoints.LEVEL_2.getSetpoint(), 0)),
        LEVEL_3(new ProfileType.MM_POSITION(() -> Setpoints.LEVEL_3.getSetpoint(), 0)),
        LEVEL_4(new ProfileType.MM_POSITION(() -> Setpoints.LEVEL_4.getSetpoint(), 0)),
        CLIMB(new ProfileType.MM_POSITION(() -> Setpoints.CLIMB.getSetpoint(), 0)),
        ALGAE_LOW(new ProfileType.MM_POSITION(() -> Setpoints.ALGAE_LOW.getSetpoint(), 0)),
        ALGAE_HIGH(new ProfileType.MM_POSITION(() -> Setpoints.ALGAE_HIGH.getSetpoint(), 0)),
        ALGAE_GROUND(new ProfileType.MM_POSITION(() -> Setpoints.ALGAE_GROUND.getSetpoint(), 0)),
        PROCESSOR_SCORE(
            new ProfileType.MM_POSITION(() -> Setpoints.PROCESSOR_SCORE.getSetpoint(), 0)),
        BARGE(new ProfileType.MM_POSITION(() -> Setpoints.BARGE.getSetpoint(), 0)),
        CHARACTERIZATION(new ProfileType.CHARACTERIZATION()),
        COAST(new ProfileType.DISABLED_COAST()),
        BRAKE(new ProfileType.DISABLED_BRAKE());

        private final ProfileType profileType;
    }

    @Getter
    @Setter
    private State state = State.STOW;

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

}
