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
        LEVEL_1(Units.degreesToRotations(94.13), Units.degreesToRotations(94.13)),
        LEVEL_2(Units.degreesToRotations(94.48), Units.degreesToRotations(94.48)),
        LEVEL_3(Units.degreesToRotations(94.48), Units.degreesToRotations(94.48)),
        LEVEL_4(Units.degreesToRotations(101.33), Units.degreesToRotations(101.33)),
        CLIMB(Units.degreesToRotations(50.4), Units.degreesToRotations(50.4)),
        ALGAE_LOW(.2377, .2377),
        ALGAE_HIGH(.2446, .2446),
        ALGAE_GROUND(Units.degreesToRotations(70.0), Units.degreesToRotations(70.0)),
        ALGAE_SCORE(Units.degreesToRotations(120.0), Units.degreesToRotations(120.0)),
        BARGE(Units.degreesToRotations(140.0), Units.degreesToRotations(140.0));

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
        STOW(new ProfileType.MM_POSITION(() -> Setpoints.STOW.getSetpoint())),
        CORAL_INTAKE(new ProfileType.MM_POSITION(() -> Setpoints.CORAL_INTAKE.getSetpoint())),
        LEVEL_1(new ProfileType.MM_POSITION(() -> Setpoints.LEVEL_1.getSetpoint())),
        LEVEL_2(new ProfileType.MM_POSITION(() -> Setpoints.LEVEL_2.getSetpoint())),
        LEVEL_3(new ProfileType.MM_POSITION(() -> Setpoints.LEVEL_3.getSetpoint())),
        LEVEL_4(new ProfileType.MM_POSITION(() -> Setpoints.LEVEL_4.getSetpoint())),
        CLIMB(new ProfileType.MM_POSITION(() -> Setpoints.CLIMB.getSetpoint())),
        ALGAE_LOW(new ProfileType.MM_POSITION(() -> Setpoints.ALGAE_LOW.getSetpoint())),
        ALGAE_HIGH(new ProfileType.MM_POSITION(() -> Setpoints.ALGAE_HIGH.getSetpoint())),
        ALGAE_GROUND(new ProfileType.MM_POSITION(() -> Setpoints.ALGAE_GROUND.getSetpoint())),
        ALGAE_SCORE(new ProfileType.MM_POSITION(() -> Setpoints.ALGAE_SCORE.getSetpoint())),
        BARGE(new ProfileType.MM_POSITION(() -> Setpoints.BARGE.getSetpoint())),
        // CHARACTERIZATION(new ProfileType.CHARACTERIZATION()),
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
