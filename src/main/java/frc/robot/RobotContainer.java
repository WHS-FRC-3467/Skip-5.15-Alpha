// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SampleProfiledArm.SampleProfiledArm;
import frc.robot.subsystems.SampleProfiledArm.SampleProfiledArmIO;
import frc.robot.subsystems.SampleProfiledArm.SampleProfiledArmIOSim;
import frc.robot.subsystems.SampleProfiledArm.SampleProfiledArmIOTalonFX;
import frc.robot.subsystems.SampleProfiledElevator.SampleProfiledElevator;
import frc.robot.subsystems.SampleProfiledElevator.SampleProfiledElevatorIO;
import frc.robot.subsystems.SampleProfiledElevator.SampleProfiledElevatorIOSim;
import frc.robot.subsystems.SampleProfiledElevator.SampleProfiledElevatorIOTalonFX;
import frc.robot.subsystems.SampleProfiledRoller.SampleProfiledRoller;
import frc.robot.subsystems.SampleProfiledRoller.SampleProfiledRollerIO;
import frc.robot.subsystems.SampleProfiledRoller.SampleProfiledRollerIOSim;
import frc.robot.subsystems.SampleProfiledRoller.SampleProfiledRollerIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Controllers
    private final CommandXboxController m_driver = new CommandXboxController(0);
    // private final CommandXboxController m_operator = new
    // CommandXboxController(1);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> m_autoChooser;

    // AK-enabled Subsystems
    private final Drive m_drive;
    // private final SampleRollers m_sampleRollersSubsystem;
    private final SampleProfiledArm m_sampleArmSubsystem;
    private final SampleProfiledElevator m_sampleElevatorSubsystem;
    private final SampleProfiledRoller m_sampleProfiledRollerSubsystem;

    public final Vision m_vision;

    // Non-AK-enabled Subsystems
    // private final SimpleSubsystem m_simpleSubsystem = new SimpleSubsystem();
    // private final ComplexSubsystem m_complexSubsystem = new ComplexSubsystem();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                m_drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));

                // m_sampleRollersSubsystem = new SampleRollers(new SampleRollersIOTalonFX());
                m_sampleArmSubsystem = new SampleProfiledArm(new SampleProfiledArmIOTalonFX(), false);
                m_sampleElevatorSubsystem = new SampleProfiledElevator(new SampleProfiledElevatorIOTalonFX(), false);
                m_sampleProfiledRollerSubsystem = new SampleProfiledRoller(new SampleProfiledRollerIOTalonFX(), false);

                m_vision = new Vision(
                        m_drive::addVisionMeasurement,
                        new VisionIOPhotonVision(camera0Name, robotToCamera0));

                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                m_drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));

                // m_sampleRollersSubsystem = new SampleRollers(new SampleRollersIOSim());
                m_sampleArmSubsystem = new SampleProfiledArm(new SampleProfiledArmIOSim(), true);
                m_sampleElevatorSubsystem = new SampleProfiledElevator(new SampleProfiledElevatorIOSim(), true);
                m_sampleProfiledRollerSubsystem = new SampleProfiledRoller(new SampleProfiledRollerIOSim(), true);

                m_vision = new Vision(
                        m_drive::addVisionMeasurement,
                        new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, m_drive::getPose));
                break;

            default:
                // Replayed robot, disable IO implementations
                m_drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        },
                        new ModuleIO() {
                        });
                // m_sampleRollersSubsystem = new SampleRollers(new SampleRollersIO() {});
                m_sampleArmSubsystem = new SampleProfiledArm(new SampleProfiledArmIO() {
                }, true);
                m_sampleElevatorSubsystem = new SampleProfiledElevator(new SampleProfiledElevatorIO() {
                }, true);
                m_sampleProfiledRollerSubsystem = new SampleProfiledRoller(new SampleProfiledRollerIO() {
                }, false);

                m_vision = new Vision(m_drive::addVisionMeasurement, new VisionIO() {
                }, new VisionIO() {
                });

                break;
        }

        // Set up auto routines
        m_autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        m_autoChooser.addOption(
                "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(m_drive));
        m_autoChooser.addOption(
                "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(m_drive));
        m_autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        m_autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        m_autoChooser.addOption(
                "Drive SysId (Dynamic Forward)", m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        m_autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)", m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the controller button and joystick bindings
        configureControllerBindings();

        // Detect if controllers are missing / Stop multiple warnings
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    /** Use this method to define your joystick and button -> command mappings. */
    private void configureControllerBindings() {

        // Default command, normal field-relative drive
        m_drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        m_drive,
                        () -> -m_driver.getLeftY(),
                        () -> -m_driver.getLeftX(),
                        () -> -m_driver.getRightX()));

        // Driver A Button: Lock to 0°
        m_driver
                .a()
                .whileTrue(
                        DriveCommands.joystickDriveAtAngle(
                                m_drive,
                                () -> -m_driver.getLeftY(),
                                () -> -m_driver.getLeftX(),
                                () -> new Rotation2d()));

        m_driver
                .y()
                .whileTrue(
                        Commands.either(
                                DriveCommands.joystickDriveAtAngle(
                                        m_drive,
                                        () -> -m_driver.getLeftY(),
                                        () -> -m_driver.getLeftX(),
                                        () -> RobotState.getInstance().getAngleToTarget(m_drive.getPose())),
                                DriveCommands.joystickDrive(
                                        m_drive,
                                        () -> -m_driver.getLeftY(),
                                        () -> -m_driver.getLeftX(),
                                        () -> -m_driver.getRightX()),
                                () -> RobotState.getInstance().getTarget() != RobotState.TARGET.NONE));

        m_driver
                .leftBumper()
                .onTrue(
                        Commands.runOnce(
                                () -> RobotState.getInstance().setTarget(RobotState.getInstance().getNextTarget())));

        // Driver X Button: Switch wheel modules to X pattern
        m_driver.x().onTrue(Commands.runOnce(m_drive::stopWithX, m_drive));

        // Driver B Button: Reset gyro to 0°
        m_driver
                .b()
                .onTrue(
                        Commands.runOnce(
                                () -> m_drive.setPose(
                                        new Pose2d(m_drive.getPose().getTranslation(), new Rotation2d())),
                                m_drive)
                                .ignoringDisable(true));

        // // Driver X Button: Run the Sample Roller in Eject direction when held
        // m_driver.x().whileTrue(m_sampleRollersSubsystem.setStateCommand(SampleRollers.State.EJECT));
        // // Driver Y Button: Run the Sample Roller in Intake direction when held
        // m_driver.y().whileTrue(m_sampleRollersSubsystem.setStateCommand(SampleRollers.State.INTAKE));

        // Driver Left & Right Bumpers: Run the Sample Profiled Roller out and in when
        // held
        m_driver
                .leftBumper()
                .whileTrue(
                        m_sampleProfiledRollerSubsystem.setStateCommand(SampleProfiledRoller.State.EJECT));
        m_driver
                .rightBumper()
                .whileTrue(
                        m_sampleProfiledRollerSubsystem.setStateCommand(SampleProfiledRoller.State.INTAKE));

        // Driver Right Trigger: Run the Sample Profiled Roller to the requested
        // position
        m_driver
                .rightTrigger()
                .whileTrue(
                        m_sampleProfiledRollerSubsystem.setStateCommand(SampleProfiledRoller.State.POSITION));

        // Driver POV Down: Bring Arm and Elevator to Home position
        m_driver
                .povDown()
                .onTrue(
                        Commands.parallel(
                                m_sampleArmSubsystem.setStateCommand(SampleProfiledArm.State.HOME),
                                m_sampleElevatorSubsystem.setStateCommand(SampleProfiledElevator.State.HOME)));

        // Driver POV Left: Send Arm and Elevator to LEVEL_1
        m_driver
                .povLeft()
                .onTrue(
                        Commands.parallel(
                                m_sampleArmSubsystem.setStateCommand(SampleProfiledArm.State.LEVEL_1),
                                m_sampleElevatorSubsystem.setStateCommand(SampleProfiledElevator.State.LEVEL_1)));

        // Driver POV Up: Send Arm and Elevator to LEVEL_2
        m_driver
                .povUp()
                .onTrue(
                        Commands.parallel(
                                m_sampleArmSubsystem.setStateCommand(SampleProfiledArm.State.LEVEL_2),
                                m_sampleElevatorSubsystem.setStateCommand(SampleProfiledElevator.State.LEVEL_2)));

        // Driver POV Right: Send Arm and Elevator to LEVEL_3
        m_driver
                .povRight()
                .onTrue(
                        Commands.parallel(
                                m_sampleArmSubsystem.setStateCommand(SampleProfiledArm.State.LEVEL_3),
                                m_sampleElevatorSubsystem.setStateCommand(SampleProfiledElevator.State.LEVEL_3)));

        // Operator Buttons A & B run the Complex and Simple subsystems when held
        // m_operator.a().whileTrue(m_complexSubsystem.setStateCommand(ComplexSubsystem.State.SCORE));
        // m_operator.b().whileTrue(m_simpleSubsystem.setStateCommand(SimpleSubsystem.State.ON));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return m_autoChooser.get();
    }
}
