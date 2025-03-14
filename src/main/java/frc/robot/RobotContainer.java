// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.Vision.VisionConstants.*;
import java.util.function.Supplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.RobotType;
import frc.robot.FieldConstants.ReefSide;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Arm.*;
import frc.robot.subsystems.Claw.ClawRoller.ClawRoller;
import frc.robot.subsystems.Claw.ClawRoller.ClawRollerIO;
import frc.robot.subsystems.Claw.ClawRoller.ClawRollerIOSim;
import frc.robot.subsystems.Claw.ClawRoller.ClawRollerIOTalonFX;
import frc.robot.subsystems.Claw.ClawRoller.ClawRoller.State;
import frc.robot.subsystems.Claw.ClawRollerLaserCAN.ClawRollerLaserCAN;
import frc.robot.subsystems.Claw.ClawRollerLaserCAN.ClawRollerLaserCANIO;
import frc.robot.subsystems.Claw.ClawRollerLaserCAN.ClawRollerLaserCANIOReal;
import frc.robot.subsystems.Claw.ClawRollerLaserCAN.ClawRollerLaserCANIOSim;
import frc.robot.subsystems.Claw.IntakeLaserCAN.IntakeLaserCAN;
import frc.robot.subsystems.Claw.IntakeLaserCAN.IntakeLaserCANIO;
import frc.robot.subsystems.Claw.IntakeLaserCAN.IntakeLaserCANIOReal;
import frc.robot.subsystems.Claw.IntakeLaserCAN.IntakeLaserCANIOSim;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.ClimberIO;
import frc.robot.subsystems.Climber.ClimberIOSim;
import frc.robot.subsystems.Climber.ClimberIOTalonFX;
import frc.robot.subsystems.Elevator.*;
import frc.robot.subsystems.LED.LEDSubsystem;
import frc.robot.subsystems.LED.LEDSubsystemIO;
import frc.robot.subsystems.LED.LEDSubsystemIOCANdle;
import frc.robot.subsystems.LED.LEDSubsystemIOWPILib;
import frc.robot.subsystems.Vision.*;
import frc.robot.subsystems.drive.*;
import frc.robot.util.WindupXboxController;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Controllers
    private final WindupXboxController m_driver = new WindupXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> m_autoChooser;

    // AK-enabled Subsystems
    public final Drive m_drive;



    private double speedMultiplier = 0.9;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                m_drive =
                    new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));


                // break;

                // m_drive =
                // new Drive(
                // new GyroIO() {},
                // new ModuleIO() {},
                // new ModuleIO() {},
                // new ModuleIO() {},
                // new ModuleIO() {},
                // (robotPose) -> {
                // });



                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                m_drive =
                    new Drive(
                        new GyroIO() {},
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));


                break;

            default:
                // Replayed robot, disable IO implementations
                m_drive =
                    new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {});

                // Logic Triggers
                registerNamedCommands();

                // Set up auto routines
                m_autoChooser =
                    new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

                // Set up SysId routines
                m_autoChooser.addOption(
                    "Drive Wheel Radius Characterization",
                    DriveCommands.wheelRadiusCharacterization(m_drive));
                m_autoChooser.addOption(
                    "Drive Simple FF Characterization",
                    DriveCommands.feedforwardCharacterization(m_drive));
                m_autoChooser.addOption(
                    "Drive SysId (Quasistatic Forward)",
                    m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                m_autoChooser.addOption(
                    "Drive SysId (Quasistatic Reverse)",
                    m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
                m_autoChooser.addOption(
                    "Drive SysId (Dynamic Forward)",
                    m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
                m_autoChooser.addOption(
                    "Drive SysId (Dynamic Reverse)",
                    m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

                // Configure the controller button and joystick bindings
                configureControllerBindings();

                // Detect if controllers are missing / Stop multiple warnings
                DriverStation.silenceJoystickConnectionWarning(true);
        }
    }

    private Command joystickDrive()
    {
        return DriveCommands.joystickDrive(
            m_drive,
            () -> -m_driver.getLeftY() * speedMultiplier,
            () -> -m_driver.getLeftX() * speedMultiplier,
            () -> -m_driver.getRightX());
    }

    private Command joystickDriveAtAngle(Supplier<Rotation2d> angle)
    {
        return DriveCommands.joystickDriveAtAngle(
            m_drive,
            () -> -m_driver.getLeftY() * speedMultiplier,
            () -> -m_driver.getLeftX() * speedMultiplier,
            angle);
    }

    private Command joystickApproach(Supplier<Pose2d> approachPose)
    {
        return DriveCommands.joystickApproach(
            m_drive,
            () -> -m_driver.getLeftY() * speedMultiplier,
            approachPose);
    }


    /** Button and Command mappings */
    private void configureControllerBindings()
    {
        // Default command, normal field-relative drive
        m_drive.setDefaultCommand(joystickDrive());

        // Driver Right Bumper: Approach Nearest Right-Side Reef Branch
        m_driver.rightBumper()
            .whileTrue(
                joystickApproach(
                    () -> FieldConstants.getNearestReefBranch(m_drive.getPose(), ReefSide.RIGHT)));

        // Driver Left Bumper: Approach Nearest Left-Side Reef Branch
        m_driver.leftBumper()
            .whileTrue(
                joystickApproach(
                    () -> FieldConstants.getNearestReefBranch(m_drive.getPose(), ReefSide.LEFT)));

        // Driver Left Bumper and Algae mode: Approach Nearest Reef Face
        m_driver.rightBumper()
            .whileTrue(
                joystickApproach(() -> FieldConstants.getNearestReefFace(m_drive.getPose())));



        // Driver Left Trigger: Drivetrain drive at coral station angle, prepare the elevator and
        // arm, Get Ready to Intake Coral
        // m_driver
        // .leftTrigger().and(isCoralMode)
        // .whileTrue(
        // m_clawRoller.setStateCommand(ClawRoller.State.INTAKESLOW)
        // .andThen(
        // m_superStruct
        // .getTransitionCommand(Arm.State.CORAL_INTAKE,
        // Elevator.State.CORAL_INTAKE))
        // .andThen(
        // Commands.waitUntil(m_intakeLaserCAN.triggered
        // .and(m_clawRollerLaserCAN.triggered.negate())))
        // .andThen(
        // Commands.waitUntil(m_intakeLaserCAN.triggered.negate()
        // .and(m_clawRollerLaserCAN.triggered)))
        // .andThen(m_clawRoller.holdCoralCommand(m_clawRollerLaserCAN.triggered)));

        // Driver POV Down: Zero the Elevator (HOMING)
        m_driver.povDown().onTrue(m_profiledArm.setStateCommand(Arm.State.STOW)
            .andThen(m_profiledElevator.getHomeCommand()));

        m_driver.povUp().onTrue(
            m_profiledElevator.setStateCommand(Elevator.State.STOW));

        // Driver Right Bumper: Toggle between Coral and Algae Modes.
        // Make sure the Approach nearest reef face does not mess with this
        m_driver.start().and(m_driver.leftBumper().negate())
            .onTrue(setCoralAlgaeModeCommand()
                .andThen(m_superStruct.getTransitionCommand(Arm.State.STOW, Elevator.State.STOW))
                .andThen(m_clawRoller.setStateCommand(ClawRoller.State.OFF))
                .andThen(m_driver.rumbleForTime(1, 1)));

    }

    /**
     * Register Named commands for use in PathPlanner
     */
    private void registerNamedCommands()
    {
        // Go to the L1 Position
        NamedCommands.registerCommand(
            "L1",
            Commands.waitUntil(m_clawRollerLaserCAN.triggered).andThen(
                m_superStruct.getTransitionCommand(Arm.State.LEVEL_1, Elevator.State.LEVEL_1, 0.1,
                    0.8)));
        // Go to the L2 Position
        NamedCommands.registerCommand(
            "L2Prep",
            Commands.waitUntil(m_clawRollerLaserCAN.triggered).andThen(
                m_superStruct.getTransitionCommand(Arm.State.STOW, Elevator.State.LEVEL_2, 0.1,
                    0.8)));
        // Go to the L3 Position
        NamedCommands.registerCommand(
            "L3Prep",
            Commands.waitUntil(m_clawRollerLaserCAN.triggered).andThen(
                m_superStruct.getTransitionCommand(Arm.State.STOW, Elevator.State.LEVEL_3, 0.1,
                    0.8)));
        // Go to the L4 Position
        NamedCommands.registerCommand(
            "L4",
            Commands.waitUntil(m_clawRollerLaserCAN.triggered)
                .andThen(
                    m_superStruct.getTransitionCommand(Arm.State.LEVEL_4, Elevator.State.LEVEL_4,
                        0.001,
                        0.8)));

        NamedCommands.registerCommand(
            "L4Prep",
            Commands.waitUntil(m_clawRollerLaserCAN.triggered)
                .andThen(
                    m_superStruct.getTransitionCommand(Arm.State.STOW, Elevator.State.LEVEL_4,
                        0.001,
                        0.8)));

        // Go to the Home Position
        NamedCommands.registerCommand(
            "Home",
            m_superStruct.getTransitionCommand(Arm.State.STOW, Elevator.State.STOW, 0.1, 0.8));

        // Wait for intake laserCAN to be triggered
        NamedCommands.registerCommand("SuperstructureIntake",
            m_superStruct
                .getTransitionCommand(Arm.State.CORAL_INTAKE,
                    Elevator.State.CORAL_INTAKE, 0.1, 0.8)
                .andThen(m_clawRoller.setStateCommand(ClawRoller.State.INTAKE)));

        NamedCommands.registerCommand(
            "WaitForCoral",
            Commands.waitUntil(m_intakeLaserCAN.triggered));

        // Intake Coral
        NamedCommands.registerCommand(
            "IntakeCoral",
            m_clawRoller.setStateCommand(ClawRoller.State.INTAKE)
                .andThen(
                    Commands.waitUntil(
                        m_intakeLaserCAN.triggered.negate().and(m_clawRollerLaserCAN.triggered)))
                .andThen(m_clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL)));


        NamedCommands.registerCommand(
            "Score",
            m_clawRoller.setStateCommand(ClawRoller.State.SCORE)
                .andThen(Commands.waitUntil(m_clawRollerLaserCAN.triggered.negate()))
                .andThen(m_clawRoller.setStateCommand(ClawRoller.State.OFF)));

        NamedCommands.registerCommand("Coast", m_drive.run(() -> {
        }));

        NamedCommands.registerCommand("HoldCoral",
            m_clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL));

        NamedCommands.registerCommand("WaitForEnd", Commands.waitSeconds(14.7));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return m_autoChooser.get();
    }
}
