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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.FieldConstants.ReefSide;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Arm.*;
import frc.robot.subsystems.Claw.ClawRoller.ClawRoller;
import frc.robot.subsystems.Claw.ClawRoller.ClawRollerIO;
import frc.robot.subsystems.Claw.ClawRoller.ClawRollerIOSim;
import frc.robot.subsystems.Claw.ClawRoller.ClawRollerIOTalonFX;
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
import frc.robot.subsystems.LED.LED;
import frc.robot.subsystems.Vision.*;
import frc.robot.subsystems.drive.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Controllers
    private final CommandXboxController m_driver = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> m_autoChooser;

    // Maple Sim
    private SwerveDriveSimulation m_driveSimulation = null;

    // AK-enabled Subsystems
    private final Drive m_drive;
    private final Arm m_profiledArm;
    private final Elevator m_profiledElevator;
    private final Climber m_profiledClimber;
    private final ClawRoller m_clawRoller;
    private final ClawRollerLaserCAN m_clawRollerLaserCAN;
    public final IntakeLaserCAN m_intakeLaserCAN;
    private final Superstructure m_superStruct;

    public final Vision m_vision;
    // public final LED m_LED;

    // Trigger for algae/coral mode switching
    private boolean coralModeEnabled = true;
    private Trigger isCoralMode = new Trigger(() -> coralModeEnabled);

    // private final LaserCANSensor m_clawLaserCAN =
    // new LaserCANSensor(Ports.CLAW_LASERCAN.getDeviceNumber(), Inches.of(6));
    // private final LaserCANSensor m_rampLaserCAN =
    // new LaserCANSensor(Ports.RAMP_LASERCAN.getDeviceNumber(), Inches.of(6));

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                // m_drive =
                // new Drive(
                // new GyroIOPigeon2(),
                // new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                // new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                // new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                // new ModuleIOTalonFXReal(TunerConstants.BackRight),
                // (robotPose) -> {
                // });

                // m_profiledArm = new Arm(new ArmIOTalonFX(), false);
                // m_profiledElevator = new Elevator(new ElevatorIOTalonFX(), false);
                // m_profiledClimber = new Climber(new ClimberIOTalonFX(), false);
                // m_clawRoller = new ClawRoller(new ClawRollerIOTalonFX(), false);
                m_clawRollerLaserCAN = new ClawRollerLaserCAN(new ClawRollerLaserCANIOReal());
                m_intakeLaserCAN = new IntakeLaserCAN(new IntakeLaserCANIOReal());

                // m_vision =
                // new Vision(
                // m_drive,
                // new VisionIOPhotonVision(camera0Name, robotToCamera0),
                // new VisionIOPhotonVision(camera1Name, robotToCamera1));

                // break;

                m_drive =
                    new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        (robotPose) -> {
                        });

                m_profiledArm = new Arm(new ArmIO() {}, true);
                m_profiledElevator = new Elevator(new ElevatorIO() {}, true);
                m_profiledClimber = new Climber(new ClimberIO() {}, true);
                m_clawRoller = new ClawRoller(new ClawRollerIO() {}, true);
                // m_clawRollerLaserCAN = new ClawRollerLaserCAN(new ClawRollerLaserCANIO() {});
                // m_intakeLaserCAN = new IntakeLaserCAN(new IntakeLaserCANIO() {});

                m_vision = new Vision(m_drive, new VisionIO() {}, new VisionIO() {});

                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                m_driveSimulation =
                    new SwerveDriveSimulation(Drive.mapleSimConfig,
                        new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(m_driveSimulation);
                m_drive =
                    new Drive(
                        new GyroIOSim(this.m_driveSimulation.getGyroSimulation()),
                        new ModuleIOTalonFXSim(
                            TunerConstants.FrontLeft, this.m_driveSimulation.getModules()[0]),
                        new ModuleIOTalonFXSim(
                            TunerConstants.FrontRight, this.m_driveSimulation.getModules()[1]),
                        new ModuleIOTalonFXSim(
                            TunerConstants.BackLeft, this.m_driveSimulation.getModules()[2]),
                        new ModuleIOTalonFXSim(
                            TunerConstants.BackRight, this.m_driveSimulation.getModules()[3]),
                        m_driveSimulation::setSimulationWorldPose);

                m_profiledArm = new Arm(new ArmIOSim(), true);
                m_profiledElevator = new Elevator(new ElevatorIOSim(), true);
                m_profiledClimber = new Climber(new ClimberIOSim(), true);
                m_clawRoller = new ClawRoller(new ClawRollerIOSim(), true);
                m_clawRollerLaserCAN = new ClawRollerLaserCAN(new ClawRollerLaserCANIOSim());
                m_intakeLaserCAN = new IntakeLaserCAN(new IntakeLaserCANIOSim());

                m_vision =
                    new Vision(
                        m_drive,
                        new VisionIOPhotonVisionSim(
                            camera0Name, robotToCamera0,
                            m_driveSimulation::getSimulatedDriveTrainPose),
                        new VisionIOPhotonVisionSim(
                            camera1Name, robotToCamera1,
                            m_driveSimulation::getSimulatedDriveTrainPose));

                break;

            default:
                // Replayed robot, disable IO implementations
                m_drive =
                    new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        (robotPose) -> {
                        });

                m_profiledArm = new Arm(new ArmIO() {}, true);
                m_profiledElevator = new Elevator(new ElevatorIO() {}, true);
                m_profiledClimber = new Climber(new ClimberIO() {}, true);
                m_clawRoller = new ClawRoller(new ClawRollerIO() {}, true);
                m_clawRollerLaserCAN = new ClawRollerLaserCAN(new ClawRollerLaserCANIO() {});
                m_intakeLaserCAN = new IntakeLaserCAN(new IntakeLaserCANIO() {});

                m_vision = new Vision(m_drive, new VisionIO() {}, new VisionIO() {});
                break;
        }

        // Superstructure coordinates Arm and Elevator motions
        m_superStruct = new Superstructure(m_profiledArm, m_profiledElevator);

        // LED subsystem reads status from all other subsystems to control LEDs via CANdle
        // m_LED = new LED(m_driver, m_profiledArm, m_clawRoller, m_profiledClimber, m_drive,
        // m_profiledElevator, m_vision, m_clawRollerLaserCAN, isCoralMode);

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
        m_autoChooser.addOption("Elevator static", m_profiledElevator.staticCharacterization(2.0));
        m_autoChooser.addOption("Arm static", m_profiledArm.staticCharacterization(2.0));

        // Configure the controller button and joystick bindings
        configureControllerBindings();

        // Detect if controllers are missing / Stop multiple warnings
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    private Command joystickDrive()
    {
        return DriveCommands.joystickDrive(
            m_drive,
            () -> -m_driver.getLeftY(),
            () -> -m_driver.getLeftX(),
            () -> -m_driver.getRightX());
    }

    private Command joystickDriveAtAngle(Supplier<Rotation2d> angle)
    {
        return DriveCommands.joystickDriveAtAngle(
            m_drive,
            () -> -m_driver.getLeftY(),
            () -> -m_driver.getLeftX(),
            angle);
    }

    private Command joystickApproach(Supplier<Pose2d> approachPose)
    {
        return DriveCommands.joystickApproach(
            m_drive,
            () -> -m_driver.getLeftY(),
            approachPose);
    }

    public Command setCoralAlgaeModeCommand()
    {
        return Commands.runOnce(
            () -> {
                coralModeEnabled = !coralModeEnabled;
            });
    }

    /** Use this method to define your joystick and button -> command mappings. */
    private void configureControllerBindings()
    {
        // Default command, normal field-relative drive
        m_drive.setDefaultCommand(joystickDrive());

        // Driver Back Button: Reset gyro / odometry
        final Runnable setPose =
            Constants.currentMode == Constants.Mode.SIM
                ? () -> m_drive.setPose(m_driveSimulation.getSimulatedDriveTrainPose())
                : () -> m_drive
                    .setPose(new Pose2d(m_drive.getPose().getTranslation(), new Rotation2d()));
        m_driver
            .back()
            .onTrue(
                Commands.runOnce(setPose).ignoringDisable(true));

        // Driver Left Bumper: Face Nearest Reef Face
        // m_driver.leftBumper()
        // .whileTrue(
        // joystickDriveAtAngle(
        // () -> FieldConstants.getNearestReefFace(m_drive.getPose()).getRotation()
        // .rotateBy(Rotation2d.k180deg)));

        // Driver Left Bumper + Right Stick Right: Approach Nearest Right-Side Reef Branch
        // m_driver.leftBumper().and(m_driver.axisGreaterThan(XboxController.Axis.kRightX.value,
        // 0.8))
        // .whileTrue(
        // joystickApproach(
        // () -> FieldConstants.getNearestReefBranch(m_drive.getPose(), ReefSide.RIGHT)));

        // Driver Left Bumper + Right Stick Left: Approach Nearest Left-Side Reef Branch
        // m_driver.leftBumper().and(m_driver.axisLessThan(XboxController.Axis.kRightX.value, -0.8))
        // .whileTrue(
        // joystickApproach(
        // () -> FieldConstants.getNearestReefBranch(m_drive.getPose(), ReefSide.LEFT)));

        // Driver Left Bumper + Right Bumper: Approach Nearest Reef Face
        // m_driver.leftBumper().and(m_driver.rightBumper())
        // .whileTrue(
        // joystickApproach(() -> FieldConstants.getNearestReefFace(m_drive.getPose())));

        // Driver A Button: Send Arm and Elevator to LEVEL_1
        m_driver
            .a()
            .onTrue(
                m_superStruct.getTransitionCommand(Arm.State.LEVEL_1, Elevator.State.LEVEL_1));

        // Driver A Button held and Right Bumper Pressed: Send Arm and Elevator to Processor
        // m_driver
        // .a().and(isCoralMode.negate())
        // .onTrue(
        // m_superStruct.getTransitionCommand(Arm.State.ALGAE_GROUND,
        // Elevator.State.ALGAE_SCORE));

        // Driver X Button: Send Arm and Elevator to LEVEL_2
        m_driver
            .x()
            .onTrue(
                m_superStruct.getTransitionCommand(Arm.State.LEVEL_2, Elevator.State.LEVEL_2));

        // Driver B Button: Send Arm and Elevator to LEVEL_3
        m_driver
            .b()
            .onTrue(
                m_superStruct.getTransitionCommand(Arm.State.LEVEL_3, Elevator.State.LEVEL_3));

        // Driver Y Button: Send Arm and Elevator to LEVEL_4
        m_driver
            .y()
            .onTrue(
                m_superStruct.getTransitionCommand(Arm.State.LEVEL_4, Elevator.State.LEVEL_4));

        // Driver Y Button held and Right Bumper having been pressed to ALGAE mode: Send Arm and
        // Elevator to NET
        // m_driver
        // .y().and(isCoralMode.negate())
        // .onTrue(
        // m_superStruct.getTransitionCommand(Arm.State.LEVEL_4, Elevator.State.BARGE));
        // TODO: Test Arm Level 4 in Sim

        // Driver Right Trigger: Place Coral or Algae (Should be done once the robot is in position)
        m_driver.rightTrigger()
            .whileTrue(m_clawRoller.setStateCommand(ClawRoller.State.EJECT));



        // Driver Left Trigger: Drivetrain drive at coral station angle, prepare the elevator and
        // arm, Get Ready to Intake Coral
        m_driver
            .leftTrigger()
            .whileTrue(
                Commands.sequence(
                    m_profiledElevator.setStateCommand(Elevator.State.CORAL_INTAKE),
                    Commands.waitUntil(() -> m_profiledElevator.atPosition(0.1))
                        .andThen(Commands.parallel(
                            m_profiledArm.setStateCommand(Arm.State.CORAL_INTAKE),
                            m_clawRoller.setStateCommand(ClawRoller.State.INTAKE))
                            .until(m_clawRollerLaserCAN.triggered))));
        // m_driver
        // .leftTrigger().and(isCoralMode)
        // .whileTrue(
        // Commands.sequence(
        // joystickDriveAtAngle(
        // () -> FieldConstants.getNearestCoralStation(m_drive.getPose())
        // .getRotation()),
        // m_profiledElevator.setStateCommand(Elevator.State.CORAL_INTAKE),
        // Commands.waitUntil(() -> m_profiledElevator.atPosition(0.1))
        // .andThen(Commands.parallel(
        // m_profiledArm.setStateCommand(Arm.State.CORAL_INTAKE),
        // m_clawRoller.setStateCommand(ClawRoller.State.INTAKE)),
        // Commands.waitUntil(m_clawRollerLaserCAN.triggered)
        // .andThen(m_clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL))))
        // .andThen(
        // Commands.parallel(
        // m_clawRoller.setStateCommand(ClawRoller.State.OFF),
        // m_profiledArm.setStateCommand(Arm.State.STOW))));

        // Driver Left Trigger + Right Bumper: Algae Intake
        // m_driver.leftTrigger().and(isCoralMode.negate()).whileTrue(
        // Commands.sequence(
        // (FieldConstants.getNearestReefBranch(m_drive.getPose(), ReefSide.RIGHT)
        // .getTranslation().getX() > 0)
        // ? m_profiledElevator.setStateCommand(Elevator.State.ALGAE_HIGH)
        // : m_profiledElevator.setStateCommand(Elevator.State.ALGAE_LOW),
        // m_clawRoller.setStateCommand(ClawRoller.State.INTAKE),
        // Commands.waitUntil(m_clawRoller.stalled)
        // .andThen(m_clawRoller.setStateCommand(ClawRoller.State.OFF)))
        // .andThen(m_profiledElevator.setStateCommand(Elevator.State.STOW)));
        // m_driver.rightTrigger().whileTrue(m_clawRoller.setStateCommand(ClawRoller.State.SCORE));

        // Driver Start Button: Climb Request (toggle)
        // m_driver.start().onTrue(Commands.runOnce(() -> {
        // m_profiledClimber.climbRequested = true;
        // m_profiledClimber.climbStep += 1;
        // }));

        // Climb step 1: Get the Arm Down, then the Elevator down, and then and move climber to prep
        // m_profiledClimber.getClimbRequest().and(m_profiledClimber.getClimbStep1()).whileTrue(
        // Commands.parallel(
        // Commands.parallel(
        // m_profiledArm.setStateCommand(Arm.State.CLIMB),
        // Commands.waitUntil(() -> m_profiledArm.atPosition(0.1))
        // .andThen(m_profiledElevator.setStateCommand(Elevator.State.STOW))),
        // Commands
        // .waitUntil(
        // () -> m_profiledElevator.atPosition(0.1) && m_profiledArm.atPosition(0.1)))
        // .andThen(m_profiledClimber.setStateCommand(Climber.State.PREP)));

        // Climb step 2: Move climber to climb
        // m_profiledClimber.getClimbRequest().and(m_profiledClimber.getClimbStep2())
        // .whileTrue(
        // m_profiledClimber.setStateCommand(Climber.State.CLIMB)
        // .until(m_profiledClimber.getClimbedTrigger()));

        // m_profiledClimber.getClimbedTrigger().onTrue(m_profiledClimber.climbedAlertCommand());

        // Driver POV Right: End Climbing Sequence if needed
        // m_driver
        // .povRight()
        // .onTrue(
        // Commands.runOnce(
        // () -> {
        // m_profiledClimber.climbRequested = false;
        // m_profiledClimber.climbStep = 0;
        // }));

        // Slow drivetrain to 25% while climbing
        // m_profiledClimber.getClimbRequest().whileTrue(
        // DriveCommands.joystickDrive(
        // m_drive,
        // () -> -m_driver.getLeftY() * 0.25,
        // () -> -m_driver.getLeftX() * 0.25,
        // () -> -m_driver.getRightX() * 0.25));

        // Driver POV Down: Zero the Elevator (HOMING)
        m_driver.povDown().whileTrue(
            m_profiledElevator.setStateCommand(Elevator.State.HOMING)
                .until(m_profiledElevator.getHomedTrigger())
                .andThen(m_profiledElevator.zeroSensorCommand()));

        // Driver Right Bumper: Toggle between Coral and Algae Modes.
        // Make sure the Approach nearest reef face does not mess with this
        // m_driver.rightBumper()
        // .onTrue(
        // !m_driver.leftBumper().getAsBoolean() ? setCoralAlgaeModeCommand()
        // : Commands.runOnce(() -> {
        // }));

    }

    /**
     * Register Named commands for use in PathPlanner
     */
    private void registerNamedCommands()
    {
        // Go to the L1 Position
        NamedCommands.registerCommand(
            "L1",
            Commands.parallel(
                m_profiledElevator.setStateCommand(Elevator.State.LEVEL_1),
                m_profiledArm.setStateCommand(Arm.State.LEVEL_1)));
        // Go to the L2 Position
        NamedCommands.registerCommand(
            "L2",
            Commands.parallel(
                m_profiledElevator.setStateCommand(Elevator.State.LEVEL_2),
                m_profiledArm.setStateCommand(Arm.State.LEVEL_2)));
        // Go to the L3 Position
        NamedCommands.registerCommand(
            "L3",
            Commands.parallel(
                m_profiledElevator.setStateCommand(Elevator.State.LEVEL_3),
                m_profiledArm.setStateCommand(Arm.State.LEVEL_3)));
        // Go to the L4 Position
        NamedCommands.registerCommand(
            "L4",
            Commands.parallel(
                m_profiledElevator.setStateCommand(Elevator.State.LEVEL_4),
                Commands.waitUntil(() -> m_profiledElevator.atPosition(1))
                    .andThen(m_profiledArm.setStateCommand(Arm.State.LEVEL_4))));
        // Go to the Home Position
        NamedCommands.registerCommand(
            "Home",
            Commands.parallel(
                m_profiledElevator.setStateCommand(Elevator.State.STOW),
                m_profiledArm.setStateCommand(Arm.State.STOW)));

        // Wait for intake laserCAN to be triggered
        NamedCommands.registerCommand("WaitForCoral",
            Commands.waitUntil(m_intakeLaserCAN.triggered));

        // Wait for intake laserCAN to be triggered
        NamedCommands.registerCommand("WaitForCoral",
            Commands.waitUntil(m_intakeLaserCAN.triggered));

        // Intake Coral
        NamedCommands.registerCommand(
            "IntakeCoral",
            m_clawRoller.setStateCommand(ClawRoller.State.INTAKE)
                .until(m_clawRollerLaserCAN.triggered)
                .andThen(m_clawRoller.setStateCommandNoEnd(ClawRoller.State.HOLDCORAL))
                .andThen(Commands.waitUntil(() -> m_clawRoller.atPosition(1))));

        // Intake Algae
        NamedCommands.registerCommand(
            "IntakeAlgae",
            Commands.sequence(
                (FieldConstants.getNearestReefBranch(m_drive.getPose(), ReefSide.RIGHT)
                    .getTranslation().getX() > 0)
                        ? m_profiledElevator.setStateCommand(Elevator.State.ALGAE_HIGH)
                        : m_profiledElevator.setStateCommand(Elevator.State.ALGAE_LOW),
                m_clawRoller.setStateCommand(ClawRoller.State.INTAKE),
                Commands.waitUntil(m_clawRoller.stalled)
                    .andThen(m_clawRoller.setStateCommand(ClawRoller.State.OFF)))
                .andThen(m_profiledElevator.setStateCommand(Elevator.State.STOW)));

        NamedCommands.registerCommand(
            "Score",
            m_clawRoller.setStateCommandNoEnd(ClawRoller.State.SCORE)
                .andThen(Commands.waitUntil(m_clawRollerLaserCAN.triggered.negate()))
                .andThen(m_clawRoller.setStateCommandNoEnd(ClawRoller.State.OFF)));
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

    /*
     * Simulation-specific routines
     */
    public void resetSimulation()
    {
        if (Constants.currentMode != Constants.Mode.SIM)
            return;

        m_drive.setPose(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void resetSimulationField()
    {
        if (Constants.currentMode != Constants.Mode.SIM)
            return;
    }

    public void displaySimFieldToAdvantageScope()
    {
        if (Constants.currentMode != Constants.Mode.SIM)
            return;

        // SimulatedArena.getInstance().addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(2,
        // 2)));
        Logger.recordOutput(
            "FieldSimulation/RobotPosition", m_driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
            "FieldSimulation/Coral",
            SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
            "FieldSimulation/Algae",
            SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }
}
