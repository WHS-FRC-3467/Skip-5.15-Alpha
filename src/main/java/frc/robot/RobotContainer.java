// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.Vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmIO;
import frc.robot.subsystems.Arm.ArmIOSim;
import frc.robot.subsystems.Arm.ArmIOTalonFX;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.ClimberIO;
import frc.robot.subsystems.Climber.ClimberIOSim;
import frc.robot.subsystems.Climber.ClimberIOTalonFX;
import frc.robot.subsystems.Elevator.*;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionIO;
import frc.robot.subsystems.Vision.VisionIOPhotonVision;
import frc.robot.subsystems.Vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFXReal;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
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
    // private final CommandXboxController m_operator = new CommandXboxController(1);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> m_autoChooser;

    // Maple Sim
    private SwerveDriveSimulation m_driveSimulation = null;

    // AK-enabled Subsystems
    private final Drive m_drive;
    private final Arm m_profiledArm;
    private final Elevator m_profiledElevator;
    private final Climber m_profiledClimber;

    public final Vision m_vision;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                m_drive =
                    new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                        new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                        new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                        new ModuleIOTalonFXReal(TunerConstants.BackRight),
                        (robotPose) -> {
                        });

                m_profiledArm = new Arm(new ArmIOTalonFX(), false);
                m_profiledElevator = new Elevator(new ElevatorIOTalonFX(), false);
                m_profiledClimber = new Climber(new ClimberIOTalonFX(), false);

                m_vision =
                    new Vision(
                        m_drive,
                        new VisionIOPhotonVision(camera0Name, robotToCamera0),
                        new VisionIOPhotonVision(camera1Name, robotToCamera1));

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

                m_vision = new Vision(m_drive, new VisionIO() {}, new VisionIO() {});
                break;
        }

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

        // Configure the controller button and joystick bindings
        configureControllerBindings();

        // Detect if controllers are missing / Stop multiple warnings
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    //Climbing Triggers
	private boolean climbRequested = false; //Whether or not a climb request is active
	private Trigger climbRequest = new Trigger(() -> climbRequested); //Trigger for climb request
	private int climbStep = 0; //Tracking what step in the climb sequence we are on

	//Triggers for each step of the climb sequence
	private Trigger climbStep1 = new Trigger(() -> climbStep == 1);
	private Trigger climbStep2 = new Trigger(() -> climbStep == 2);

    /** Use this method to define your joystick and button -> command mappings. */
    private void configureControllerBindings()
    {
        // Default command, normal field-relative drive
        m_drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                m_drive,
                () -> -m_driver.getLeftY(),
                () -> -m_driver.getLeftX(),
                () -> -m_driver.getRightX()));

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

        // POV Up Button: Run Homing Sequence
        m_driver
            .povUp()
            .onTrue(
                m_profiledElevator
                    .setStateCommand(Elevator.State.HOMING)
                    .until(m_profiledElevator.getHomedTrigger())
                    .andThen(m_profiledElevator.zeroSensorCommand()));
        m_profiledElevator.getHomedTrigger().onTrue(m_profiledElevator.homedAlertCommand());

        // Driver Left Trigger: Bring Arm and Elevator to INTAKE position
        m_driver
            .leftTrigger()
            .onTrue(
                Commands.parallel(
                    m_profiledArm.setStateCommand(Arm.State.INTAKE),
                    m_profiledElevator.setStateCommand(Elevator.State.INTAKE)));

        // Driver A Button: Send Arm and Elevator to LEVEL_1
        m_driver
            .a()
            .onTrue(
                Commands.parallel(
                    m_profiledArm.setStateCommand(Arm.State.LEVEL_1),
                    Commands.waitUntil(() -> m_profiledArm.atPosition(0))
                        .andThen(m_profiledElevator.setStateCommand(Elevator.State.LEVEL_1))));

        // Driver X Button: Send Arm and Elevator to LEVEL_2
        m_driver
            .x()
            .onTrue(
                Commands.parallel(
                    m_profiledArm.setStateCommand(Arm.State.LEVEL_2),
                    m_profiledElevator.setStateCommand(Elevator.State.LEVEL_2)));

        // Driver B Button: Send Arm and Elevator to LEVEL_3
        m_driver
            .b()
            .onTrue(
                Commands.parallel(
                    m_profiledArm.setStateCommand(Arm.State.LEVEL_3),
                    m_profiledElevator.setStateCommand(Elevator.State.LEVEL_3)));

        // Driver Y Button: Send Arm and Elevator to LEVEL_4
        m_driver
            .y()
            .onTrue(
                Commands.parallel(
                    m_profiledElevator.setStateCommand(Elevator.State.LEVEL_4),
                    Commands.waitUntil(() -> m_profiledElevator.atPosition(0.1))
                        .andThen(m_profiledArm.setStateCommand(Arm.State.LEVEL_4))));


        // Driver Start Button: Climb Request (toggle)
		m_driver.start().onTrue(Commands.runOnce(() -> {
            climbRequested = true;
            climbStep += 1;
            }));

		//Climb step 1: Raise shooter and move climber to prep
		climbRequest.and(climbStep1).whileTrue(
				m_profiledClimber.setStateCommand(Climber.State.PREP));

		//Climb step 2: Move climber to climb
		climbRequest.and(climbStep2)
            .whileTrue(
                m_profiledClimber.setStateCommand(Climber.State.CLIMB)
                    .until(m_profiledClimber.climbedTrigger));
            
        m_profiledClimber.getClimbedTrigger().onTrue(m_profiledClimber.climbedAlertCommand());

        // Driver POV Right: End Climbing Sequence if needed
        m_driver
            .povRight()
            .onTrue(
                Commands.runOnce(
                    () -> {
                        climbRequested = false;
                        climbStep = 0;
                    }));

		//Slow drivetrain to 25% while climbing
        climbRequest.whileTrue(
            DriveCommands.joystickDrive(
                m_drive,
                () -> -m_driver.getLeftY() * 0.25,
                () -> -m_driver.getLeftX() * 0.25,
                () -> -m_driver.getRightX() * 0.25));

        // Driver uses Left Bumper for orbitting and targeting maneuvers
        m_driver
            .leftBumper()
            .whileTrue(
                DriveCommands.joystickOrbitAtAngle(
                    m_drive,
                    () -> m_driver.getLeftY(),
                    () -> m_driver.getLeftX(),
                    () -> RobotState.getInstance()
                        .getAngleToTarget(m_drive.getPose().getTranslation())));

        m_driver
            .leftBumper()
            .and(m_driver.axisGreaterThan(XboxController.Axis.kRightX.value, 0.8))
            .onTrue(
                Commands.runOnce(
                    () -> RobotState.getInstance()
                        .setTarget(RobotState.TARGET.RIGHT_CORAL_STATION)));

        m_driver
            .leftBumper()
            .and(m_driver.axisLessThan(XboxController.Axis.kRightX.value, -0.8))
            .onTrue(
                Commands.runOnce(
                    () -> RobotState.getInstance()
                        .setTarget(RobotState.TARGET.LEFT_CORAL_STATION)));

        m_driver
            .leftBumper()
            .and(m_driver.axisLessThan(XboxController.Axis.kRightY.value, -0.8))
            .onTrue(
                Commands.runOnce(() -> RobotState.getInstance().setTarget(RobotState.TARGET.REEF)));

        m_driver
            .povDown()
            .whileTrue(
                Commands.parallel(
                    m_profiledArm.setStateCommand(Arm.State.GROUND),
                    m_profiledElevator.setStateCommand(Elevator.State.HOME)));

        m_driver
            .povLeft()
            .whileTrue(
                Commands.runOnce(
                    () -> SimulatedArena.getInstance()
                        .addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(2, 2)))));

        m_driver
            .rightBumper()
            .onTrue(
                Commands.runOnce(
                    () -> SimulatedArena.getInstance()
                        .addGamePieceProjectile(
                            new ReefscapeCoralOnFly(
                                // Obtain robot position from drive simulation
                                m_driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                                // The scoring mechanism is installed at (0.46, 0) (meters) on the
                                // robot
                                new Translation2d(0.48, 0),
                                // Obtain robot speed from drive simulation
                                m_driveSimulation
                                    .getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                // Obtain robot facing from drive simulation
                                m_driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                                // The height at which the coral is ejected
                                Meters.of(2.3),
                                // The initial speed of the coral
                                MetersPerSecond.of(1),
                                // The coral is ejected vertically downwards
                                Degrees.of(-75)))));
                 
        // Driver START Button: Auto Angle to closest Reef target TODO: FIX
        m_driver
            .povRight()
            .whileTrue(
                Commands.parallel(
                    RobotState.getInstance().setTargetCommand(RobotState.getInstance().chooseReefTarget()),
                    DriveCommands.joystickDriveAtAngle(
                        m_drive,
                        () -> -m_driver.getLeftY(),
                        () -> -m_driver.getLeftX(),
                        () -> RobotState.getInstance().getAngleToTarget(m_drive.getPose().getTranslation()))));

        // Driver Right bumper: Cardinal direction to PROCESSOR
        m_driver
            .povUp()
            .whileTrue(
                Commands.parallel(
                    RobotState.getInstance().setTargetCommand(RobotState.TARGET.PROCESSOR),
                    DriveCommands.joystickDriveAtAngle(
                        m_drive,
                        () -> -m_driver.getLeftY(),
                        () -> -m_driver.getLeftX(),
                        () -> RobotState.getInstance().getAngleOfTarget().plus(Rotation2d.fromDegrees(180)))));
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
                Commands.waitUntil(() -> m_profiledElevator.atPosition(0.1))
                    .andThen(m_profiledArm.setStateCommand(Arm.State.LEVEL_4))));
        // Go to the Home Position
        NamedCommands.registerCommand(
            "Home",
            Commands.parallel(
                m_profiledElevator.setStateCommand(Elevator.State.HOME),
                m_profiledArm.setStateCommand(Arm.State.HOME)));

        NamedCommands.registerCommand(
            "SIMScore",
            Commands.runOnce(
                () -> SimulatedArena.getInstance()
                    .addGamePieceProjectile(
                        new ReefscapeCoralOnFly(
                            // Obtain robot position from drive simulation
                            m_driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                            // The scoring mechanism is installed at (0.46, 0) (meters) on the
                            // robot
                            new Translation2d(0.48, 0),
                            // Obtain robot speed from drive simulation
                            m_driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                            // Obtain robot facing from drive simulation
                            m_driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                            // The height at which the coral is ejected
                            Meters.of(2.3),
                            // The initial speed of the coral
                            MetersPerSecond.of(1),
                            // The coral is ejected vertically downwards
                            Degrees.of(-75)))));
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
