// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.Vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmIO;
import frc.robot.subsystems.Arm.ArmIOSim;
import frc.robot.subsystems.Arm.ArmIOTalonFX;
import frc.robot.subsystems.Elevator.*;
import frc.robot.subsystems.SampleProfiledRoller.SampleProfiledRoller;
import frc.robot.subsystems.SampleProfiledRoller.SampleProfiledRollerIO;
import frc.robot.subsystems.SampleProfiledRoller.SampleProfiledRollerIOSim;
import frc.robot.subsystems.SampleProfiledRoller.SampleProfiledRollerIOTalonFX;
// import frc.robot.subsystems.SampleRollers.SampleRollers;
// import frc.robot.subsystems.SampleRollers.SampleRollersIO;
// import frc.robot.subsystems.SampleRollers.SampleRollersIOSim;
// import frc.robot.subsystems.SampleRollers.SampleRollersIOTalonFX;
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

  private static final int RIGHT_STICK_HORIZONTAL = 4;
  private static final int RIGHT_STICK_VERTICAL = 5;

  // Controllers
  private final CommandXboxController m_driver = new CommandXboxController(0);
  // private final CommandXboxController m_operator = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> m_autoChooser;

  // Maple Sim
  private SwerveDriveSimulation driveSimulation = null;

  // AK-enabled Subsystems
  private final Drive m_drive;
  // private final SampleRollers m_sampleRollersSubsystem;
  private final Arm m_sampleArmSubsystem;
  private final Elevator m_profiledElevator;
  private final SampleProfiledRoller m_sampleProfiledRollerSubsystem;

  public final Vision m_vision;

  // Non-AK-enabled Subsystems
  // private final SimpleSubsystem m_simpleSubsystem = new SimpleSubsystem();
  // private final ComplexSubsystem m_complexSubsystem = new ComplexSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

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
                (robotPose) -> {});

        // m_sampleRollersSubsystem = new SampleRollers(new SampleRollersIOTalonFX());
        m_sampleArmSubsystem = new Arm(new ArmIOTalonFX(), false);
        m_profiledElevator = new Elevator(new ElevatorIOTalonFX(), false);
        m_sampleProfiledRollerSubsystem =
            new SampleProfiledRoller(new SampleProfiledRollerIOTalonFX(), false);

        m_vision =
            new Vision(
                m_drive,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1));

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        driveSimulation =
            new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        m_drive =
            new Drive(
                new GyroIOSim(this.driveSimulation.getGyroSimulation()),
                new ModuleIOTalonFXSim(
                    TunerConstants.FrontLeft, this.driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(
                    TunerConstants.FrontRight, this.driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(
                    TunerConstants.BackLeft, this.driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(
                    TunerConstants.BackRight, this.driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);
        // m_drive.setPose(startingPose);
        // m_sampleRollersSubsystem = new SampleRollers(new SampleRollersIOSim());
        m_sampleArmSubsystem = new Arm(new ArmIOSim(), true);
        m_profiledElevator = new Elevator(new ElevatorIOSim(), true);
        m_sampleProfiledRollerSubsystem =
            new SampleProfiledRoller(new SampleProfiledRollerIOSim(), true);

        m_vision =
            new Vision(
                m_drive,
                new VisionIOPhotonVisionSim(
                    camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    camera1Name, robotToCamera1, driveSimulation::getSimulatedDriveTrainPose));

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
                (robotPose) -> {});
        // m_sampleRollersSubsystem = new SampleRollers(new SampleRollersIO() {});
        m_sampleArmSubsystem = new Arm(new ArmIO() {}, true);
        m_profiledElevator = new Elevator(new ElevatorIO() {}, true);
        m_sampleProfiledRollerSubsystem =
            new SampleProfiledRoller(new SampleProfiledRollerIO() {}, false);

        m_vision = new Vision(m_drive, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // Logic Triggers

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
    // m_driver
    // .a()
    // .whileTrue(
    // DriveCommands.joystickDriveAtAngle(
    // m_drive,
    // () -> -m_driver.getLeftY(),
    // () -> -m_driver.getLeftX(),
    // () -> new Rotation2d()));
    // Driver A Button: Lock to 0°

    m_driver
        .leftBumper()
        .whileTrue(
            DriveCommands.joystickOrbitAtAngle(
                m_drive,
                () -> m_driver.getLeftY(),
                () -> m_driver.getLeftX(),
                () ->
                    RobotState.getInstance().getAngleToTarget(m_drive.getPose().getTranslation())));

    m_driver
        .leftBumper()
        .and(m_driver.axisGreaterThan(RIGHT_STICK_HORIZONTAL, 0.8))
        .onTrue(
            Commands.runOnce(
                () -> RobotState.getInstance().setTarget(RobotState.TARGET.RIGHT_CORAL_STATION)));

    m_driver
        .leftBumper()
        .and(m_driver.axisLessThan(RIGHT_STICK_HORIZONTAL, -0.8))
        .onTrue(
            Commands.runOnce(
                () -> RobotState.getInstance().setTarget(RobotState.TARGET.LEFT_CORAL_STATION)));

    m_driver
        .leftBumper()
        .and(m_driver.axisLessThan(RIGHT_STICK_VERTICAL, -0.8))
        .onTrue(Commands.runOnce(() -> RobotState.getInstance().setTarget(RobotState.TARGET.REEF)));

    // Driver X Button: Switch wheel modules to X pattern
    // m_driver.x().onTrue(Commands.runOnce(m_drive::stopWithX, m_drive));
    // Reset gyro / odometry
    final Runnable setPose =
        Constants.currentMode == Constants.Mode.SIM
            ? () -> m_drive.setPose(driveSimulation.getSimulatedDriveTrainPose())
            : () ->
                m_drive.setPose(new Pose2d(m_drive.getPose().getTranslation(), new Rotation2d()));
    m_driver.x().onTrue(Commands.runOnce(setPose).ignoringDisable(true));

    // Driver B Button: Reset gyro to 0°
    // m_driver
    // .b()
    // .onTrue(
    // Commands.runOnce(
    // () -> m_drive.setPose(
    // new Pose2d(m_drive.getPose().getTranslation(), new Rotation2d())),
    // m_drive)
    // .ignoringDisable(true));

    // // Driver X Button: Run the Sample Roller in Eject direction when held
    // m_driver.x().whileTrue(m_sampleRollersSubsystem.setStateCommand(SampleRollers.State.EJECT));
    // // Driver Y Button: Run the Sample Roller in Intake direction when held
    // m_driver.y().whileTrue(m_sampleRollersSubsystem.setStateCommand(SampleRollers.State.INTAKE));

    // Driver Left & Right Bumpers: Run the Sample Profiled Roller out and in when
    // held
    // m_driver
    // .leftBumper()
    // .whileTrue(
    // m_sampleProfiledRollerSubsystem.setStateCommand(SampleProfiledRoller.State.EJECT));

    m_driver
        .povDown()
        .whileTrue(
            Commands.parallel(
                m_sampleArmSubsystem.setStateCommand(Arm.State.GROUND),
                m_profiledElevator.setStateCommand(Elevator.State.HOME)));

    m_driver
        .povLeft()
        .whileTrue(
            Commands.runOnce(
                () ->
                    SimulatedArena.getInstance()
                        .addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(2, 2)))));

    // Driver Right Trigger: Run the Sample Profiled Roller to the requested
    // position
    // Driver Right Trigger: Run the Sample Profiled Roller to the requested
    // position
    m_driver
        .rightTrigger()
        .whileTrue(
            m_sampleProfiledRollerSubsystem.setStateCommand(SampleProfiledRoller.State.EJECT));

    m_driver
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                () ->
                    SimulatedArena.getInstance()
                        .addGamePieceProjectile(
                            new ReefscapeCoralOnFly(
                                // Obtain robot position from drive simulation
                                driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                                // The scoring mechanism is installed at (0.46, 0) (meters) on the
                                // robot
                                new Translation2d(0.48, 0),
                                // Obtain robot speed from drive simulation
                                driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                // Obtain robot facing from drive simulation
                                driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                                // The height at which the coral is ejected
                                Meters.of(2.3),
                                // The initial speed of the coral
                                MetersPerSecond.of(1),
                                // The coral is ejected vertically downwards
                                Degrees.of(-75)))));

    // Driver POV Down: Bring Arm and Elevator to Home position
    m_driver
        .leftTrigger()
        .onTrue(
            Commands.parallel(
                m_sampleArmSubsystem.setStateCommand(Arm.State.INTAKE),
                m_profiledElevator.setStateCommand(Elevator.State.INTAKE)));

    // Driver POV Left: Send Arm and Elevator to LEVEL_1
    m_driver
        .a()
        .onTrue(
            Commands.parallel(
                m_sampleArmSubsystem.setStateCommand(Arm.State.LEVEL_1),
                Commands.waitUntil(() -> m_sampleArmSubsystem.atPosition(0))
                    .andThen(m_profiledElevator.setStateCommand(Elevator.State.LEVEL_1))));

    // Driver POV Up: Send Arm and Elevator to LEVEL_2
    m_driver
        .x()
        .onTrue(
            Commands.parallel(
                m_sampleArmSubsystem.setStateCommand(Arm.State.LEVEL_2),
                m_profiledElevator.setStateCommand(Elevator.State.LEVEL_2)));

    // Driver POV Right: Send Arm and Elevator to LEVEL_3
    m_driver
        .y()
        .onTrue(
            Commands.parallel(
                m_sampleArmSubsystem.setStateCommand(Arm.State.LEVEL_3),
                m_profiledElevator.setStateCommand(Elevator.State.LEVEL_3)));

    // Driver BACK: Send Arm to LEVEL_3 and Elevator to LEVEL_4
    m_driver
        .b()
        .onTrue(
            Commands.parallel(
                m_profiledElevator.setStateCommand(Elevator.State.LEVEL_4),
                Commands.waitUntil(() -> m_profiledElevator.atPosition(0.1))
                    .andThen(m_sampleArmSubsystem.setStateCommand(Arm.State.LEVEL_4))));

    // Driver POV Center: Send Elevator to Homing
    m_driver
        .start()
        .onTrue(
            m_profiledElevator
                .setStateCommand(Elevator.State.HOMING)
                .until(m_profiledElevator.getHomedTrigger())
                .andThen(m_profiledElevator.zeroSensorCommand()));

    m_profiledElevator.getHomedTrigger().onTrue(m_profiledElevator.homedAlertCommand());

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

  public void resetSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    m_drive.setPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    // SimulatedArena.getInstance().addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(2,
    // 2)));
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
