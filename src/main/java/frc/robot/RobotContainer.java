// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.Vision.VisionConstants.*;

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
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmIO;
import frc.robot.subsystems.Arm.ArmIOSim;
import frc.robot.subsystems.Arm.ArmIOTalonFX;
import frc.robot.subsystems.Elevator.*;
import frc.robot.subsystems.SampleProfiledRoller.SampleProfiledRoller;
import frc.robot.subsystems.SampleProfiledRoller.SampleProfiledRollerIO;
import frc.robot.subsystems.SampleProfiledRoller.SampleProfiledRollerIOSim;
import frc.robot.subsystems.SampleProfiledRoller.SampleProfiledRollerIOTalonFX;
import frc.robot.subsystems.SampleRollers.SampleRollers;
import frc.robot.subsystems.SampleRollers.SampleRollersIO;
import frc.robot.subsystems.SampleRollers.SampleRollersIOSim;
import frc.robot.subsystems.SampleRollers.SampleRollersIOTalonFX;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionIO;
import frc.robot.subsystems.Vision.VisionIOPhotonVision;
import frc.robot.subsystems.Vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
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
  // private final CommandXboxController m_operator = new
  // CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> m_autoChooser;

  // AK-enabled Subsystems
  private final Drive m_drive;
  private final SampleRollers m_sampleRollersSubsystem;
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
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        m_sampleRollersSubsystem = new SampleRollers(new SampleRollersIOTalonFX());
        m_sampleArmSubsystem = new Arm(new ArmIOTalonFX(), false);
        m_profiledElevator = new Elevator(new ElevatorIOTalonFX(), false);
        m_sampleProfiledRollerSubsystem =
            new SampleProfiledRoller(new SampleProfiledRollerIOTalonFX(), false);

        m_vision =
            new Vision(
                m_drive::addVisionMeasurement,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1));

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

        m_sampleRollersSubsystem = new SampleRollers(new SampleRollersIOSim());
        m_sampleArmSubsystem = new Arm(new ArmIOSim(), true);
        m_profiledElevator = new Elevator(new ElevatorIOSim(), true);
        m_sampleProfiledRollerSubsystem =
            new SampleProfiledRoller(new SampleProfiledRollerIOSim(), true);

        m_vision =
            new Vision(
                m_drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, m_drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, m_drive::getPose));
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
        m_sampleRollersSubsystem = new SampleRollers(new SampleRollersIO() {});
        m_sampleArmSubsystem = new Arm(new ArmIO() {}, true);
        m_profiledElevator = new Elevator(new ElevatorIO() {}, true);
        m_sampleProfiledRollerSubsystem =
            new SampleProfiledRoller(new SampleProfiledRollerIO() {}, false);

        m_vision = new Vision(m_drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
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

    // Driver X Button: Switch wheel modules to X pattern
    m_driver.x().onTrue(Commands.runOnce(m_drive::stopWithX, m_drive));

    // Driver B Button: Reset gyro to 0°
    m_driver
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        m_drive.setPose(
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
                m_sampleArmSubsystem.setStateCommand(Arm.State.HOME),
                m_profiledElevator.setStateCommand(Elevator.State.HOME)));

    // Driver POV Left: Send Arm and Elevator to LEVEL_1
    m_driver
        .povLeft()
        .onTrue(
            Commands.parallel(
                m_sampleArmSubsystem.setStateCommand(Arm.State.LEVEL_1),
                m_profiledElevator.setStateCommand(Elevator.State.LEVEL_1)));

    // Driver POV Up: Send Arm and Elevator to LEVEL_2
    m_driver
        .povUp()
        .onTrue(
            Commands.parallel(
                m_sampleArmSubsystem.setStateCommand(Arm.State.LEVEL_2),
                m_profiledElevator.setStateCommand(Elevator.State.LEVEL_2)));

    // Driver POV Right: Send Arm and Elevator to LEVEL_3
    m_driver
        .povRight()
        .onTrue(
            Commands.parallel(
                m_sampleArmSubsystem.setStateCommand(Arm.State.LEVEL_3),
                m_profiledElevator.setStateCommand(Elevator.State.LEVEL_3)));

    // Driver BACK: Send Arm to LEVEL_3 and Elevator to LEVEL_4
    m_driver
        .back()
        .onTrue(
            Commands.parallel(
                m_sampleArmSubsystem.setStateCommand(Arm.State.LEVEL_3),
                m_profiledElevator.setStateCommand(Elevator.State.LEVEL_3)));

    // Driver POV Center: Send Elevator to Homing
    /*
     * m_driver
     * .start()
     * .onTrue(
     * m_profiledElevator
     * .setStateCommand(Elevator.State.HOMING)
     * .until(m_profiledElevator.getHomedTrigger())
     * .andThen(m_profiledElevator.zeroSensorCommand()));
     */

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
