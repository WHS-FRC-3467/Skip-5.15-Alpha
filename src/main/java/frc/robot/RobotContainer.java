// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ProfiledCoralRoller.ProfiledCoralRoller;
import frc.robot.subsystems.ProfiledCoralRoller.ProfiledCoralRollerIO;
import frc.robot.subsystems.ProfiledCoralRoller.ProfiledCoralRollerIOSim;
import frc.robot.subsystems.ProfiledCoralRoller.SampleProfiledRollerIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.util.sensors.LaserCanSensor;
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

  // AK-enabled Subsystems
  private final Drive m_drive;

  private final ProfiledCoralRoller m_sampleProfiledRollerSubsystem;

  private final LaserCanSensor m_laserCanSensor;

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {

    switch (Constants.currentMode) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
        m_drive = new Drive(new GyroIOPigeon2(), new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight));

        m_sampleProfiledRollerSubsystem =
            new ProfiledCoralRoller(new SampleProfiledRollerIOTalonFX(), false);

        m_laserCanSensor = new LaserCanSensor(Ports.INTAKE_LASERCAN.getDeviceNumber(), 180);
        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
        m_drive = new Drive(new GyroIO() {}, new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight), new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight));

        m_sampleProfiledRollerSubsystem =
            new ProfiledCoralRoller(new ProfiledCoralRollerIOSim(), true);

        m_laserCanSensor = null; // TODO

        break;

      // Replayed robot, disable IO implementations
      default:
        m_drive = new Drive(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {},
            new ModuleIO() {}, new ModuleIO() {});

        m_sampleProfiledRollerSubsystem =
            new ProfiledCoralRoller(new ProfiledCoralRollerIO() {}, false);

        m_laserCanSensor = null; // TODO
        break;
    }

    // Set up auto routines
    m_autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    m_autoChooser.addOption("Drive Wheel Radius Characterization",
        DriveCommands.wheelRadiusCharacterization(m_drive));
    m_autoChooser.addOption("Drive Simple FF Characterization",
        DriveCommands.feedforwardCharacterization(m_drive));
    m_autoChooser.addOption("Drive SysId (Quasistatic Forward)",
        m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption("Drive SysId (Quasistatic Reverse)",
        m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_autoChooser.addOption("Drive SysId (Dynamic Forward)",
        m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption("Drive SysId (Dynamic Reverse)",
        m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the controller button and joystick bindings
    configureControllerBindings();

    // Detect if controllers are missing / Stop multiple warnings
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /** Use this method to define your joystick and button -> command mappings. */
  private void configureControllerBindings() {

    // Default command, normal field-relative drive
    m_drive.setDefaultCommand(DriveCommands.joystickDrive(m_drive, () -> -m_driver.getLeftY(),
        () -> -m_driver.getLeftX(), () -> -m_driver.getRightX()));

    // Driver A Button: Lock to 0°
    m_driver.a().whileTrue(DriveCommands.joystickDriveAtAngle(m_drive, () -> -m_driver.getLeftY(),
        () -> -m_driver.getLeftX(), () -> new Rotation2d()));

    // Driver X Button: Switch wheel modules to X pattern
    m_driver.x().onTrue(Commands.runOnce(m_drive::stopWithX, m_drive));

    // Driver B Button: Reset gyro to 0°
    m_driver.b()
        .onTrue(Commands.runOnce(
            () -> m_drive.setPose(new Pose2d(m_drive.getPose().getTranslation(), new Rotation2d())),
            m_drive).ignoringDisable(true));

    // Driver Y Button: Intake Coral
    m_driver.y().onTrue(new IntakeCommand(m_sampleProfiledRollerSubsystem, m_laserCanSensor));
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
