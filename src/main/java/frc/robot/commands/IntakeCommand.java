package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ProfiledCoralRoller.ProfiledCoralRoller;
import frc.robot.util.sensors.LaserCanSensor;

public class IntakeCommand extends Command {
    ProfiledCoralRoller m_roller;
    LaserCanSensor m_sensor;

    public IntakeCommand(ProfiledCoralRoller roller, LaserCanSensor sensor) {
        m_roller = roller;
        m_sensor = sensor;
    }

    public void initialize() {
        m_roller.setState(ProfiledCoralRoller.State.INTAKE);
    }

    public boolean isFinished() {
        return m_sensor.isClose();
    }
}
