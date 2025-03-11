package frc.robot.util;

import java.util.function.BooleanSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller.Axis;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class WindupXboxController extends CommandXboxController {

    Timer timer = new Timer();
    private GenericHID m_driveRmbl;
    private double m_deadband = 0.0;

    /**
     * Construct an instance of a controller.
     *
     * @param port The port index on the Driver Station that the controller is plugged into.
     */
    public WindupXboxController(int port)
    {
        super(port);
        m_driveRmbl = this.getHID();
    }

    /**
     * Return a Command that rumbled both sides of the driver controller at a specific intensity for
     * a set amount of time. Intensity should be between 0 and 1
     */
    public Command rumbleForTime(double seconds, double intensity)
    {
        return Commands.startEnd(() -> {
            timer.restart();
            m_driveRmbl.setRumble(GenericHID.RumbleType.kBothRumble, intensity);
        },
            () -> {
                m_driveRmbl.setRumble(GenericHID.RumbleType.kBothRumble, 0);
            })
            .until(() -> timer.get() >= seconds);

    }

    /**
     * Return a Command that rumbled both sides of the driver controller at a specific intensity
     * until a condition is met. Intensity should be between 0 and 1
     */
    public Command rumbleUntilCondition(double intensity, BooleanSupplier condition)
    {
        return Commands.startEnd(
            () -> {
                m_driveRmbl.setRumble(GenericHID.RumbleType.kBothRumble, intensity);
            },
            () -> {
                m_driveRmbl.setRumble(GenericHID.RumbleType.kBothRumble, 0);
            })
            .until(condition);
    }

    public WindupXboxController withDeadband(double deadband)
    {
        m_deadband = deadband;
        return this;
    }

    /*
     * 
     * Get the X axis value of left side of the controller. Right is positive.**@return The axis
     * value.
     */

    @Override
    public double getLeftX()
    {
        return MathUtil.applyDeadband(getRawAxis(Axis.kLeftX.value), m_deadband);
    }

    /**
     * Get the X axis value of right side of the controller. Right is positive.
     *
     * @return The axis value.
     */

    @Override
    public double getRightX()
    {
        return MathUtil.applyDeadband(getRawAxis(Axis.kRightX.value), m_deadband);
    }

    /**
     * Get the Y axis value of left side of the controller. Back is positive.
     *
     * @return The axis value.
     */

    @Override
    public double getLeftY()
    {
        return MathUtil.applyDeadband(getRawAxis(Axis.kLeftY.value), m_deadband);
    }

    /**
     * Get the Y axis value of right side of the controller. Back is positive.
     *
     * @return The axis value.
     */

    @Override
    public double getRightY()
    {
        return MathUtil.applyDeadband(getRawAxis(Axis.kRightY.value), m_deadband);
    }
}
