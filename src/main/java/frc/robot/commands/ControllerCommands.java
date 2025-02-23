package frc.robot.commands;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControllerCommands {

    // Controller
    private static CommandXboxController m_driver = null;
    private static GenericHID m_driveRmbl;
    private static Timer timer = new Timer();

            
    private ControllerCommands()
    {  
    }
        
    public static void setController(CommandXboxController driverCtrl) {
        m_driver = driverCtrl;
        m_driveRmbl = m_driver.getHID();
    }

    /**
    * Return a Command that rumbled both sides of the driver controller at a specific intensity for a set amount of time. 
    * Intensity should be between 0 and 1
    */
    public static Command rumbleForTime(double seconds, double intensity)
    {
        return Commands.startEnd(() -> {
                timer.restart();
                m_driveRmbl.setRumble(GenericHID.RumbleType.kBothRumble, intensity);},
            () -> {m_driveRmbl.setRumble(GenericHID.RumbleType.kBothRumble, 0); } )
        .until(() -> timer.get() >= seconds);

    }

    /**
     * Return a Command that rumbled both sides of the driver controller at a specific intensity until a condition is met. 
     * Intensity should be between 0 and 1
     */
    public static Command rumbleUntilCondition(double intensity, BooleanSupplier condition)
    {
        return Commands.startEnd(
            () -> {m_driveRmbl.setRumble(GenericHID.RumbleType.kBothRumble, intensity);},
            () -> {m_driveRmbl.setRumble(GenericHID.RumbleType.kBothRumble, 0);} )
        .until(condition);
    }

}
