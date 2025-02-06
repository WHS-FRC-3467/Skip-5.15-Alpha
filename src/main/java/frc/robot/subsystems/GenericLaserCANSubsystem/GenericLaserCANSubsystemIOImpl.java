package frc.robot.subsystems.GenericLaserCANSubsystem;

import org.littletonrobotics.junction.Logger;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import static edu.wpi.first.units.Units.*;

/**
 * Generic motion IO implementation for any motion mechanism using a TalonFX motor controller, an
 * optional follower motor, and an optional remote CANcoder encoder.
 */
public class GenericLaserCANSubsystemIOImpl implements GenericLaserCANSubsystemIO {

    private LaserCan lc;
    private String name;

    private Distance currentDistance;

    private final Alert failedConfig =
        new Alert("Failed to configure LaserCAN!", AlertType.kError);
    private final Alert sensorAlert =
        new Alert("Failed to get LaserCAN measurement", Alert.AlertType.kWarning);

    /*
     * Constructor
     */
    public GenericLaserCANSubsystemIOImpl(
        GenericLaserCANSubsystemConstants constants)
    {
        lc = new LaserCan(constants.laserCANDeviceId.getDeviceNumber());
        name = constants.kName;
        try {
            lc.setRangingMode(constants.rangingMode);
            lc.setRegionOfInterest(constants.regionOfInterest);
            lc.setTimingBudget(constants.timingBudget);
            failedConfig.set(false);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
            failedConfig.setText("Failed to configure " + name + "!");
            failedConfig.set(true);
        }
    }

    public Distance getMeasurement()
    {
        Measurement measurement = lc.getMeasurement();
        if (measurement != null
            && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            sensorAlert.set(false);
            currentDistance = Millimeters.of(measurement.distance_mm);
        } else if (measurement != null
            && measurement.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            sensorAlert
                .setText("Failed to get LaserCAN ID: " + name
                    + ", no valid measurement");
            sensorAlert.set(true);
            currentDistance = Millimeters.of(Double.POSITIVE_INFINITY);
        } else {
            sensorAlert.setText("Failed to get LaserCAN ID: " + name
                + ", measurement null");
            sensorAlert.set(true);
            currentDistance = Millimeters.of(Double.POSITIVE_INFINITY);
        }
        Logger.recordOutput("LaserCANSensors/LaserCAN" + name,
            currentDistance.in(Inches));
        return currentDistance;
    }

    @Override
    public void updateInputs(LaserCANIOInputs inputs)
    {
        inputs.distance = getMeasurement();
    }
}
