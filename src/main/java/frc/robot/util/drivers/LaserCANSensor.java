// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.drivers;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Getter;
import edu.wpi.first.wpilibj.Alert;
import static edu.wpi.first.units.Units.*;


public class LaserCANSensor {
    private LaserCan lc;
    private int CAN_ID;
    private Distance triggerDistance = Inches.zero();

    @Getter
    public Trigger nearTrigger = new Trigger(() -> getMeasurement().lte(triggerDistance));

    private final Alert failedConfig =
        new Alert("Failed to configure LaserCAN!", AlertType.kError);
    private final Alert sensorAlert =
        new Alert("Failed to get LaserCAN measurement", Alert.AlertType.kWarning);

    /**
     * Constructs a new LaserCAN sensor
     *
     * @param ID CAN ID assigned to the sensor
     * @param triggerDistance distance at which to trigger the sensor
     */
    public LaserCANSensor(int ID, Distance distance)
    {
        CAN_ID = ID;
        lc = new LaserCan(CAN_ID);
        triggerDistance = distance;
        try {
            lc.setRangingMode(LaserCan.RangingMode.SHORT);
            lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 6, 6));
            lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
            failedConfig.set(false);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
            failedConfig.setText("Failed to configure LaserCAN ID: " + ID + "!");
            failedConfig.set(true);
        }

    }

    /**
     * Returns distance reading from LaserCAN sensor
     */
    public Distance getMeasurement()
    {
        Measurement measurement = lc.getMeasurement();
        if (measurement != null
            && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            sensorAlert.set(false);
            return Millimeters.of(measurement.distance_mm);
        } else if (measurement != null
            && measurement.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            sensorAlert
                .setText("Failed to get LaserCAN ID: " + this.CAN_ID + ", no valid measurement");
            sensorAlert.set(true);
            return Millimeters.of(Double.POSITIVE_INFINITY);
        } else {
            sensorAlert.setText("Failed to get LaserCAN ID: " + this.CAN_ID + ", measurement null");
            sensorAlert.set(true);
            return Millimeters.of(Double.POSITIVE_INFINITY);
        }
    }

}
