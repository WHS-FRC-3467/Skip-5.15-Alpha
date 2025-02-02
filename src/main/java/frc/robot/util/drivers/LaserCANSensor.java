// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.drivers;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;

/** Add your docs here. */
public class LaserCANSensor {
    private LaserCan lc;
    private double closeCut = 200;

    public LaserCANSensor(int ID, double closeCut)
    {
        lc = new LaserCan(ID);
        this.closeCut = closeCut;
        try {
            lc.setRangingMode(LaserCan.RangingMode.SHORT);
            lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 6, 6));
            lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }
    }

    public boolean isClose()
    {
        Measurement measurement = lc.getMeasurement();
        if (measurement != null
            && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            return (measurement.distance_mm < closeCut);
        } else {
            return false;
        }

    }
}
