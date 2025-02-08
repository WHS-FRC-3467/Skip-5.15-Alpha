package frc.robot.subsystems.Claw.IntakeLaserCAN;

import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import frc.robot.Ports;
import frc.robot.subsystems.GenericLaserCANSubsystem.GenericLaserCANSubsystemConstants;

public class IntakeLaserCANConstants {
    public static final GenericLaserCANSubsystemConstants kSubSysConstants =
        new GenericLaserCANSubsystemConstants();

    static {
        kSubSysConstants.kName = "IntakeLaserCAN";
        kSubSysConstants.laserCANDeviceId = Ports.CLAW_LASERCAN;
        kSubSysConstants.rangingMode = RangingMode.SHORT;
        kSubSysConstants.regionOfInterest = new RegionOfInterest(10, 10, 10, 10);
        kSubSysConstants.timingBudget = TimingBudget.TIMING_BUDGET_20MS;
    }
}
