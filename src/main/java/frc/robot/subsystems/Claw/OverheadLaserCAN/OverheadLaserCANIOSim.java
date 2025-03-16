package frc.robot.subsystems.Claw.OverheadLaserCAN;

import frc.robot.subsystems.GenericLaserCANSubsystem.GenericLaserCANSubsystemIOImpl;

public class OverheadLaserCANIOSim extends GenericLaserCANSubsystemIOImpl
    implements OverheadLaserCANIO {

    public OverheadLaserCANIOSim()
    {
        super(OverheadLaserCANConstants.kSubSysConstants, true);
    }
}
