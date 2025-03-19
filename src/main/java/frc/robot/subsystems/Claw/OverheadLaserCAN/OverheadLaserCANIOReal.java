package frc.robot.subsystems.Claw.OverheadLaserCAN;

import frc.robot.subsystems.GenericLaserCANSubsystem.GenericLaserCANSubsystemIOImpl;

public class OverheadLaserCANIOReal extends GenericLaserCANSubsystemIOImpl
    implements OverheadLaserCANIO {

    public OverheadLaserCANIOReal()
    {
        super(OverheadLaserCANConstants.kSubSysConstants, false);
    }
}
