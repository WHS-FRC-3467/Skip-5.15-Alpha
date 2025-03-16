package frc.robot.subsystems.Claw.RampLaserCAN;

import frc.robot.subsystems.GenericLaserCANSubsystem.GenericLaserCANSubsystemIOImpl;

public class RampLaserCANIOReal extends GenericLaserCANSubsystemIOImpl
    implements RampLaserCANIO {

    public RampLaserCANIOReal()
    {
        super(RampLaserCANConstants.kSubSysConstants, false);
    }
}
