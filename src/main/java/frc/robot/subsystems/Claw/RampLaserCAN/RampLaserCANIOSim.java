package frc.robot.subsystems.Claw.RampLaserCAN;

import frc.robot.subsystems.GenericLaserCANSubsystem.GenericLaserCANSubsystemIOImpl;

public class RampLaserCANIOSim extends GenericLaserCANSubsystemIOImpl
    implements RampLaserCANIO {

    public RampLaserCANIOSim()
    {
        super(RampLaserCANConstants.kSubSysConstants, true);
    }
}
