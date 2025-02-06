package frc.robot.subsystems.ClawRoller.ClawRollerLaserCAN;

import frc.robot.subsystems.GenericLaserCANSubsystem.GenericLaserCANSubsystemIOImpl;

public class ClawRollerLaserCANIOSim extends GenericLaserCANSubsystemIOImpl
    implements ClawRollerLaserCANIO {

    public ClawRollerLaserCANIOSim()
    {
        super(ClawRollerLaserCANConstants.kSubSysConstants, true);
    }
}
