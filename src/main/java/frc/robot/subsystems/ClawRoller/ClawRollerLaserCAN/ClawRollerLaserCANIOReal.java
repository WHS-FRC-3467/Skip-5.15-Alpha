package frc.robot.subsystems.ClawRoller.ClawRollerLaserCAN;

import frc.robot.subsystems.GenericLaserCANSubsystem.GenericLaserCANSubsystemIOImpl;

public class ClawRollerLaserCANIOReal extends GenericLaserCANSubsystemIOImpl
    implements ClawRollerLaserCANIO {

    public ClawRollerLaserCANIOReal()
    {
        super(ClawRollerLaserCANConstants.kSubSysConstants);
    }
}
