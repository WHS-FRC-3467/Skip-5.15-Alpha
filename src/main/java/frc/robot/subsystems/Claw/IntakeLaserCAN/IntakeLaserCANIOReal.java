package frc.robot.subsystems.Claw.IntakeLaserCAN;

import frc.robot.subsystems.GenericLaserCANSubsystem.GenericLaserCANSubsystemIOImpl;

public class IntakeLaserCANIOReal extends GenericLaserCANSubsystemIOImpl
    implements IntakeLaserCANIO {

    public IntakeLaserCANIOReal()
    {
        super(IntakeLaserCANConstants.kSubSysConstants, false);
    }
}
