package frc.robot.subsystems.Claw.IntakeLaserCAN;

import frc.robot.subsystems.GenericLaserCANSubsystem.GenericLaserCANSubsystemIOImpl;

public class IntakeLaserCANIOSim extends GenericLaserCANSubsystemIOImpl
    implements IntakeLaserCANIO {

    public IntakeLaserCANIOSim()
    {
        super(IntakeLaserCANConstants.kSubSysConstants, true);
    }
}
