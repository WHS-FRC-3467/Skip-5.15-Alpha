package frc.robot.subsystems.ClawRoller;

import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemIOImpl;

public class ClawRollerIOSim extends GenericMotionProfiledSubsystemIOImpl
    implements ClawRollerIO {

    public ClawRollerIOSim()
    {
        super(ClawRollerConstants.kSubSysConstants, true);
    }
}
