package frc.robot.subsystems.SampleProfiledRoller;

import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemIOImpl;

public class SampleProfiledRollerIOTalonFX extends GenericMotionProfiledSubsystemIOImpl
    implements SampleProfiledRollerIO {

    public SampleProfiledRollerIOTalonFX()
    {
        super(SampleProfiledRollerConstants.kSubSysConstants, false);
    }
}
