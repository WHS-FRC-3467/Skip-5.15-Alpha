package frc.robot.subsystems.SampleRollers;

import frc.robot.subsystems.GenericRollerSubsystem.GenericRollerSubsystemIOImpl;

public class SampleRollersIOSim extends GenericRollerSubsystemIOImpl implements SampleRollersIO {

    public SampleRollersIOSim()
    {
        super(SampleRollersConstants.kSubSysConstants, true);
    }
}
