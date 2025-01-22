package frc.robot.subsystems.SampleProfiledRoller;

import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemIOImpl;

public class SampleProfiledRollerIOSim extends GenericMotionProfiledSubsystemIOImpl
    implements SampleProfiledRollerIO {

  public SampleProfiledRollerIOSim() {
    super(SampleProfiledRollerConstants.kSubSysConstants, true);
  }
}
