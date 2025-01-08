package frc.robot.subsystems.SampleProfiledArm;

import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemIOImpl;

public class SampleProfiledArmIOSim extends GenericMotionProfiledSubsystemIOImpl
    implements SampleProfiledArmIO {

  public SampleProfiledArmIOSim() {
    super(SampleProfiledArmConstants.kSubSysConstants, true);
  }
}
