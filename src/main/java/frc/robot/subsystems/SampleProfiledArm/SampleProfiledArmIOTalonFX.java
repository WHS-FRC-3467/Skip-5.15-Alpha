package frc.robot.subsystems.SampleProfiledArm;

import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemIOImpl;

public class SampleProfiledArmIOTalonFX extends GenericMotionProfiledSubsystemIOImpl
    implements SampleProfiledArmIO {

  public SampleProfiledArmIOTalonFX() {
    super(SampleProfiledArmConstants.kSubSysConstants, false);
  }
}
