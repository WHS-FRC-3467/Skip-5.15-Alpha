package frc.robot.subsystems.ProfiledCoralRoller;

import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemIOImpl;

public class SampleProfiledRollerIOTalonFX extends GenericMotionProfiledSubsystemIOImpl
    implements ProfiledCoralRollerIO {

  public SampleProfiledRollerIOTalonFX() {
    super(ProfiledCoralRollerConstants.kSubSysConstants, false);
  }
}
