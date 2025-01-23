package frc.robot.subsystems.ProfiledClimber;

import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemIOImpl;

public class ProfiledClimberIOTalonFX extends GenericMotionProfiledSubsystemIOImpl
    implements ProfiledClimberIO {

  public ProfiledClimberIOTalonFX() {
    super(ProfiledClimberConstants.kSubSysConstants, false);
  }
}
