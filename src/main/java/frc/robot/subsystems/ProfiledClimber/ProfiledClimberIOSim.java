package frc.robot.subsystems.ProfiledClimber;

import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemIOImpl;

public class ProfiledClimberIOSim extends GenericMotionProfiledSubsystemIOImpl
    implements ProfiledClimberIO {

  public ProfiledClimberIOSim() {
    super(ProfiledClimberConstants.kSubSysConstants, true);
  }
}
