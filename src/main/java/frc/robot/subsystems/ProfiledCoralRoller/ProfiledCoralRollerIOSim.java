package frc.robot.subsystems.ProfiledCoralRoller;

import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemIOImpl;

public class ProfiledCoralRollerIOSim extends GenericMotionProfiledSubsystemIOImpl
    implements ProfiledCoralRollerIO {

  public ProfiledCoralRollerIOSim() {
    super(ProfiledCoralRollerConstants.kSubSysConstants, true);
  }
}
