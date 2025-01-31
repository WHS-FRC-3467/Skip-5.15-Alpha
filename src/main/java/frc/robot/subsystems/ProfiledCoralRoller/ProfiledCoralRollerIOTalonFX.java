package frc.robot.subsystems.ProfiledCoralRoller;

import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemIOImpl;

public class ProfiledCoralRollerIOTalonFX extends GenericMotionProfiledSubsystemIOImpl
    implements ProfiledCoralRollerIO {

    public ProfiledCoralRollerIOTalonFX()
    {
        super(ProfiledCoralRollerConstants.kSubSysConstants, false);
    }
}
