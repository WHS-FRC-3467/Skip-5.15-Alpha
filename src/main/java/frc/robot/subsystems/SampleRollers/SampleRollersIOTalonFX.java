package frc.robot.subsystems.SampleRollers;

import frc.robot.subsystems.GenericRollerSubsystem.GenericRollerSubsystemIOImpl;

public class SampleRollersIOTalonFX extends GenericRollerSubsystemIOImpl
    implements SampleRollersIO {

  public SampleRollersIOTalonFX() {
    super(SampleRollersConstants.kSubSysConstants, false);
  }
}
