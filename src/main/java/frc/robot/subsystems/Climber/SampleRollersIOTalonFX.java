package frc.robot.subsystems.Climber;

import frc.robot.subsystems.GenericRollerSubsystem.GenericRollerSubsystemIOImpl;

public class SampleRollersIOTalonFX extends GenericRollerSubsystemIOImpl
    implements SampleRollersIO {

  public SampleRollersIOTalonFX() {
    super(SampleRollersConstants.kSubSysConstants, false);
  }
}
