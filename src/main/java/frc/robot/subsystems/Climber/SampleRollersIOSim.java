package frc.robot.subsystems.Climber;

import frc.robot.subsystems.GenericRollerSubsystem.GenericRollerSubsystemIOImpl;

public class SampleRollersIOSim extends GenericRollerSubsystemIOImpl implements SampleRollersIO {

  public SampleRollersIOSim() {
    super(SampleRollersConstants.kSubSysConstants, true);
  }
}
