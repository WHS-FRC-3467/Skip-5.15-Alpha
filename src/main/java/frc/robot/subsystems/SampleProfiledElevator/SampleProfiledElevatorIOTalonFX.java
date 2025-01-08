package frc.robot.subsystems.SampleProfiledElevator;

import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemIOImpl;

public class SampleProfiledElevatorIOTalonFX extends GenericMotionProfiledSubsystemIOImpl
    implements SampleProfiledElevatorIO {

  public SampleProfiledElevatorIOTalonFX() {
    super(SampleProfiledElevatorConstants.kSubSysConstants, false);
  }
}
