package frc.robot.subsystems.SampleProfiledElevator;

import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemIOImpl;

public class SampleProfiledElevatorIOSim extends GenericMotionProfiledSubsystemIOImpl
    implements SampleProfiledElevatorIO {

  public SampleProfiledElevatorIOSim() {
    super(SampleProfiledElevatorConstants.kSubSysConstants, true);
  }
}
