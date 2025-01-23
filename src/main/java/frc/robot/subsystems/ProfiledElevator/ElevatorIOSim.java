package frc.robot.subsystems.ProfiledElevator;

import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemIOImpl;

public class ElevatorIOSim extends GenericMotionProfiledSubsystemIOImpl implements ElevatorIO {

  public ElevatorIOSim() {
    super(ElevatorConstants.kSubSysConstants, true);
  }
}
