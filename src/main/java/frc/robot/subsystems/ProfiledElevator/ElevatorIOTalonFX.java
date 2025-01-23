package frc.robot.subsystems.ProfiledElevator;

import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystemIOImpl;

public class ElevatorIOTalonFX extends GenericMotionProfiledSubsystemIOImpl implements ElevatorIO {

  public ElevatorIOTalonFX() {
    super(ElevatorConstants.kSubSysConstants, false);
  }
}
