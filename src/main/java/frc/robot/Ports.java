package frc.robot;

import frc.robot.util.drivers.CanDeviceId;

public class Ports {
  /*
   * LIST OF CHANNEL AND CAN IDS
   */

  /* DRIVETRAIN CAN DEVICE IDS */
  // public static final CanDeviceId FL_DRIVE = new CanDeviceId(0, "canivore1");
  // public static final CanDeviceId FL_ROTATION = new CanDeviceId(1, "canivore1");
  // public static final CanDeviceId FL_CANCODER = new CanDeviceId(0, "canivore1");

  /* SUBSYSTEM CAN DEVICE IDS */
  public static final CanDeviceId SAMPLE_ROLLER = new CanDeviceId(15, "rio");

  public static final CanDeviceId TWO_ROLLER_1 = new CanDeviceId(16, "rio");
  public static final CanDeviceId TWO_ROLLER_2 = new CanDeviceId(17, "rio");

  public static final CanDeviceId ELEVATOR_MAIN = new CanDeviceId(18, "rio");
  public static final CanDeviceId ELEVATOR_FOLLOWER = new CanDeviceId(19, "rio");
  public static final CanDeviceId ELEVATOR_CANCODER = new CanDeviceId(20, "rio");

  public static final CanDeviceId ARM_MAIN = new CanDeviceId(21, "rio");
  public static final CanDeviceId ARM_FOLLOWER = new CanDeviceId(22, "rio");
  public static final CanDeviceId ARM_CANCODER = new CanDeviceId(23, "rio");

  public static final CanDeviceId CLIMBER = new CanDeviceId(24, "rio");

  /* BEAM BREAK DIO CHANNELS*/
  /*
  	public static final int SERIALIZER_BREAK = 7;
  	public static final int FEEDER_BREAK = 8;
  	public static final int AMP_BREAK = 9;
  */

}
