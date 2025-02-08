// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.LED;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.ClawRoller.ClawRoller;
import frc.robot.subsystems.ClawRoller.ClawRollerLaserCAN.ClawRollerLaserCAN;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Vision.Vision;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Ports;


public class LED extends SubsystemBase {

    // Driver controller to query
    private final CommandXboxController m_controller;
    private final GenericHID m_driveRmbl;

    // Subsystems to query
    Arm m_Arm;
    ClawRoller m_ClawRoller;
    Climber m_Climber;
    Drive m_Drive;
    Elevator m_Elevator;
    Vision m_Vision;
    ClawRollerLaserCAN m_clawLaserCAN;
    Trigger m_isCoralMode;
    
    // Control everything with a CANdle
    private static final CANdle m_candle = new CANdle(Ports.CANDLE.getDeviceNumber());

    // https://github.com/WHS-FRC-3467/Skip-5.14-Nocturne/blob/main/src/main/java/frc/robot/Subsystems/LED/LEDSubsystem.java

    // Create Triggers for each state
    // TODO: Figure out the priority level of driver feedback for each state
    private Trigger intakingTrigger;
    // private Trigger hasPieceInFunnel;
    private Trigger hasPieceInClaw;
    private Trigger climbingTrigger;
    private Trigger aimingTrigger;
    // private Trigger readyTrigger;
    private Trigger disabledTrigger = new Trigger(() -> DriverStation.isDisabled());
    private Trigger disabledTargetTrigger;
    private Trigger autonomousTrigger = new Trigger(() -> DriverStation.isAutonomousEnabled());

    /*
     * Constructor
     * Creates a new LEDSubsystem
     */
    public LED(CommandXboxController controller, Arm arm,
                        ClawRoller clawRoller,
                        Climber climber,
                        Drive drive,
                        Elevator elevator,
                        Vision vision,
                        ClawRollerLaserCAN laserCAN,
                        Trigger isCoralMode) {
        
        m_controller = controller;
        m_Arm = arm;
        m_ClawRoller = clawRoller;
        m_Climber = climber;
        m_Drive = drive;
        m_Elevator = elevator;
        m_Vision = vision;
        m_clawLaserCAN = laserCAN;
        m_isCoralMode = isCoralMode;

        // Create Triggers for each state
        // TODO: Figure out the priority level of driver feedback for each state
        intakingTrigger = new Trigger(() -> m_Arm.getState() == Arm.State.INTAKE || m_ClawRoller.getState() == ClawRoller.State.INTAKE); // and check to see if elevator and arm are at setpoint
        // hasPieceInFunnel = new Trigger(m_rampLaserCAN.getNearTrigger());
        hasPieceInClaw = new Trigger(intakingTrigger.and(m_clawLaserCAN.triggered));
        climbingTrigger = new Trigger(() -> m_Climber.getState() == Climber.State.PREP | m_Climber.getState() == Climber.State.CLIMB);
        aimingTrigger = new Trigger(() -> m_Drive.getCurrentCommand() != m_Drive.getDefaultCommand());
        // readyTrigger = new Trigger(() -> m_Elevator.io.isAtSetpoint() ); work on once there is a generic implementation
        disabledTargetTrigger = new Trigger(() -> DriverStation.isDisabled() && m_Vision.visionHasTarget);

        m_driveRmbl = m_controller.getHID();

        m_candle.configFactoryDefault();
        
        CANdleConfiguration candleConfiguration = new CANdleConfiguration();
        // m_candle.getAllConfigs(candleConfiguration);

        candleConfiguration.statusLedOffWhenActive = true;
        candleConfiguration.disableWhenLOS = false;
        candleConfiguration.stripType = LEDStripType.RGB;
        candleConfiguration.brightnessScalar = 0.5;
        candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(candleConfiguration, 100);

        // m_candle.getAllConfigs(candleConfiguration);

        // m_candle.configLEDType(LEDStripType.RGB, 300);

        // m_candle.getAllConfigs(candleConfiguration);

    }

    @Override
    public void periodic() {
        // Use the LEDStateMachine to set the LEDs
        LEDStateMachine();
    }

    private void LEDStateMachine() {
        // Light up the robot based on the triggers/state
        // Set the color of the mode LED strip based on Coral/Algae Mode
        m_isCoralMode.whileTrue(Commands.runOnce(() -> m_Mode.setColor(white)));
        m_isCoralMode.whileFalse(Commands.runOnce(() -> m_Mode.setColor(cyan)));

        /* --- List of Priorities ---
         * Disabled, Disabled with Target, and Autonomous never conflict
         * Has Piece in Claw has precedence over Intaking trigger
         * Then come intaking and climbing triggers
         * TODO: Make a (aiming and) ready trigger for coral/algae placement
         * Bottom priority is the aiming trigger
         */
        disabledTrigger.whileTrue(
            Commands.parallel(
                Commands.runOnce(() -> runMatchTimerPattern()),
                Commands.runOnce(() -> m_State.setAnimation(a_InAutonomous))));
        disabledTargetTrigger.whileTrue(Commands.runOnce(() -> {
            m_State.setAnimation(a_LeftIntakeRainbow);
            m_Mode.setAnimation(a_RightIntakeRainbow);
            this.timerDisabled();}));
        autonomousTrigger.whileTrue(Commands.runOnce(() -> {
            m_State.setAnimation(a_LeftFlame);
            m_Mode.setAnimation(a_RightFlame);
        }));
        intakingTrigger.and(()-> !m_clawLaserCAN.isTriggered()).whileTrue(
            Commands.runOnce(() -> m_State.setColor(red)));
        hasPieceInClaw.whileTrue(
                Commands.runOnce(() -> {m_State.setColor(green); // Tells the driver that they can back away from the coral station
                    m_driveRmbl.setRumble(GenericHID.RumbleType.kLeftRumble, 1);}));
        climbingTrigger.and(m_Climber.getClimbedTrigger().negate()).whileTrue(
            Commands.runOnce(() -> m_State.setColor(magenta)));
        aimingTrigger.and(intakingTrigger.negate()).and(climbingTrigger.negate()).whileTrue(
            Commands.runOnce(() -> m_State.setAnimation(a_AimingPingPong)));
        // TODO: Ready Trigger
        // When robot is successfully auto aligned, set one LED to green, the other to the coral/algae and rumble the controller
    }
    
    /*
     * Colors
     */
    class Color {
        int r, g, b;

        private Color(int red, int green, int blue) {
            this.r = red; this.g = green; this.b = blue;
        }
    }

    Color black = new Color(0, 0, 0); // This will Turn off the CANdle
    Color white = new Color(255, 255, 255);
    Color red = new Color(255, 0, 0);
    Color green = new Color(0, 255, 0);
    Color blue = new Color(0, 0, 255);
    Color yellow = new Color(255, 255, 0);
    Color cyan = new Color(0, 255, 255);
    Color magenta = new Color(255, 0, 255);
    
    /*
     * LED Segments
     */
    class LEDSegment {

        int startIndex;
        int segmentSize;
        int animationSlot;

        private LEDSegment(int startIndex, int segmentSize, int animationSlot) {
            this.startIndex = startIndex;
            this.segmentSize = segmentSize;
            this.animationSlot = animationSlot;
        }

        public void setColor(Color color) {
            m_candle.clearAnimation(animationSlot);
            m_candle.setLEDs(color.r, color.g, color.b, 0, startIndex, segmentSize);
            m_candle.modulateVBatOutput(0.95);
        }

        private void setAnimation(Animation animation) {
            m_candle.clearAnimation(animationSlot);
            m_candle.animate(animation, animationSlot);
            m_candle.modulateVBatOutput(0.95);
        }

        public void setOff() {
            m_candle.clearAnimation(animationSlot);
            m_candle.setLEDs(0, 0, 0, 0, startIndex, segmentSize);
            m_candle.modulateVBatOutput(0.0);

        }

    }

   /* Match Timer Strip
    * Autonomous (15 sec): Yellow Ping-pong
    * 2:15 -> 1:00: Solid Green
    * 1:00 -> 0:20: Solid Yellow
    * 0:20 -> 0:10: Solid Red
    * 0:10 -> 0:00: Strobing Red
    * Non-auto periods & Disabled: White
    */
    Timer m_pseudoTimer = new Timer();
    Color currentColor = black;

    // There will be two LED strips
    // This one for Coral/Algae Mode
    LEDSegment m_Mode = new LEDSegment(8, 47, 1);
    // This one for what the robot is doing
    LEDSegment m_State = new LEDSegment(55, 47, 2);

    // Animations
    Animation a_TimeExpiring = new StrobeAnimation(red.r, red.g, red.b, 0, 0.5, m_Mode.segmentSize, m_Mode.startIndex);
    Animation a_LeftIntakeRainbow = new RainbowAnimation(0.7, 0.5, m_State.segmentSize, false, m_State.startIndex);
    Animation a_RightIntakeRainbow = new RainbowAnimation(0.7, 0.5, m_Mode.segmentSize, false, m_Mode.startIndex);
    Animation a_AimingPingPong = new LarsonAnimation(green.r, green.g, green.b, 0, 0.8, m_State.segmentSize, BounceMode.Back, 6, m_State.startIndex);
    Animation a_InAutonomous = new LarsonAnimation(yellow.r, yellow.g, yellow.b, 0, 0.8, m_Mode.segmentSize, BounceMode.Back, 3, m_Mode.startIndex);
    Animation a_LeftFlame = new FireAnimation(0.9, 0.75, m_State.segmentSize, 1.0, 0.3, true, m_State.startIndex);
    Animation a_RightFlame = new FireAnimation(0.9, 0.75, m_Mode.segmentSize, 1.0, 0.3, false, m_Mode.startIndex);

    private void runMatchTimerPattern() {

        Color newColor = black;

        double matchTime = DriverStation.getMatchTime();
        if (matchTime < 0.0) {
            m_pseudoTimer.start();            
            matchTime = (int) (150.0 - m_pseudoTimer.get());
        }

        if (matchTime > 60.0) {
            newColor = green;
        } else if (matchTime > 20.0) {
            newColor = yellow;
        } else if (matchTime > 10.0) {
            newColor = red;
        } else if (matchTime > 0.0) {
            newColor = magenta;
        } else {
            newColor = white;
        }

        if (newColor != currentColor) {
            if (newColor == magenta) {
                m_Mode.setAnimation(a_TimeExpiring);
            } else {
                m_Mode.setColor(newColor);
            }
            currentColor = newColor;
        }
    }

    public void timerDisabled() {
        m_Mode.setColor(white);
        m_pseudoTimer.stop();
        m_pseudoTimer.reset();
    }
}
