// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.LED;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Claw.ClawRoller.ClawRoller;
import frc.robot.subsystems.Claw.ClawRollerLaserCAN.ClawRollerLaserCAN;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Vision.Vision;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Ports;


public class LED extends SubsystemBase {

    // Subsystems to query
    ClawRoller m_ClawRoller;
    Climber m_Climber;
    Drive m_Drive;
    Superstructure m_Superstructure;
    Vision m_Vision;
    ClawRollerLaserCAN m_clawLaserCAN;
    Trigger m_isCoralMode;

    // Control everything with a CANdle
    private static final CANdle m_candle = new CANdle(Ports.ELEVATOR_CANDLE.getDeviceNumber());

    // https://github.com/WHS-FRC-3467/Skip-5.14-Nocturne/blob/main/src/main/java/frc/robot/Subsystems/LED/LEDSubsystem.java


    /*
     * Constructor Creates a new LEDSubsystem
     */
    public LED(
        ClawRoller clawRoller,
        Climber climber,
        Drive drive,
        Superstructure superstructure,
        Vision vision,
        ClawRollerLaserCAN laserCAN,
        Trigger isCoralMode)
    {

        m_ClawRoller = clawRoller;
        m_Climber = climber;
        m_Drive = drive;
        m_Superstructure = superstructure;
        m_Vision = vision;
        m_clawLaserCAN = laserCAN;
        m_isCoralMode = isCoralMode;

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
    public void periodic()
    {
        // Use the LEDStateMachine to set the LEDs
        LEDStateMachine();
    }

    private void LEDStateMachine()
    {
        // Light up the robot based on the triggers/state
        // Set the color of the mode LED strip based on Coral/Algae Mode
        if (m_isCoralMode.getAsBoolean()) {
            m_Mode.setColor(cyan);
        } else {
            m_Mode.setColor(white);
        }

        /*
         * --- List of Priorities --- Disabled, Disabled with Target, and Autonomous never conflict
         * Has Piece in Claw has precedence over Intaking trigger 
         * Then come intaking and climbing triggers 
         * At the end comes aiming:
         * A to-be-implemented ready condition for coral/algae placement 
         * Bottom priority is the aiming and not ready condition
         */

        // Elevator LED Rainbow if disabled and vision has target
        if (DriverStation.isDisabled()) {
            if (m_Vision.visionHasTarget) {
                m_State.setAnimation(a_LeftElevatorRainbow);
                m_Mode.setAnimation(a_RightElevatorRainbow);
                this.timerDisabled();
            } else {
                runMatchTimerPattern();
                m_State.setAnimation(a_InAutonomous);
            }
        } else if (DriverStation.isAutonomousEnabled()) {
            m_State.setAnimation(a_LeftFlame);
            m_Mode.setAnimation(a_RightFlame);
        } else {
            // All the teleop states/logic comes here
            if (m_Superstructure.getArmState() == Arm.State.CORAL_INTAKE
            || m_ClawRoller.getState() == ClawRoller.State.INTAKE) {
                if (m_clawLaserCAN.isTriggered()) {
                    // Checks to see if intaking is complete
                    // If so, tell driver to back away from coral station
                    m_State.setColor(green);
                } else {
                    // When intaking, set the state LED to red
                    m_State.setColor(red);               
                }
            } else if (m_Superstructure.getArmState() == Arm.State.CLIMB || m_Climber.getState() == Climber.State.PREP
            | m_Climber.getState() == Climber.State.CLIMB) {
                // TODO: if climb is complete, set state LED to green
                if (m_Climber.atPosition(0.1)) {
                    m_State.setColor(green);
                } else {
                    // If robot is still climbing, then set the state LED to magenta
                    m_State.setColor(magenta);
                }
             } else if (m_Superstructure.getElevatorState() != Elevator.State.STOW && m_Superstructure.getElevatorState() != Elevator.State.CHARACTERIZATION 
                && m_Superstructure.getElevatorState() != Elevator.State.CLIMB && m_Superstructure.getElevatorState() != Elevator.State.HOMING) {
                // The above statement says that the robot is aiming or has finished aiming if the arm isn't in the above states
                    // TODO: Robot is Ready If Statement
                        // When robot is successfully auto aligned, set one LED to green, the other to the
                        // coral/algae and rumble the controller
                if (m_Superstructure.atPosition(0.1, 0.1) && m_ClawRoller.atPosition(0.1)) {
                    m_State.setColor(green);
                } else {
                    // Else: the robot is aiming, set state LED to aiming ping pong
                    // The robot is aiming if the drivetrain isn't in its default command
                    // which suggests that the robot is auto aligning to the reef or coral station
                    // This else if being underneath intaking and climbing 
                    // means that an intaking or climbing robot will never reach this statement
                    m_State.setAnimation(a_AimingPingPong);
                }

            } else if (m_Drive.getCurrentCommand() != m_Drive.getDefaultCommand()) {
                // The robot is also aiming if the drivetrain isn't in its default command,
                // assuming that the robot has finished aiming
                m_State.setAnimation(a_AimingPingPong);
            }
        }
    }

    /*
     * Colors
     */
    class Color {
        int r, g, b;

        private Color(int red, int green, int blue)
        {
            this.r = red;
            this.g = green;
            this.b = blue;
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

        private LEDSegment(int startIndex, int segmentSize, int animationSlot)
        {
            this.startIndex = startIndex;
            this.segmentSize = segmentSize;
            this.animationSlot = animationSlot;
        }

        public void setColor(Color color)
        {
            m_candle.clearAnimation(animationSlot);
            m_candle.setLEDs(color.r, color.g, color.b, 0, startIndex, segmentSize);
            m_candle.modulateVBatOutput(0.95);
        }

        private void setAnimation(Animation animation)
        {
            m_candle.clearAnimation(animationSlot);
            m_candle.animate(animation, animationSlot);
            m_candle.modulateVBatOutput(0.95);
        }

        public void setOff()
        {
            m_candle.clearAnimation(animationSlot);
            m_candle.setLEDs(0, 0, 0, 0, startIndex, segmentSize);
            m_candle.modulateVBatOutput(0.0);

        }

    }

    /*
     * Match Timer Strip Autonomous (15 sec): Yellow Ping-pong 2:15 -> 1:00: Solid Green 1:00 ->
     * 0:20: Solid Yellow 0:20 -> 0:10: Solid Red 0:10 -> 0:00: Strobing Red Non-auto periods &
     * Disabled: White
     */
    Timer m_pseudoTimer = new Timer();
    Color currentColor = black;

    // There will be two LED strips
    // This one for Coral/Algae Mode
    LEDSegment m_Mode = new LEDSegment(8, 47, 1);
    // This one for what the robot is doing
    LEDSegment m_State = new LEDSegment(55, 47, 2);

    // Animations
    Animation a_TimeExpiring =
        new StrobeAnimation(red.r, red.g, red.b, 0, 0.5, m_Mode.segmentSize, m_Mode.startIndex);
    Animation a_LeftElevatorRainbow =
        new RainbowAnimation(0.7, 0.5, m_State.segmentSize, false, m_State.startIndex);
    Animation a_RightElevatorRainbow =
        new RainbowAnimation(0.7, 0.5, m_Mode.segmentSize, false, m_Mode.startIndex);
    Animation a_AimingPingPong = new LarsonAnimation(green.r, green.g, green.b, 0, 0.8,
        m_State.segmentSize, BounceMode.Back, 6, m_State.startIndex);
    Animation a_InAutonomous = new LarsonAnimation(yellow.r, yellow.g, yellow.b, 0, 0.8,
        m_Mode.segmentSize, BounceMode.Back, 3, m_Mode.startIndex);
    Animation a_LeftFlame =
        new FireAnimation(0.9, 0.75, m_State.segmentSize, 1.0, 0.3, true, m_State.startIndex);
    Animation a_RightFlame =
        new FireAnimation(0.9, 0.75, m_Mode.segmentSize, 1.0, 0.3, false, m_Mode.startIndex);

    private void runMatchTimerPattern()
    {

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

    public void timerDisabled()
    {
        m_Mode.setColor(white);
        m_pseudoTimer.stop();
        m_pseudoTimer.reset();
    }
}
