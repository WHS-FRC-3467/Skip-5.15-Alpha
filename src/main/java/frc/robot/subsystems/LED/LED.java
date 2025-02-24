// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.LED;

import org.littletonrobotics.junction.Logger;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    // Sim strings
    // String modeSim = "";
    // String stateSim = "";
    String driveCommand = "";

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
        // Smartdashboard the LEDs for Sim
        // SmartDashboard.putString("State LED", stateSim);
        // SmartDashboard.putString("Mode LED", modeSim);
        try {
            driveCommand = m_Drive.getCurrentCommand().getClass().toString();
        } catch (Exception e) {
            driveCommand = "";
        }
        Logger.recordOutput("Current Drive Command", driveCommand);
    }

    private void LEDStateMachine()
    {
        // Light up the robot based on the triggers/state
        // Set the color of the mode LED strip based on Coral/Algae Mode
        if (m_isCoralMode.getAsBoolean()) {
            m_Mode.setColor(white);
            // modeSim = "Coral";
        } else {
            m_Mode.setColor(cyan);
            // modeSim = "Algae";
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
                // stateSim = "Left Elevator Rainbow";
                // modeSim = "Right Elevator Rainbow";
            } else {
                m_Mode.setAnimation(a_DisabledMode);
                m_State.setAnimation(a_DisabledState);
                // stateSim = "Disabled State";
                // modeSim = "Disabled Mode";
            }
        } else if (DriverStation.isAutonomousEnabled()) {
            m_State.setAnimation(a_LeftFlame);
            m_Mode.setAnimation(a_RightFlame);
            // stateSim = "Auto - State Flame";
            // modeSim = "Auto - Mode Flame";
        } else {
            // All the teleop states/logic comes here
            if (m_ClawRoller.getState() == ClawRoller.State.ALGAE_INTAKE || m_ClawRoller.getState() == ClawRoller.State.INTAKE
                || m_ClawRoller.getState() == ClawRoller.State.INTAKESLOW) {
                if (m_clawLaserCAN.isTriggered() || m_ClawRoller.getState() == ClawRoller.State.INTAKESLOW) {
                    // Checks to see if robot is intaking and coral is in robot
                    // If so, alert driver that coral is being intaked
                    m_State.setColor(blue);
                    // stateSim = "Intaking in Progress - State Blue";
                } else {
                    // Intaking but no piece detected, so set the state LED to red
                    m_State.setAnimation(a_FlashRed);
                    // stateSim = "Intake - State Red";
                }
            } else if (m_Superstructure.getArmState() == Arm.State.CLIMB || m_Climber.getState() == Climber.State.PREP
            || m_Climber.getState() == Climber.State.CLIMB) {
                // If climb is complete, set state LED to green
                if (m_Climber.atPosition(0.1) && m_Climber.getState() == Climber.State.CLIMB) {
                    m_State.setColor(green);
                    // stateSim = "Climb - State Green";
                } else {
                    // If robot is still climbing, then set the state LED red
                    m_State.setAnimation(a_FlashRed);
                    // stateSim = "Climb - State Red";
                }
            } else if (m_Superstructure.getElevatorState() == Elevator.State.LEVEL_1 || m_Superstructure.getElevatorState() == Elevator.State.LEVEL_2 
                || m_Superstructure.getElevatorState() == Elevator.State.LEVEL_3 || m_Superstructure.getElevatorState() == Elevator.State.LEVEL_4
                || m_Superstructure.getElevatorState() == Elevator.State.ALGAE_SCORE) {
                // The above statement says that the robot's superstructure is going to a state if the arm isn't in the above states
                // This superstructure part of LED control may be deleted if the driver changes his mind about keeping this
                    // When robot is successfully auto aligned, set one LED to green, the other to the
                    // coral/algae and rumble the controller
                if (m_Superstructure.atPosition(0.1, 0.1) && m_ClawRoller.atPosition(0.1)) {
                    m_State.setColor(green);
                    // stateSim = "Superstructure - State Green";
                } else {
                    // Else: the robot is aiming, set state LED to aiming ping pong
                    // The robot is aiming if the drivetrain isn't in its default command
                    // which suggests that the robot is auto aligning to the reef or coral station
                    // This else if being underneath intaking and climbing 
                    // means that an intaking or climbing robot will never reach this statement
                    m_State.setAnimation(a_AimingPingPong);
                    // stateSim = "Superstructure - State Red";
                }
            } else if (m_ClawRoller.getState() == ClawRoller.State.HOLDCORAL) {
                // Intaking: Coral is firmly in  If so, tell driver to back away from coral station
                m_State.setColor(green);
                // stateSim = "Intake - State Green";

            } else {
                try {
                    if (m_Drive.getCurrentCommand().getClass() == edu.wpi.first.wpilibj2.command.SequentialCommandGroup.class) {
                        // The robot is also aiming if the drivetrain isn't in its default command,
                        // assuming that the robot has finished aiming
                        m_State.setAnimation(a_AimingPingPong);
                        // stateSim = "Drivetrain is Aiming At something";
                    } else {
                        // Default Teleop
                        // stateSim = "Default Teleop";
                        m_State.setColor(black);
                    }
                } catch (Exception e) {
                    // Default Teleop
                    // stateSim = "Default Teleop";
                    m_State.setColor(black);
                }

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

    // There will be two LED strips
    // This one for Coral/Algae Mode
    LEDSegment m_Mode = new LEDSegment(8, 144, 1);
    // This one for what the robot is doing
    LEDSegment m_State = new LEDSegment(152, 144, 2);

    // Animations
    Animation a_FlashRed =
        new StrobeAnimation(red.r, red.g, red.b, 0, 0.5, m_Mode.segmentSize, m_Mode.startIndex);
    Animation a_LeftElevatorRainbow =
        new RainbowAnimation(0.7, 0.5, m_State.segmentSize, false, m_State.startIndex);
    Animation a_RightElevatorRainbow =
        new RainbowAnimation(0.7, 0.5, m_Mode.segmentSize, false, m_Mode.startIndex);
    Animation a_AimingPingPong = new StrobeAnimation(magenta.r, magenta.g, magenta.b, 0, 0.75, m_State.segmentSize, m_State.startIndex);
    Animation a_FacingPingPong = new LarsonAnimation(green.r, green.g, green.b, 0, 0.8,
    m_State.segmentSize, BounceMode.Back, 6, m_State.startIndex);
    Animation a_DisabledMode = new LarsonAnimation(yellow.r, yellow.g, yellow.b, 0, 0.8,
        m_Mode.segmentSize, BounceMode.Back, 3, m_Mode.startIndex);
    Animation a_DisabledState = new LarsonAnimation(yellow.r, yellow.g, yellow.b, 0, 0.8,
        m_State.segmentSize, BounceMode.Back, 3, m_State.startIndex);
    Animation a_LeftFlame =
        new FireAnimation(0.9, 0.75, m_State.segmentSize, 1.0, 0.3, true, m_State.startIndex);
    Animation a_RightFlame =
        new FireAnimation(0.9, 0.75, m_Mode.segmentSize, 1.0, 0.3, false, m_Mode.startIndex);

}
