package frc.robot.subsystems.LED;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Claw.ClawRoller.ClawRoller;
import frc.robot.subsystems.Claw.ClawRollerLaserCAN.ClawRollerLaserCAN;
import frc.robot.subsystems.Claw.IntakeLaserCAN.IntakeLaserCAN;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.util.LoggedTunableNumber;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Ports;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveCommands.DriveMode;

public class LEDSubsystem extends SubsystemBase {

    // Subsystems to query
    ClawRoller m_ClawRoller;
    Arm m_Arm;
    Elevator m_Elevator;
    Climber m_Climber;
    Vision m_Vision;
    ClawRollerLaserCAN m_clawLaserCAN;
    IntakeLaserCAN m_intakeLaserCAN;
    Trigger m_isCoralMode;

    // Control everything with a CANdle
    private static final CANdle m_candle = new CANdle(Ports.ELEVATOR_CANDLE.getDeviceNumber());

    // LoggedTunableNumbers for testing LED states
    private LoggedTunableNumber kMode, kState;
    // Flag for testing mode
    boolean kTesting = true;

    Alert ledConfigError = new Alert("LED Configuration Error!", Alert.AlertType.kWarning);

    /*
     * Robot LED States
     */
    private static enum LEDState {
        START,
        DISABLED,
        DISABLED_TARGET,
        AUTONOMOUS,
        INTAKING,
        FEEDING,
        CLIMBING,
        CLIMBED,
        SUPER_MOVE,
        ALIGNING,
        HAVE_CORAL,
        ENABLED
    }

    // Game Piece Mode
    private static enum GPMode {
        START,
        CORAL,
        ALGAE
    }

    LEDState m_currentState = LEDState.START;
    GPMode m_currentGPMode = GPMode.START;

    /*
     * Constructor Creates a new LEDSubsystem
     */
    public LEDSubsystem(
        ClawRoller clawRoller,
        Arm arm,
        Elevator elevator,
        Climber climber,
        Vision vision,
        ClawRollerLaserCAN clawLaserCAN,
        IntakeLaserCAN intakeLaserCAN,
        Trigger isCoralMode)
    {

        m_ClawRoller = clawRoller;
        m_Arm = arm;
        m_Elevator = elevator;
        m_Climber = climber;
        m_Vision = vision;
        m_clawLaserCAN = clawLaserCAN;
        m_intakeLaserCAN = intakeLaserCAN;
        m_isCoralMode = isCoralMode;

        m_candle.configFactoryDefault();

        CANdleConfiguration candleConfiguration = new CANdleConfiguration();
        // m_candle.getAllConfigs(candleConfiguration);

        candleConfiguration.statusLedOffWhenActive = true;
        candleConfiguration.disableWhenLOS = false;
        candleConfiguration.stripType = LEDStripType.RGB;
        candleConfiguration.brightnessScalar = 0.5;
        candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
        candleConfiguration.v5Enabled = false;
        ErrorCode ec = m_candle.configAllSettings(candleConfiguration, 100);
        if (ec != ErrorCode.OK) {
            ledConfigError.set(true);
            ledConfigError.setText(ec.toString());
        }

        // StripType setting needs to be done twice (for some reason, once doesn't work)
        ec = m_candle.configLEDType(LEDStripType.RGB, 300);
        if (ec != ErrorCode.OK) {
            ledConfigError.set(true);
            ledConfigError.setText(ec.toString());
        }

        // Tunable numbers for testing
        kMode = new LoggedTunableNumber("LED/Mode", 0);
        kState = new LoggedTunableNumber("LED/State", 0);
    }

    @Override
    public void periodic()
    {
        LEDState newState;
        GPMode newGPMode;

        if (kTesting) {
            // Testing Mode - change values using Tunable Numbers
            newState = testLEDState((int) kState.get());
            newGPMode = kMode.get() == 0 ? GPMode.CORAL : GPMode.ALGAE;

            if (newState == LEDState.START) {
                runMatchTimerPattern();
            } else {
                timerDisabled();
            }

        } else {
            // Real Robot
            // Determine Game Piece Mode
            if (m_isCoralMode.getAsBoolean()) {
                newGPMode = GPMode.CORAL;
            } else {
                newGPMode = GPMode.ALGAE;
            }
            // Get latest robot state
            newState = getRobotState();
        }

        // Check for changes & process them
        if ((newGPMode != m_currentGPMode) || (newState != m_currentState)) {
            LEDStateMachine(newGPMode, newState);
            m_currentGPMode = newGPMode;
            m_currentState = newState;
        }
    }

    // Determine the current state of the robot
    private LEDState getRobotState()
    {
        // --- Order of Priorities ---
        // Whole Robot:
        // - DISABLED -> Alliance color Larson
        // - DISABLED_TARGET -> Rainbow
        // - AUTONOMOUS -> Flames
        // Mode:
        // - GPMode -> Tips: White or "algae" color
        // State:
        // - INTAKING -> Red Flash Fast
        // - FEEDING -> Blue
        // - CLIMBING -> Red Flash Slow
        // - CLIMBED -> Green
        // - SUPER_MOVE -> Magenta Flash Medium
        // - ALIGNING -> Cyan Flash Medium
        // - HAVE_CORAL -> Green
        // - ENABLED -> Yellow SingleFade Fast

        LEDState newState = LEDState.START;

        // Determine state of robot to be displayed
        if (DriverStation.isDisabled()) {
            // Disabled patterns are different depending if
            // there is a target (AprilTag) in view
            if (m_Vision.visionHasTarget) {
                newState = LEDState.DISABLED_TARGET;
            } else {
                newState = LEDState.DISABLED;
            }

        } else if (DriverStation.isAutonomousEnabled()) {
            // In Autonomous mode
            newState = LEDState.AUTONOMOUS;

        } else {
            // If not Disabled or in Auto, determine robot state

            // Run MatchTimer
            runMatchTimerPattern();

            // Intaking Coral?
            if (m_ClawRoller.getState() == ClawRoller.State.INTAKE) {
                if (m_intakeLaserCAN.isTriggered()) {
                    // Coral has entered and is being positioned
                    newState = LEDState.FEEDING;
                } else {
                    // Waiting for Coral
                    newState = LEDState.INTAKING;
                }

                // Climbing?
            } else if (m_Climber.getState() == Climber.State.PREP ||
                m_Climber.getState() == Climber.State.CLIMB) {
                // Climb complete?
                if (m_Climber.atPosition(0.1)) {
                    newState = LEDState.CLIMBED;
                } else {
                    newState = LEDState.CLIMBING;
                }

                // Moving Superstructure?
            } else if (m_Elevator.isElevated()) {
                if (!m_Elevator.atPosition(0.0) || !m_Arm.atPosition(0.0)) {
                    // An Elevated position has been commanded, but it's not there yet
                    newState = LEDState.SUPER_MOVE;
                }

                // Aligning?
            } else if (DriveCommands.getDriveMode() == DriveMode.dmApproach) {
                // The robot is auto-aligning
                newState = LEDState.ALIGNING;

                // Holding Coral?
            } else if (m_ClawRoller.getState() == ClawRoller.State.HOLDCORAL) {
                // Claw is holding Coral
                newState = LEDState.HAVE_CORAL;

            } else {
                // Default state: Just Enabled
                newState = LEDState.ENABLED;
            }
        }

        return newState;
    }

    // Set the Mode and State indicator LEDS
    private void LEDStateMachine(GPMode newGPMode, LEDState newState)
    {
        // If State has changed, clean up old colors or animations
        if (newState != m_currentState) {
            switch (m_currentState) {
                case DISABLED:
                case DISABLED_TARGET:
                case AUTONOMOUS:
                    switch (newState) {
                        case DISABLED:
                        case DISABLED_TARGET:
                        case AUTONOMOUS:
                            // No change required
                            break;
                        default:
                            // Need to clean up
                            m_FullLeft.setOff();
                            m_FullRight.setOff();
                            break;
                    }
                    break;

                default:
                    switch (newState) {
                        case DISABLED:
                        case DISABLED_TARGET:
                        case AUTONOMOUS:
                            // Need to clean up
                            m_LeftTip.setOff();
                            m_RightTip.setOff();
                            m_State.setOff();
                            break;
                        default:
                            // No change required
                            break;
                    }
                    break;
            }
        }

        // Process and make changes for changed LEDState
        if (newState != m_currentState) {
            switch (newState) {
                case START:
                    // Test mode only
                    m_LeftTip.setColor(red);
                    m_RightTip.setColor(blue);
                    m_State.setColor(green);
                    break;

                case DISABLED:
                    if (DriverStation.getAlliance().isPresent()) {
                        if (DriverStation.getAlliance().get() == Alliance.Blue) {
                            // m_FullLeft.setAnimation(a_LeftBlueLarson);
                            // m_FullRight.setAnimation(a_RightBlueLarson);
                            m_FullLeft.setColor(blue);
                            m_FullRight.setColor(blue);
                        } else {
                            // m_FullLeft.setAnimation(a_LeftRedLarson);
                            // m_FullRight.setAnimation(a_RightRedLarson);
                            m_FullLeft.setColor(red);
                            m_FullRight.setColor(red);
                        }
                    } else {
                        m_FullLeft.setColor(purple);
                        m_FullRight.setColor(purple);
                    }

                    this.timerDisabled();
                    break;

                case DISABLED_TARGET:
                    m_FullLeft.setAnimation(a_LeftRainbow);
                    m_FullRight.setAnimation(a_RightRainbow);
                    this.timerDisabled();
                    break;

                case AUTONOMOUS:
                    m_FullLeft.setAnimation(a_LeftFlame);
                    m_FullRight.setAnimation(a_RightFlame);
                    m_MatchTime.setAnimation(a_InAutonomous);
                    break;

                case INTAKING:
                    // m_State.setAnimation(a_FastFlashRed);
                    m_State.setColor(red);
                    break;

                case FEEDING:
                    m_State.setColor(blue);
                    break;

                case CLIMBING:
                    m_State.setAnimation(a_SlowFlashRed);
                    break;

                case CLIMBED:
                    m_State.setColor(green);
                    break;

                case SUPER_MOVE:
                    m_State.setAnimation(a_MedFlashMagenta);
                    break;

                case ALIGNING:
                    m_State.setAnimation(a_MedFlashCyan);
                    break;

                case HAVE_CORAL:
                    m_State.setColor(green);
                    break;

                case ENABLED:
                    // m_State.setAnimation(a_SingleFadeFastYellow);
                    m_State.setColor(yellow);
                    break;

                default:
                    break;
            }
        }

        // Process and make changes for changed GPMode
        switch (newState) {
            case DISABLED:
            case DISABLED_TARGET:
            case AUTONOMOUS:
                // Mode is not displayed in these cases
                // so just break out
                break;
            default:
                switch (m_currentState) {
                    case DISABLED:
                    case DISABLED_TARGET:
                    case AUTONOMOUS:
                        // Have to set GPMode in these cases
                        // regardless of if it changed
                        switchGPMode(newGPMode);
                        break;
                    default:
                        if (newGPMode != m_currentGPMode)
                            switchGPMode(newGPMode);
                        break;
                }
                break;
        }
    }

    private void switchGPMode(GPMode newMode)
    {
        switch (newMode) {
            case ALGAE:
                m_LeftTip.setColor(green);
                m_RightTip.setColor(green);
                break;
            case CORAL:
            default:
                m_LeftTip.setColor(white);
                m_RightTip.setColor(white);
                break;
        }
    }

    public void setBrightness(double percent)
    {
        /* Here we will set the brightness of the LEDs */
        m_candle.configBrightnessScalar(percent, 100);
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
    Color algae = new Color(52, 235, 113);
    Color purple = new Color(125, 30, 165);

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

    // Define LED Segments
    // Numbering Sequence: Right LEDs go down, Left LEDs go Up
    // Match Time Indicator - CANdle module
    LEDSegment m_MatchTime = new LEDSegment(0, 8, 0);
    // These are for Disabled/Auto states
    // They are each a full strip
    LEDSegment m_FullRight = new LEDSegment(8, 90, 1);
    LEDSegment m_FullLeft = new LEDSegment(98, 90, 2);
    // These are for Coral/Algae Mode while robot is Enabled
    // These are the top pixels on each strip
    LEDSegment m_RightTip = new LEDSegment(8, 20, 3);
    LEDSegment m_LeftTip = new LEDSegment(168, 20, 4);
    // This is for what the robot is doing while robot is Enabled
    // This is the bottom of each strip combined into one segment
    LEDSegment m_State = new LEDSegment(28, 148, 5);

    // Disabled Animations
    Animation a_RightRedLarson =
        new LarsonAnimation(red.r, red.g, red.b, 0, 0.2, m_FullRight.segmentSize, BounceMode.Front,
            10, m_FullRight.startIndex);
    Animation a_LeftRedLarson =
        new LarsonAnimation(red.r, red.g, red.b, 0, 0.2, m_FullLeft.segmentSize, BounceMode.Front,
            10, m_FullLeft.startIndex);
    Animation a_RightBlueLarson =
        new LarsonAnimation(blue.r, blue.g, blue.b, 0, 0.2, m_FullRight.segmentSize,
            BounceMode.Front, 10, m_FullRight.startIndex);
    Animation a_LeftBlueLarson =
        new LarsonAnimation(blue.r, blue.g, blue.b, 0, 0.2, m_FullLeft.segmentSize,
            BounceMode.Front, 10, m_FullLeft.startIndex);
    Animation a_RightRainbow =
        new RainbowAnimation(0.7, 0.5, m_FullRight.segmentSize, true, m_FullRight.startIndex);
    Animation a_LeftRainbow =
        new RainbowAnimation(0.7, 0.5, m_FullLeft.segmentSize, false, m_FullLeft.startIndex);
    Animation a_RightFlame =
        new FireAnimation(1.0, 0.75, m_FullRight.segmentSize, 1.0, 0.1, true,
            m_FullRight.startIndex);
    Animation a_LeftFlame =
        new FireAnimation(1.0, 0.75, m_FullLeft.segmentSize, 1.0, 0.1, false,
            m_FullLeft.startIndex);

    // Robot State Animations
    // Intaking
    Animation a_FastFlashRed = new StrobeAnimation(red.r, red.g, red.b, 0, 0.8,
        m_State.segmentSize, m_State.startIndex);
    // Climbing
    Animation a_SlowFlashRed = new StrobeAnimation(red.r, red.g, red.b, 0, 0.2,
        m_State.segmentSize, m_State.startIndex);
    // Super Move
    Animation a_MedFlashMagenta = new StrobeAnimation(magenta.r, magenta.g, magenta.b, 0, 0.5,
        m_State.segmentSize, m_State.startIndex);
    // Aligning
    Animation a_MedFlashCyan = new StrobeAnimation(cyan.r, cyan.g, cyan.b, 0, 0.5,
        m_State.segmentSize, m_State.startIndex);
    // Enabled
    Animation a_SingleFadeFastYellow = new SingleFadeAnimation(yellow.r, yellow.g, yellow.b, 0, 0.8,
        m_State.segmentSize, m_State.startIndex);

    // Match Timer Animations
    Animation a_InAutonomous = new StrobeAnimation(yellow.r, yellow.g, yellow.b, 0, 0.8,
        m_MatchTime.segmentSize, m_MatchTime.startIndex);
    Animation a_TimeExpiring = new StrobeAnimation(red.r, red.g, red.b, 0, 0.5,
        m_MatchTime.segmentSize, m_MatchTime.startIndex);


    // Match Timer Module
    // * Autonomous (15 sec): Flashing Yellow
    // * 2:15 -> 1:00: Solid Green
    // * 1:00 -> 0:20: Solid Yellow
    // * 0:20 -> 0:10: Solid Red
    // * 0:10 -> 0:00: Strobing Red
    // * Non-auto periods & Disabled: White

    Timer m_pseudoTimer = new Timer();
    Color currentColor = black;

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
                m_MatchTime.setAnimation(a_TimeExpiring);
            } else {
                m_MatchTime.setColor(newColor);
            }
            currentColor = newColor;
        }
    }

    public void timerDisabled()
    {
        m_MatchTime.setOff();
        m_pseudoTimer.stop();
        m_pseudoTimer.reset();
    }

    LEDState testLEDState(int stateNum)
    {

        switch (stateNum) {

            default:
            case 0:
                return LEDState.START;
            case 1:
                return LEDState.DISABLED;
            case 2:
                return LEDState.DISABLED_TARGET;
            case 3:
                return LEDState.AUTONOMOUS;
            case 4:
                return LEDState.INTAKING;
            case 5:
                return LEDState.FEEDING;
            case 6:
                return LEDState.CLIMBING;
            case 7:
                return LEDState.CLIMBED;
            case 8:
                return LEDState.SUPER_MOVE;
            case 9:
                return LEDState.ALIGNING;
            case 10:
                return LEDState.HAVE_CORAL;
            case 11:
                return LEDState.ENABLED;
        }
    }

}

