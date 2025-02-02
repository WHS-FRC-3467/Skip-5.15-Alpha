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
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Vision.Vision;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;


public class LED extends SubsystemBase {

    // Subsystems to query
    Arm m_Arm;
    Climber m_Climber;
    Drive m_Drive;
    Elevator m_Elevator;
    Vision m_Vision;
    
    // Control everything with a CANdle
    private static final CANdle m_candle = new CANdle(Ports.CANDLE.getDeviceNumber());

    /*
     * Robot LED States
     */
    private static enum LEDState {
        START, DISABLED, DISABLED_TARGET, AUTONOMOUS, ENABLED, INTAKING, CLIMBING, HAVEALGAE, HAVECORAL, AIMING, READY
    }
    LEDState m_currentState = LEDState.START;

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
     * Constructor
     * Creates a new LEDSubsystem
     */
    public LED(Arm arm,
                        Climber climber,
                        Drive drive,
                        Elevator elevator,
                        Vision vision) {
        
        m_Arm = arm;
        m_Climber = climber;
        m_Drive = drive;
        m_Elevator = elevator;
        m_Vision = vision;

        m_candle.configFactoryDefault();
        
        CANdleConfiguration candleConfiguration = new CANdleConfiguration();
        m_candle.getAllConfigs(candleConfiguration);

        candleConfiguration.statusLedOffWhenActive = true;
        candleConfiguration.disableWhenLOS = false;
        candleConfiguration.stripType = LEDStripType.RGB;
        candleConfiguration.brightnessScalar = 0.5;
        candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(candleConfiguration, 100);

        m_candle.getAllConfigs(candleConfiguration);

        m_candle.configLEDType(LEDStripType.RGB, 300);

        m_candle.getAllConfigs(candleConfiguration);
    }

    @Override
    public void periodic()
    {
        // https://github.com/WHS-FRC-3467/Skip-5.14-Nocturne/blob/main/src/main/java/frc/robot/Subsystems/LED/LEDSubsystem.java
        // Flesh out periodic. Use the states of the subsystems to set the LED state

        // Use the LEDStateMachine to set the LEDs
        LEDStateMachine(m_currentState);
    }

    private void LEDStateMachine(LEDState newState) {
        switch (newState) {
                case START:
                    break;
                case DISABLED:
                    break;
                case DISABLED_TARGET:
                    break;
                case AUTONOMOUS:
                    break;
                case ENABLED:
                    break;
                case INTAKING:
                    break;
                case CLIMBING:
                    break;
                case HAVEALGAE:
                    break;
                case HAVECORAL:
                    break;
                case AIMING:
                    break;
                case READY:
                    break;
                default:
                    break;
            }
    }
}
