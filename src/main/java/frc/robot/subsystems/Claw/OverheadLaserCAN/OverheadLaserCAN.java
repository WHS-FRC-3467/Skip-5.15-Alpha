package frc.robot.subsystems.Claw.OverheadLaserCAN;

import static edu.wpi.first.units.Units.Meter;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.GenericLaserCANSubsystem.GenericLaserCANSubsystem;
import frc.robot.subsystems.GenericLaserCANSubsystem.GenericLaserCANSubsystem.DistanceState;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class OverheadLaserCAN extends GenericLaserCANSubsystem<OverheadLaserCAN.State> {

    public Trigger triggered = new Trigger(() -> super.isTriggered());

    @RequiredArgsConstructor
    @Getter
    public enum State implements DistanceState {
        DEFAULT(Distance.ofBaseUnits(0.29, Meter));

        @Getter
        private final Distance distance;
    }

    @Getter
    @Setter
    private State state = State.DEFAULT;

    public OverheadLaserCAN(OverheadLaserCANIO io)
    {
        super(OverheadLaserCANConstants.kSubSysConstants.kName, io);
    }

}
