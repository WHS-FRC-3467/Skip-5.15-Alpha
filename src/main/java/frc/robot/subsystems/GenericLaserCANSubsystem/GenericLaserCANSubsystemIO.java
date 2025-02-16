package frc.robot.subsystems.GenericLaserCANSubsystem;

import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Distance;
import lombok.Getter;

public interface GenericLaserCANSubsystemIO {
    @AutoLog
    abstract class LaserCANIOInputs {
        public Distance distance;
    }

    default BooleanSupplier getValidStatus() 
    {
        return ()-> true;
    }

    default void updateInputs(LaserCANIOInputs inputs)
    {}
}
