package frc.robot.subsystem.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
    
    @AutoLog
    public static class HopperIOInputs {
        public double hopperFeederSpeed = 0.0;
        public double hopperFeederVoltage = 0.0;

        public double hopperFuelSpeed = 0.0;
        public double hopperFuelVoltage = 0.0;

    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(HopperIOInputsAutoLogged inputs) {}

    default void setHopperFeederSpeed(double speed) {}
    default void setHopperFeederVoltage(double voltage) {}

    default void setHopperFuelSpeed(double speed) {}
    default void setHopperFuelVoltage(double voltage) {}

    default double getHopperFeederSpeed(){
        return 0.0;
    }
    default double getHopperFeederVoltage(){
        return 0.0;
    }

    default double getHopperFuelSpeed(){
        return 0.0;
    }
    default double getHopperFuelVoltage(){
        return 0.0;
    }
}
