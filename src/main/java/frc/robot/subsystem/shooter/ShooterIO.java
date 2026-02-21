package frc.robot.subsystem.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    
    @AutoLog
    public static class ShooterIOInputs {
        public double shooterWheelRPM = 0.0;
        public double shooterWheelVoltage = 0.0;
        public double shooterWheelCurrent = 0.0;
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(ShooterIOInputsAutoLogged inputs) {}

    /** Sets the shooter wheel motor voltage. */
    default void setShooterWheelVoltage(double voltage) {}

    /** Sets the shooter pivot motor voltage. */
    default void setShooterPivotVoltage(double voltage) {}

    /** Gets the current shooter wheel RPM. */
    default double getShooterWheelRPM() {
        return 0.0;
    }

    /** Gets the current shooter wheel voltage. */
    default double getShooterWheelVoltage() {
        return 0.0;
    }

    /** Gets the current shooter wheel current draw. */
    default double getShooterWheelCurrent() {
        return 0.0;
    }
}

