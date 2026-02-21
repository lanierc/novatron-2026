package frc.robot.subsystem.hopper;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HopperConstants;

public class HopperSubsystem extends SubsystemBase {

    public enum WantedState {
        IDLE,
        INTAKE,
    }

    private enum SystemState {
        IDLED,
        INTAKING
    }

    private final HopperIO io;
    private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLED;

    private double feederVoltageSetpoint = 0.0;
    private double fuelVoltageSetpoint = 0.0;

    
    public HopperSubsystem(HopperIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        
        systemState = handleStateTransition();
        handleSystemState();
        
        updateRecords();
    }

    private void updateRecords() {
        Logger.processInputs("Subsystems/Hopper", inputs);
        Logger.recordOutput("Subsystems/Hopper/WantedState", wantedState);
        Logger.recordOutput("Subsystems/Hopper/SystemState", systemState);
    }

    private SystemState handleStateTransition() {
        switch (wantedState) {
            case INTAKE:
                return SystemState.INTAKING;

            case IDLE:
            default:
                return SystemState.IDLED;
        }
    }
    public void setIntakeVoltage(double voltage) {
        feederVoltageSetpoint = voltage;
        fuelVoltageSetpoint = voltage;
        io.setHopperFeederVoltage(feederVoltageSetpoint);
        io.setHopperFuelVoltage(fuelVoltageSetpoint);
    }
    public void setFeederVoltage(double voltage) {
        feederVoltageSetpoint = voltage;
        io.setHopperFeederVoltage(feederVoltageSetpoint);
    }
    private void handleSystemState() {
        switch (systemState) {
            case INTAKING:
                feederVoltageSetpoint = HopperConstants.HOPPER_FEEDER_INTAKE_VOLTAGE;
                fuelVoltageSetpoint = HopperConstants.HOPPER_FUEL_INTAKE_VOLTAGE;
                break;

            case IDLED:
            default:
                feederVoltageSetpoint = HopperConstants.HOPPER_FEEDER_IDLE_VOLTAGE ;
                fuelVoltageSetpoint = HopperConstants.HOPPER_FUEL_IDLE_VOLTAGE ;
                break;
        }

        io.setHopperFeederVoltage(feederVoltageSetpoint);
        io.setHopperFuelVoltage(fuelVoltageSetpoint);
    }

    public void setWantedState(WantedState state) {
        wantedState = state;
    }

    public WantedState getWantedState() {
        return wantedState;
    }

    public SystemState getSystemState() {
        return systemState;
    }

    public boolean isIntaking() {
        return systemState == SystemState.INTAKING;
    }

    public boolean isIdled() {
        return systemState == SystemState.IDLED;
    }
}
