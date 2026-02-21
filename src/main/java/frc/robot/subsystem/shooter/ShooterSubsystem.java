package frc.robot.subsystem.shooter;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

    public enum WantedState {
        IDLE,
        SPINUP,
        SHOOT,
        WHEEL_IDLE
    }

    private enum SystemState { 
        IDLED, 
        SPINNING_UP, 
        READY, 
        SHOOTING, 
        WHEEL_IDLED 
    } 

    
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLED;

    private double shooterWheelVoltageSetpoint = 0.0;
    private double shooterPivotVoltageSetpoint = 0.0;
    private boolean isReadyToShoot = false;

    public ShooterSubsystem(ShooterIO io) {
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
        Logger.processInputs("Subsystems/Shooter", inputs);
        Logger.recordOutput("Subsystems/Shooter/WantedState", wantedState);
        Logger.recordOutput("Subsystems/Shooter/SystemState", systemState);
        Logger.recordOutput("Subsystems/Shooter/WheelVoltageSetpoint", shooterWheelVoltageSetpoint);
        Logger.recordOutput("Subsystems/Shooter/PivotVoltageSetpoint", shooterPivotVoltageSetpoint);
        Logger.recordOutput("Subsystems/Shooter/IsReadyToShoot", isReadyToShoot);
        Logger.recordOutput("Subsystems/Shooter/WheelRPMError", ShooterConstants.SHOOTER_WHEEL_TARGET_RPM - inputs.shooterWheelRPM);
    }

    private SystemState handleStateTransition() {
        switch (wantedState) {
            case SHOOT:
                return SystemState.SHOOTING;

            case WHEEL_IDLE:
                return SystemState.WHEEL_IDLED;

            case IDLE:
            default:
                isReadyToShoot = false;
                return SystemState.IDLED;
        }
    }

    private void handleSystemState() {
        switch (systemState) {

            case SHOOTING:
                shooterWheelVoltageSetpoint = ShooterConstants.SHOOTER_WHEEL_SHOOT_VOLTAGE;
                break;

            case WHEEL_IDLED:
                shooterWheelVoltageSetpoint = ShooterConstants.SHOOTER_WHEEL_IDLE_VOLTAGE;
                break;

            case IDLED:
            default:
                shooterWheelVoltageSetpoint = ShooterConstants.SHOOTER_WHEEL_IDLE_VOLTAGE;
                isReadyToShoot = false;
                break;
        }

        io.setShooterWheelVoltage(shooterWheelVoltageSetpoint);
    }



    // Public methods
    public void setWantedState(WantedState state) {
        wantedState = state;
    }

    public WantedState getWantedState() {
        return wantedState;
    }

    public SystemState getSystemState() {
        return systemState;
    }

    public boolean isReadyToShoot() {
        return isReadyToShoot;
    }

    public boolean isShooting() {
        return systemState == SystemState.SHOOTING;
    }

    public boolean isSpinningUp() {
        return systemState == SystemState.SPINNING_UP;
    }
}
