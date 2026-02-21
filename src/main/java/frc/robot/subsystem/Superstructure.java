package frc.robot.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HopperConstants;
import frc.robot.subsystem.drivebase.SwerveSubsystem;
import frc.robot.subsystem.hopper.HopperSubsystem;
import frc.robot.subsystem.shooter.ShooterSubsystem;

public class Superstructure extends SubsystemBase {
    private final SwerveSubsystem swerveSubsystem;
    private final HopperSubsystem hopperSubsystem;
    private final ShooterSubsystem shooterSubsystem;

    public enum WantedSuperState {
        IDLE,
        INTAKE,
        EJECT,
        FEED,
        HOLD,
        STOWED,
        SPINUP_SHOOTER,
        SHOOT
    }

    public enum CurrentSuperState {
        IDLE,
        INTAKING,
        EJECTING,
        FEEDING,
        HOLDING,
        STOWED,
        SHOOTING,
        SPINNING_UP,
        READY,
        WHEEL_IDLED
    }

    private WantedSuperState wantedSuperState = WantedSuperState.IDLE;
    private CurrentSuperState currentSuperState = CurrentSuperState.IDLE;

    public Superstructure(HopperSubsystem hopperSubsystem, SwerveSubsystem swerveSubsystem,
            ShooterSubsystem shooterSubsystem) {
        this.hopperSubsystem = hopperSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void periodic() {
        currentSuperState = handleStateTransitions();
        applyStates();
    }

    private CurrentSuperState handleStateTransitions() {
        return CurrentSuperState.IDLE;
    }

    private void applyStates() {
        switch (currentSuperState) {
            case INTAKING:
                hopperSubsystem.setIntakeVoltage(HopperConstants.HOPPER_FUEL_INTAKE_VOLTAGE);
                break;
            case EJECTING:
                hopperSubsystem.setIntakeVoltage(-HopperConstants.HOPPER_FUEL_INTAKE_VOLTAGE);
                break;
            case FEEDING:
                hopperSubsystem.setFeederVoltage(HopperConstants.HOPPER_FEEDER_INTAKE_VOLTAGE);
                break;
            case HOLDING:
                hopperSubsystem.setFeederVoltage(HopperConstants.HOPPER_FEEDER_HOLDING_VOLTAGE);
                break;
            case STOWED:
                hopperSubsystem.setIntakeVoltage(0.0);
                hopperSubsystem.setFeederVoltage(0.0);
                break;
            case SPINNING_UP:
                shooterSubsystem.setWantedState(ShooterSubsystem.WantedState.SPINUP);
                break;
            case SHOOTING:
                shooterSubsystem.setWantedState(ShooterSubsystem.WantedState.SHOOT);
                break;
            default:
                hopperSubsystem.setIntakeVoltage(0.0);
                hopperSubsystem.setFeederVoltage(0.0);
                shooterSubsystem.setWantedState(ShooterSubsystem.WantedState.IDLE);
        }
    }

    public void setWantedSuperState(WantedSuperState state) {
        wantedSuperState = state;
    }

    public WantedSuperState getWantedSuperState() {
        return wantedSuperState;
    }

    public CurrentSuperState getCurrentSuperState() {
        return currentSuperState;
    }

    public boolean isIntaking() {
        return currentSuperState == CurrentSuperState.INTAKING;
    }

    public boolean isEjecting() {
        return currentSuperState == CurrentSuperState.EJECTING;
    }

    public boolean isFeeding() {
        return currentSuperState == CurrentSuperState.FEEDING;
    }

    public boolean isHolding() {
        return currentSuperState == CurrentSuperState.HOLDING;
    }

    public boolean isIdled() {
        return currentSuperState == CurrentSuperState.IDLE;
    }

    public boolean isShooting() {
        return currentSuperState == CurrentSuperState.SHOOTING;
    }

    // Subsystem getters (for future subsystems)
    public HopperSubsystem getHopperSubsystem() {
        return hopperSubsystem;
    }

    public SwerveSubsystem getSwerveSubsystem() {
        return swerveSubsystem;
    }

    public ShooterSubsystem getShooterSubsystem() {
        return shooterSubsystem;
    }
}
