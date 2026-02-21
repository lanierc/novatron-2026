package frc.robot.subsystem.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static frc.robot.utils.PhoenixUtil.tryUntilOk;
import frc.robot.constants.ShooterConstants;

public class ShooterIOTalonFX implements ShooterIO {

    private TalonFX LEFTShooterWheelMotor1;
    private TalonFX LEFTShooterWheelMotor2;
    private TalonFX RIGHTShooterWheelMotor1;
    private  TalonFX RIGHTShooterWheelMotor2;
    
    public ShooterIOTalonFX() {
        LEFTShooterWheelMotor1 = new TalonFX(ShooterConstants.SHOOTER_WHEEL_PORT1);
        LEFTShooterWheelMotor2 = new TalonFX(ShooterConstants.SHOOTER_WHEEL_PORT2);
        RIGHTShooterWheelMotor1 = new TalonFX(ShooterConstants.SHOOTER_WHEEL_PORT3);
        RIGHTShooterWheelMotor2 = new TalonFX(ShooterConstants.SHOOTER_WHEEL_PORT4);
        configIntakeTalonFX(LEFTShooterWheelMotor1);
        LEFTShooterWheelMotor1 = new TalonFX(ShooterConstants.SHOOTER_WHEEL_PORT1);
        configIntakeTalonFX(LEFTShooterWheelMotor2);
        LEFTShooterWheelMotor2 = new TalonFX(ShooterConstants.SHOOTER_WHEEL_PORT2);
        configIntakeTalonFX(RIGHTShooterWheelMotor1);
        RIGHTShooterWheelMotor1 = new TalonFX(ShooterConstants.SHOOTER_WHEEL_PORT3);
        configIntakeTalonFX(RIGHTShooterWheelMotor2);
        RIGHTShooterWheelMotor2 = new TalonFX(ShooterConstants.SHOOTER_WHEEL_PORT4);
    }

    public void configIntakeTalonFX(TalonFX talon) {

        talon.getConfigurator().apply(new TalonFXConfiguration());
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.FeedbackRotorOffset = 0.0;
        config.Feedback.RotorToSensorRatio = 1.0;
        config.Feedback.SensorToMechanismRatio = 1.0;

        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 1;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0;

        tryUntilOk(5, () -> talon.getConfigurator().apply(config));
    }

    @Override
    public void updateInputs(ShooterIOInputsAutoLogged inputs) {
        // // Shooter Wheel bilgilerini oku
        // inputs.shooterWheelRPM = shooterWheelMotor1.getVelocity().getValueAsDouble() * 60.0 * ShooterConstants.SHOOTER_WHEEL_GEAR_RATIO;
        // inputs.shooterWheelVoltage = shooterWheelMotor1.getMotorVoltage().getValueAsDouble();
        // inputs.shooterWheelCurrent = shooterWheelMotor1.getSupplyCurrent().getValueAsDouble();

    }

    // @Override
    // public void setShooterWheelVoltage(double voltage) {
    //     // shooterWheelMotor1.setVoltage(voltage);

    // }


    // @Override
    // public double getShooterWheelRPM() {
    //     return shooterWheelMotor.getVelocity().getValueAsDouble() * 60.0 * ShooterConstants.SHOOTER_WHEEL_GEAR_RATIO;
    // }

    // @Override
    // public double getShooterWheelVoltage() {
    //     return shooterWheelMotor.getMotorVoltage().getValueAsDouble();
    // }

    // @Override
    // public double getShooterWheelCurrent() {
    //     return shooterWheelMotor.getSupplyCurrent().getValueAsDouble();
    // }
}

