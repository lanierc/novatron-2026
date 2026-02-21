package frc.robot.subsystem.hopper;


import com.revrobotics.spark.SparkMax;
import frc.robot.constants.HopperConstants;

public class HopperIOSpark implements HopperIO{

    private final SparkMax hopperFeederController;
    private final SparkMax hopperFuelController;

    public HopperIOSpark() {
        hopperFeederController = new SparkMax(
                HopperConstants.HOPPER_FEEDER_PORT,
                HopperConstants.HOPPER_FEEDER_MOTOR_TYPE);
        hopperFuelController = new SparkMax(
                HopperConstants.HOPPER_FUEL_PORT,
                HopperConstants.HOPPER_FUEL_MOTOR_TYPE);
    }

    @Override
    public void updateInputs(HopperIOInputsAutoLogged inputs) {
        inputs.hopperFeederSpeed = hopperFeederController.get();
        inputs.hopperFeederVoltage = hopperFeederController.getBusVoltage();
        inputs.hopperFuelSpeed = hopperFuelController.get();
        inputs.hopperFuelVoltage = hopperFuelController.getBusVoltage();
    }

    @Override
    public void setHopperFeederSpeed(double speed) {
        hopperFeederController.set(speed);
    }

    @Override
    public void setHopperFeederVoltage(double voltage) {
        hopperFeederController.setVoltage(voltage);
    }

    @Override
    public void setHopperFuelSpeed(double speed) {
        hopperFuelController.set(speed);
    }

    @Override
    public void setHopperFuelVoltage(double voltage) {
        hopperFuelController.setVoltage(voltage);
    }

    @Override
    public double getHopperFeederSpeed() {
        return hopperFeederController.get();
    }

    @Override
    public double getHopperFeederVoltage() {
        return hopperFeederController.getBusVoltage();
    }

    @Override
    public double getHopperFuelSpeed() {
        return hopperFuelController.get();
    }

    @Override
    public double getHopperFuelVoltage() {
        return hopperFuelController.getBusVoltage();
    }
}
