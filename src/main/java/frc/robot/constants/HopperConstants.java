package frc.robot.constants;

import com.revrobotics.spark.SparkBase;

public class HopperConstants {
    
    // Motor Ports
    public static final int HOPPER_FEEDER_PORT = 11;
    public static final int HOPPER_FUEL_PORT = 10;

    // Motor Types
    public static final SparkBase.MotorType HOPPER_FEEDER_MOTOR_TYPE = SparkBase.MotorType.kBrushless;
    public static final SparkBase.MotorType HOPPER_FUEL_MOTOR_TYPE = SparkBase.MotorType.kBrushless;
    public static final double HOPPER_FEEDER_CURRENT_LIMIT = 20.0;
    // Voltage Setpoints
    public static final double HOPPER_FEEDER_INTAKE_VOLTAGE = 12.0;
    public static final double HOPPER_FUEL_INTAKE_VOLTAGE = -4.8;

    public static final double HOPPER_FEEDER_HOLDING_VOLTAGE = 4.8;
    public static final double HOPPER_FEEDER_IDLE_VOLTAGE = 0.0;
    public static final double HOPPER_FUEL_IDLE_VOLTAGE = 0.0;

}
