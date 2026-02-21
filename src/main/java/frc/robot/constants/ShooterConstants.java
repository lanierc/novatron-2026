package frc.robot.constants;

public class ShooterConstants {
    
    // Motor Ports - Kraken X60
    public static final int SHOOTER_WHEEL_PORT1 = 30;
    public static final int SHOOTER_WHEEL_PORT2 = 31;
    public static final int SHOOTER_WHEEL_PORT3 = 32;
    public static final int SHOOTER_WHEEL_PORT4 = 33;
    
    // Shooter Wheel Configuration
    public static final double SHOOTER_WHEEL_SPINUP_VOLTAGE = 9.6;
    public static final double SHOOTER_WHEEL_SHOOT_VOLTAGE = 12;
    public static final double SHOOTER_WHEEL_IDLE_VOLTAGE = 0.0;
    

    // Pivot Configuration - Küçük voltajlar
    public static final double SHOOTER_PIVOT_UP_VOLTAGE = 1.5;
    public static final double SHOOTER_PIVOT_DOWN_VOLTAGE = -1.5;
    public static final double SHOOTER_PIVOT_IDLE_VOLTAGE = 0.0;
    
    // RPM Setpoints (Shooter Wheel)
    public static final double SHOOTER_WHEEL_TARGET_RPM = 4500.0; // Spinup hedefi
    public static final double SHOOTER_WHEEL_RPM_TOLERANCE = 200.0; // RPM toleransı (±200)
    public static final double SHOOTER_WHEEL_SPINUP_TIME = 1.5; // Spinup'a kadar olan süre (saniye)
    
    // Gear Ratio
    public static final double SHOOTER_WHEEL_GEAR_RATIO = 2.0; // Motor rotasyonundan wheel RPM'e çevirmek için
    
}

