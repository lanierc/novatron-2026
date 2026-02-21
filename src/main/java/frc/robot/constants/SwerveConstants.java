package frc.robot.constants;

import static edu.wpi.first.units.Units.Meter;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class SwerveConstants {

        public static final double stickDeadband = 0.1;

        /* Drivetrain Constants */
        private static final double trackWidth = Units.inchesToMeters(23); // Distance between left and right wheels
        private static final double wheelBase = Units.inchesToMeters(23); // Distance between front and back wheels
        private static final double wheelDiameterInches = 4.0; // Diameter of the wheel , wheel radius = 2 inch
        private static final double wheelDiameter = Units.inchesToMeters(wheelDiameterInches); // Diameter of the wheel
        private static final double centerToWheel = Math.hypot(trackWidth / 2.0, wheelBase / 2.0); // Distance from
                                                                                                   // center of robot to
                                                                                                   // wheel

        private static final double driveGearRatio = (6.75 / 1.0);
        private static final double angleGearRatio = (21.42 / 1.0);

        /* Drive Motor Conversion Factors */
        public static final double driveReductionMK4I = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
        public static final double steerReductionMK4I = (14.0 / 50.0) * (10.0 / 60.0);
        public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
        public static final double angleConversionFactor = 360.0 / angleGearRatio;
        public static final double driveMotorRPM = 6000.0;

        public static final double maxSpeed = Units
                        .feetToMeters(driveMotorRPM / driveGearRatio / 60 * wheelDiameterInches * Math.PI / 12);

        public static final Pose2d blueInitalPosition = new Pose2d(new Translation2d(Meter.of(7.139), Meter.of(3.099)),
                        Rotation2d.fromDegrees(180));
        public static final Pose2d redInitalPosition = new Pose2d(new Translation2d(Meter.of(10.139), Meter.of(3.099)),
                        Rotation2d.fromDegrees(0));

        public static final double maxAngularVelocity = maxSpeed / centerToWheel; // rad/s
        public static final PathConstraints pathfindToPoseConstraint = new PathConstraints(2, 2,
                        Units.degreesToRadians(540), Units.degreesToRadians(720));

}