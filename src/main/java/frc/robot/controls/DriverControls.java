package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystem.Superstructure;
import frc.robot.subsystem.drivebase.SwerveSubsystem;
import frc.robot.subsystem.hopper.HopperSubsystem;
import frc.robot.subsystem.shooter.ShooterSubsystem;
import swervelib.SwerveInputStream;


public class DriverControls {

    private static CommandXboxController controller;

    public static void configure(int port, Superstructure superstructure) {
        controller = new CommandXboxController(port);
        HopperSubsystem hopperSubsystem = superstructure.getHopperSubsystem();
        SwerveSubsystem swerveSubsystem = superstructure.getSwerveSubsystem();
        ShooterSubsystem shooterSubsystem = superstructure.getShooterSubsystem();

        controller.a()
                .whileTrue(
                    Commands.run(() -> hopperSubsystem.setWantedState(HopperSubsystem.WantedState.INTAKE), hopperSubsystem)
                )
                .onFalse(
                    Commands.runOnce(() -> hopperSubsystem.setWantedState(HopperSubsystem.WantedState.IDLE), hopperSubsystem)
                );

            // B BUTTON - SHOOTER WHEEL (basılı tutunca shoot, birakınca idle)
        controller.b()
                .whileTrue(
                    Commands.run(() -> shooterSubsystem.setWantedState(ShooterSubsystem.WantedState.SHOOT), shooterSubsystem)
                )
                .onFalse(
                    Commands.runOnce(() -> shooterSubsystem.setWantedState(ShooterSubsystem.WantedState.WHEEL_IDLE), shooterSubsystem)
                );

        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
                                () -> controller.getLeftY() * -1,
                                () -> controller.getLeftX() * -1)
                                .withControllerRotationAxis(controller::getRightX)
                                .deadband(0.1)
                                .scaleTranslation(0.8)
                                .scaleRotation(-1)
                                .allianceRelativeControl(true);

                swerveSubsystem.setDefaultCommand(swerveSubsystem.driveFieldOriented(driveAngularVelocity));

   
    }

    public static CommandXboxController getController() {
        return controller;
    }
}

