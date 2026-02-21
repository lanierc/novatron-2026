// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.controls.DriverControls;
import frc.robot.subsystem.Superstructure;
import frc.robot.subsystem.drivebase.SwerveSubsystem;
import frc.robot.subsystem.hopper.HopperIO;
import frc.robot.subsystem.hopper.HopperIOSpark;
import frc.robot.subsystem.hopper.HopperSubsystem;
import frc.robot.subsystem.shooter.ShooterSubsystem;
import frc.robot.subsystem.shooter.ShooterIO;
import frc.robot.subsystem.shooter.ShooterIOTalonFX;

public class RobotContainer {
  
  private final HopperIO hopperIO;
  private final ShooterIO shooterIO;
  private final HopperSubsystem hopperSubsystem;
  private final SwerveSubsystem swerveSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  
  private final Superstructure superstructure;

  private static final int DRIVER_CONTROLLER_PORT = 0;

  public RobotContainer() {
    // Initialize Hopper subsystem
    hopperIO = new HopperIOSpark();
    shooterIO = new ShooterIOTalonFX();
    hopperSubsystem = new HopperSubsystem(hopperIO);
    swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
          "swerve/talon"));
    shooterSubsystem = new ShooterSubsystem(shooterIO);
  
  
    // Initialize Superstructure with all subsystems
    superstructure = new Superstructure(hopperSubsystem, swerveSubsystem, shooterSubsystem);


    configureBindings();
  }

  private void configureBindings() {
    DriverControls.configure(DRIVER_CONTROLLER_PORT, superstructure);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
