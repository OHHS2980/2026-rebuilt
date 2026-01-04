// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.module.ModuleIOSim;

import static edu.wpi.first.units.Units.Inches;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  XboxController controller;

  Drive drive;

  SwerveDriveSimulation driveSim;
  final DriveTrainSimulationConfig driveSimConfig = DriveTrainSimulationConfig.Default()
        // Specify gyro type (for realistic gyro drifting and error simulation)
        .withGyro(COTS.ofPigeon2())
        // Specify swerve module (for realistic swerve dynamics)
        .withSwerveModule(COTS.ofMark4(
                DCMotor.getKrakenX60(1), // Drive motor is a Kraken X60
                DCMotor.getFalcon500(1), // Steer motor is a Falcon 500
                COTS.WHEELS.COLSONS.cof, // Use the COF for Colson Wheels
                3)) // L3 Gear ratio
        // Configures the track length and track width (spacing between swerve modules)
        .withTrackLengthTrackWidth(Inches.of(24), Inches.of(24))
        // Configures the bumper size (dimensions of the robot bumper)
        .withBumperSize(Inches.of(30), Inches.of(30));

  // Replace with CommandPS4Controller or CommandJoystick if needed

  public void mapleSimSetup()
  {
   
  }
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    controller = new XboxController(0);

    this.driveSim = new SwerveDriveSimulation(
      driveSimConfig,
      new Pose2d()
    );

    this.drive = new Drive(
      new ModuleIOSim(driveSim.getModules()[0],0),
      new ModuleIOSim(driveSim.getModules()[1],1),
      new ModuleIOSim(driveSim.getModules()[2],2),
      new ModuleIOSim(driveSim.getModules()[3],3),
      0.1, 0.1, 0.1
    );

    configureBindings();
    mapleSimSetup();



  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

      drive.setDefaultCommand(
        drive.driveFieldCentric
        (
          drive, 
          () -> controller.getLeftX(),
          () -> controller.getLeftY(),
          () -> controller.getRightX()
        )
      );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
