package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.module.Module;

import frc.robot.subsystems.drive.module.ModuleIO;

public class Drive extends SubsystemBase {

    public ChassisSpeeds chassisSpeeds;

    public SwerveDriveKinematics kinematics;

    public SwerveModuleState[] moduleStates = new SwerveModuleState[4];

    public Pose2d pose = new Pose2d();

    public Drive(
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {

        flModule = new Module(flModuleIO);
        frModule = new Module(frModuleIO);
        blModule = new Module(blModuleIO);
        frModule = new Module(brModuleIO);

        kinematics = new SwerveDriveKinematics(
            Constants.flLocation,
            Constants.frLocation, 
            Constants.blLocation, 
            Constants.brLocation
        );


    }

    public Module flModule, frModule, blModule, brModule;

    public void updateModuleStates()
    {
        moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

        for (int module = 0; module < 3; module++)
        {
            modules[module].runToState(moduleStates[module]);
        }
    }


    public void updateModuleSpeeds()
    {
        for (int module = 0; module < 3; module++)
        {
            modules[module].update();
        }
    }

    //fl - 1
    //fr - 2
    //bl - 3
    //br - 4
    public Module[] modules = {flModule, frModule, blModule, brModule};

    @Override
    public void periodic()
    {
        updateModuleSpeeds();
    }

    public Command driveFieldCentric(
        Drive drive,
        DoubleSupplier x,
        DoubleSupplier y,
        DoubleSupplier rotation
    )
    {
        return Commands.run(
         () ->
            {
                ChassisSpeeds newSpeeds = new ChassisSpeeds(
                    x.getAsDouble(),
                    y.getAsDouble(),
                    rotation.getAsDouble()
                );

                Rotation2d side =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red
                      ? new Rotation2d(0)
                      : new Rotation2d(Math.PI);

                drive.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds
                (
                    newSpeeds,
                    RobotState.getInstance().getRotation().plus(side)
                );

                drive.updateModuleStates();
                
            }
         , drive);
    }

    public Command driveRobotCentric(
        Drive drive,
        DoubleSupplier x,
        DoubleSupplier y,
        DoubleSupplier rotation
    )
    {
        return Commands.run(
         () ->
            {
                drive.chassisSpeeds = new ChassisSpeeds(
                    x.getAsDouble(),
                    y.getAsDouble(),
                    rotation.getAsDouble()
                );

                drive.updateModuleStates();
                
            }
         , drive);
    }

}
