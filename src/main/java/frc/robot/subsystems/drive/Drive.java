package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.module.Module;

import frc.robot.subsystems.drive.module.ModuleIO;

public class Drive extends SubsystemBase {

    public ChassisSpeeds chassisSpeeds;

    public SwerveDriveKinematics kinematics;

    public SwerveDriveOdometry odometry;

    public GyroIO gyroIO;

    public Module flModule, frModule, blModule, brModule;

    public Module[] modules = {flModule, frModule, blModule, brModule};

    public SwerveModuleState[] moduleStates = new SwerveModuleState[4];

    public SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    public Pose2d pose = new Pose2d();

    public Drive(
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,

      double kP, double kI, double kD
    ) {

        flModule = new Module(flModuleIO, kP, kI, kD);
        frModule = new Module(frModuleIO, kP, kI, kD);
        blModule = new Module(blModuleIO, kP, kI, kD);
        brModule = new Module(brModuleIO, kP, kI, kD);

        modules[0] = flModule;
        modules[1] = frModule;
        modules[2] = blModule;
        modules[3] = brModule;

        kinematics = new SwerveDriveKinematics(
            Constants.flLocation,
            Constants.frLocation, 
            Constants.blLocation, 
            Constants.brLocation
        );

        odometry = new SwerveDriveOdometry(
            kinematics, 
            new Rotation2d(0,0), 
            new SwerveModulePosition[] 
            {
                flModule.getPosition(),
                frModule.getPosition(),
                blModule.getPosition(),
                brModule.getPosition()
            }
        );

    }


    public Pose2d getOdomPose()
    {

        
        odometry.update(
            new Rotation2d(), 
            new SwerveModulePosition[] 
            {
                flModule.getPosition(),
                frModule.getPosition(),
                blModule.getPosition(),
                brModule.getPosition()
            }
        );

        return odometry.getPoseMeters();
    }

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

    @Override
    public void periodic()
    {
        RobotState.getInstance().setPose(getOdomPose());

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
