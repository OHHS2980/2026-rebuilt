package frc.robot.subsystems.drive.module;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.ModuleState;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.drive.module.ModuleIO.ModuleIOInputs;

public class Module {

    public SparkMax turnMotor; 

    public SparkMax driveMotor; 

    public PIDController turnPID;

    public PIDController drivePID;

    public ModuleIO moduleIO;

    public int moduleNumber;

    public ModuleState desiredModuleState;

    public ModuleIOInputs inputs;

    public enum position  
    {
        frontLeft,
        frontRight,
        backLeft,
        backRight
    }

    public Module(ModuleIO moduleIO)
    {
        this.moduleIO = moduleIO; 
    }

    public void update()
    {
        moduleIO.setTurnVoltage(
            turnPID.calculate(inputs.turnPosition.getDegrees())
        );
    }

    public void runToState(SwerveModuleState state)
    {
        moduleIO.setDriveVelocity(state.speedMetersPerSecond);
        moduleIO.setTurnPosition(state.angle);


    }
}
