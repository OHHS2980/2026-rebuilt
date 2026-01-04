package frc.robot.subsystems.drive.module;

import static edu.wpi.first.units.Units.Meter;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.ModuleState;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.drive.module.ModuleIO.ModuleIOInputs;

public class Module {


    public PIDController turnPID;

    public PIDController drivePID;

    public ModuleIO moduleIO;

    public int moduleNumber;

    public SwerveModulePosition modulePosition;

    public ModuleState desiredModuleState;

    public ModuleIOInputs inputs;

    public enum position  
    {
        frontLeft,
        frontRight,
        backLeft,
        backRight
    }

    public Module(
        
        ModuleIO moduleIO,

        double kP, double kI, double kD

    )

    {
        this.moduleIO = moduleIO; 

        turnPID = new PIDController(kP, kI, kD);

        inputs = new ModuleIOInputs();
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

    public SwerveModulePosition getPosition()
    {
        return new SwerveModulePosition(

            moduleIO.getDriveDistance(),

            moduleIO.getTurnDegrees()
            
        );


    }
}
