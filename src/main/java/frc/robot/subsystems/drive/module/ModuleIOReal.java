package frc.robot.subsystems.drive.module;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.module.ModuleIO.ModuleIOInputs;

public class ModuleIOReal {
    
    public SparkMax turnMotor; 

    public SparkMax driveMotor; 

    public PIDController turnPID;

    public int moduleNumber;

    public ModuleIOReal(int moduleNumber)
    {
        this.moduleNumber = moduleNumber;
    }

    public void updateInputs(ModuleIOInputs inputs) 
    {

    }


    public void setDriveVoltage(double output) 
    {

    }

    public void setTurnVoltage(double output) 
    {

    }

    public void setDriveVelocity(double velocityRadPerSec)
    {

    }

    public void setTurnPosition(Rotation2d rotation)
    {

    }

    public void getTurnDegrees()
    {

    }

}
