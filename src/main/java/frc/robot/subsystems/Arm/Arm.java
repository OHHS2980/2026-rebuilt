package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.ArmIO.ArmIOInputs;

public class Arm extends SubsystemBase {
        
    ArmIO armIO;

    public ArmIOInputs armIOInputs = new ArmIOInputs();

    public Arm(ArmIO armIO,
        double kP, double kI, double kD, 
        double kS, double kG, double kV, double kA
    )

    {
        this.armIO = armIO;
        armIO.configurePIDF(kP, kI, kD, kS, kG, kV, kA);
    }

    
    
    @Override
    public void periodic()
    {
        armIO.updateInputs(armIOInputs);
        
        armIO.update(armIOInputs);
    }
}