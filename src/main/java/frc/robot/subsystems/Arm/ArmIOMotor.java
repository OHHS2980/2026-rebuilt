package frc.robot.subsystems.Arm;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class ArmIOMotor implements ArmIO
{
    public SparkMax motor;
    
    public CANcoder encoder;


    public PIDController armPID;

    public ArmFeedforward armFeedforward;

    public ArmIOMotor()
    {
        motor = new SparkMax(Constants.armCanID, MotorType.kBrushless);
        encoder = new CANcoder(Constants.armEncoderID);
    }

    
    public void runToPosition(float setpoint) 
    {
        armPID.setSetpoint(setpoint);
        armPID.reset();
    }

        
    public void update(ArmIOInputs inputs) 
    {
        motor.set(
            armPID.calculate(inputs.position)
        
            +

            armFeedforward.calculate(inputs.position, inputs.velocity)
        );
    }

    public void setMotor(double set) 
    {
        motor.set(set);
    }

    public void updateInputs(ArmIOInputs inputs)
    {    
        inputs.voltage = motor.getBusVoltage();

        inputs.current = motor.getOutputCurrent();

        inputs.position = encoder.getAbsolutePosition().getValueAsDouble();
        
        inputs.velocity = encoder.getVelocity().getValueAsDouble();
    }

    public void configurePIDF
    (
        double kP, double kI, double kD, 
        double kS, double kG, double kV, double kA
    ) 
        
    {
      armPID.setPID(kP, kI, kD);
      armFeedforward.setKs(kS);
      armFeedforward.setKg(kG);
      armFeedforward.setKv(kV);
      armFeedforward.setKa(kA);
    }

}