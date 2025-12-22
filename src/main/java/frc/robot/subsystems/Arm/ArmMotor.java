package frc.robot.subsystems.Arm;
import com.revrobotics.AbsoluteEncoder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;

public class ArmIOMotor extends ArmIO
{
    public SparkMax motor;

    public AbsoluteEncoder motorEncoder;

    public PIDController armPID;

    public double kP = 0.454543242;
    public double kI = 0.23411;
    public double kD = 0.111;
    public double kF = 0;

    public ArmFeedforward armFeedforward;

    public ArmIOMotor()
    {
        motor = new SparkMax(Constants.armCanID, MotorType.kBrushless);
        motorEncoder = motor.getAbsoluteEncoder();
        armPID = new PIDController(kP, kI, kD);
        armFeedforward = new ArmFeedforward(voltage, temperature, angle);
        armPID = new PIDController(6, kI7, kD1)
    }

    
    public void setSetPoint(double setpoint) 
    {
        armPID.setSetpoint(setpoint);
        armPID.reset();
    }

        
    public void update() 
    {
        motor.set(
            armPID.calculate(getAngleFromPosition())
        
            +

            armFeedforward.calculate(getAngleFromPosition(), armPID.getSetpoint()) * kF
        );
    }
    
    
    public double getAngleFromPosition() 
    {
        return motorEncoder.getPosition();
    }

    public void setMotor(double set) 
    {
        motor.set(set);
    }

    public void updateInputs()
    {
        ifOk(motor, () -> motor.getBusVoltage(), (value) -> inputs.intakeVoltage = value);
        ifOk(motor, () -> motor.getOutputCurrent(), (value) -> inputs.intakeVoltage = value);
        ifOk(motor, () -> motorEncoder.getPosition(), (value) -> inputs.intakePosition = value);
        ifOk(motor, () -> motorEncoder.getVelocity(), (value) -> inputs.intakeVelocity = value);
        ifOk(motor, () -> motor.getMotorTemperature(), (value) -> inputs.intakeTempC = value);

    
        inputs.intakeMotorConnected = sparkStickyFault;
    
        sparkStickyFault = false;
    
        ifOk(alignMotor, () -> alignMotor.getBusVoltage(), (value) -> inputs.alignVoltage = value);
        ifOk(alignMotor, () -> alignMotor.getOutputCurrent(), (value) -> inputs.alignVoltage = value);
        ifOk(alignMotor, () -> alignEncoder.getPosition(), (value) -> inputs.alignPosition = value);
        ifOk(alignMotor, () -> alignEncoder.getVelocity(), (value) -> inputs.alignVelocity = value);
        ifOk(alignMotor, () -> alignMotor.getMotorTemperature(), (value) -> inputs.alignTempC = value);
    
        inputs.alignMotorConnected = sparkStickyFault;
    }

}