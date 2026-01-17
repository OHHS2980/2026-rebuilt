package frc.robot.subsystems.Turret;

import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.geometry.Rotation2d;

public class TurretIOSim implements TurretIO {

    public SparkMaxSim motor;

    public SparkAbsoluteEncoderSim encoder;

    public TurretIOSim()

    {
        encoder = motor.getAbsoluteEncoderSim();
    }

    @Override
    public Rotation2d getRotation()
    {
        return new Rotation2d(encoder.getPosition());
    }

    @Override
    public void setPower(double power) {
        motor.setAppliedOutput(power);
    }
    
}
