package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
        
    ArmIO armIO;

    public Arm(ArmIO armIO)
    {
        this.armIO = armIO;
    }

    class moveArm90degreesyay extends Command
    {
        public void initialize()
        {
            armIO.setSetPoint(90);
        }

        public void execute()
        {
            armIO.update();
        }

        public boolean isFinished()
        {
            if (Math.abs(armIO.getAngleFromPosition() - 90) < 2.5)
            {
                return true;
            }
            return false;
        }

        public void end(boolean isFinished)
        {
            armIO.setMotor(0);
        }
    }

    public void periodic()
    {
        
    }
}