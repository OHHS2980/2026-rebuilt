package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
   
    public final IntakeMotors motor1;

    public Intake(IntakeMotors motor1) {
        this.motor1 = motor1;
    }

    public Command start() {
        return Commands.run(() -> motor1.intake1SetPower(1), this);
    }

    public Command stop() {
        return Commands.runOnce(() -> motor1.intake1SetPower(0), this);
    }
}