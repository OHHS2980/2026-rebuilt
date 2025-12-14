package frc.robot.subsystems.outtake;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Outtake extends SubsystemBase{
    public final OuttakeMotor io;

    public Outtake(OuttakeMotor io) {
        this.io = io;
    }

    public Command start() {
        return Commands.run(() -> io.setVoltage(1), this);
    }

    public Command stop() {
        return Commands.runOnce(() -> io.setVoltage(0), this);
    }
}
