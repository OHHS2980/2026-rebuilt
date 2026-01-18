package frc.robot.subsystems.Indexing;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexing extends SubsystemBase{
    public final IndexMotor io;

    public Indexing(IndexMotor io) {
        this.io = io;
    }

    public Command start() {
        return Commands.run(() -> io.setVoltage(1), this);
    }

    public Command stop() {
        return Commands.runOnce(() -> io.setVoltage(0), this);
    }
}
