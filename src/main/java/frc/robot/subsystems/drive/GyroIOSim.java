package frc.robot.subsystems.drive;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOSim implements GyroIO {
    
    GyroSimulation gyroSim;

    public GyroIOSim(GyroSimulation driveGyroSim)
    {
        gyroSim = driveGyroSim;
    }

    public Rotation2d getHeading()
    {
        return gyroSim.getGyroReading();
    }
}
