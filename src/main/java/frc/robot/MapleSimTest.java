package frc.robot;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class MapleSimTest {
    private static MapleSimTest instance;


    public static MapleSimTest getInstance() {
        if (instance == null) instance = new MapleSimTest();
        return instance;
    }
}
