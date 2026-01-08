package frc.robot.subsystems.drive;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import swervelib.imu.NavXSwerve;

public class GyroIOReal implements GyroIO {
    
    NavXSwerve driveGyro;

    public GyroIOReal()
    {
        driveGyro = new NavXSwerve(NavXComType.kUSB1);
    }

    public Rotation2d getHeading()
    {
        return driveGyro.getRotation3d().toRotation2d();
    }
}
