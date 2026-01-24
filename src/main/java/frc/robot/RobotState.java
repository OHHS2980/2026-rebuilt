package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator3d;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.drive.GyroIO;

public class RobotState
{

    public PoseEstimator3d poseEstimator;

    public SwerveDrivePoseEstimator3d swerveEstimator;

    private static RobotState instance;

    public Pose2d estimatedPose = new Pose2d();

    public Rotation2d getRotation()
    {
        return estimatedPose.getRotation();
    }

    public static RobotState getInstance() {
        if (instance == null) instance = new RobotState();
        return instance;
    }

    public void addToBuffer(Pose3d)
    {

    }

    public void addMeasurement(Pose3d pose, double timestamp)
    {
        poseEstimator.addVisionMeasurement(pose, timestamp);
    }

    public void setup(SwerveDriveKinematics kinematics, GyroIO gyro)
    {
        swerveEstimator = new SwerveDrivePoseEstimator3d(kinematics,
            new Rotation3d(),
            new SwerveModulePosition[4],
            new Pose3d()
        );
    }
    
    public void update(Rotation3d gyroAngle, SwerveModulePosition[] positions)
    {
        swerveEstimator.update(gyroAngle, positions);
    }

    public Pose3d getEstimatedPose()
    {
        return poseEstimator.getEstimatedPosition();
    }

    public Translation2d getPosition()
    {
        return estimatedPose.getTranslation();
    }

    public Pose2d getPose()
    {
        return estimatedPose;
    }

    public void setPose(Pose2d odomPose) {
        estimatedPose = odomPose;
    }
}
