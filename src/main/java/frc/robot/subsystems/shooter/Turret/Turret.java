package frc.robot.subsystems.shooter.Turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.Turret.TurretIO.TurretIOInputs;
import frc.robot.subsystems.vision.*;

public class Turret {
    
    // brings things into existence. Is God
    public Pose2d turretPose;

    public PIDController turretPID;

    public Vision camera;

    public TurretIO turretIO;

    public Rotation2d desiredRotation = new Rotation2d();

    public TurretIOInputs inputs = new TurretIOInputs();

    public Turret
    (
        TurretIO turretIO,
        double kP,
        double kI,
        double kD,
        Vision vision
    )

    {
        this.turretIO = turretIO;

        turretPID = new PIDController(kP, kI, kD, 0.02d);

        camera = vision;
        
    }

    public void setDesiredRotation(Rotation2d rotation)
    {
        // defines desired rotation.
        // minus 
        double adjustedAngle = 
        (
            (rotation
            .minus(RobotState.getInstance().getRotation())
            .rotateBy(new Rotation2d(Math.toRadians(90)))
            .getDegrees()
            % Constants.turretLimit)
        );

        //relates definition to desired rotation.
        desiredRotation = new Rotation2d(Math.toRadians(adjustedAngle));

        //sets PID goal to desired rotation
        turretPID.setSetpoint(adjustedAngle);
    }

    // finds x and y from robot to hub
    // then calculates shortest distance
    // and angle
    public Rotation2d odometryAutoalign()
    {

        Translation2d hubDistance = 
            RobotState.getInstance().getPose().getTranslation()
            .minus(Constants.FieldConstants.getHub().getTranslation());

        double y = RobotState.getInstance().getPose().getY() - Constants.FieldConstants.getHub().getY();
        double x = RobotState.getInstance().getPose().getX() - Constants.FieldConstants.getHub().getX();
        double hypotenuse = Math.sqrt(Math.pow(y, 2) + Math.pow(x, 2));

        double angle = Math.asin(
          x / hypotenuse
        );

        return new Rotation2d(angle);
    }

    public Rotation2d visionAutoalign()
    {
    
        return null;
    }

    public void autoalign()
    {
        setDesiredRotation(odometryAutoalign());
    }

    public void update(Timer timer)
    {
        turretIO.updateInputs(inputs, timer);


        turretPID.setPID
        (
            Constants.SimConstants.turretP.get(), 
            Constants.SimConstants.turretI.get(), 
            Constants.SimConstants.turretD.get()
        );

        
        double output = turretPID.calculate(turretIO.getRotation().getDegrees());

        if (Double.isNaN(output))
        {
            output = 0;
        }


        turretIO.setPower
        (
            output
        );


        turretPose = new Pose2d(
            RobotState.getInstance().getPosition(),
            inputs.currentRotation.plus(RobotState.getInstance().getRotation())
        );

    }

}
