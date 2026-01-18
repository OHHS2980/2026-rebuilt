package frc.robot.subsystems.Turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import frc.robot.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Turret.TurretIO.TurretIOInputs;

import frc.robot.subsystems.vision.*;

public class Turret extends SubsystemBase {
    
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
            .plus(new Rotation2d(Math.toRadians(90)))
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
        double y = RobotState.getInstance().estimatedPose.getY()  - Constants.FieldConstants.getHub().getY();
        double x = RobotState.getInstance().estimatedPose.getX()  - Constants.FieldConstants.getHub().getX();
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

    public void update()
    {
        System.out.println(turretPID.calculate(turretIO.getRotation().getDegrees()));

        System.out.println("error"+turretPID.getPeriod());

        System.out.println("error"+turretPID.getErrorDerivative());

        System.out.println("set"+turretPID.getSetpoint());


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

    @Override
    public void periodic()
    {
        turretIO.updateInputs(inputs);

        update();

        setDesiredRotation(odometryAutoalign());
    }
}
