package frc.robot.subsystems.Turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import frc.robot.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Turret.TurretIO.TurretIOInputs;

public class Turret extends SubsystemBase {
    
    public PIDController turretPID;

    public TurretIO turretIO;

    public Rotation2d desiredRotation;

    public TurretIOInputs inputs = new TurretIOInputs();

    public Turret
    (
        TurretIO turretIO,
        double kP,
        double kI,
        double kD
    )

    {
        this.turretIO = turretIO;
        turretPID.setPID(kP,kI,kD);
    }

    public void moveToRotation(Rotation2d rotation)
    {
        turretPID.setSetpoint(rotation.getDegrees() % Constants.turretLimit);
    }

    public Rotation2d autoalign()
    {
        double adjacent = RobotState.getInstance().estimatedPose.getY()  - Constants.FieldConstants.getHub().getY();
        double opposite = RobotState.getInstance().estimatedPose.getX()  - Constants.FieldConstants.getHub().getX();
        double hypotenuse = Math.sqrt(Math.pow(adjacent, 2) + Math.pow(opposite, 2));

        double angle = Math.acos(
          adjacent / hypotenuse
        );

        return new Rotation2d(Math.toDegrees(angle));
    }

    public void update()
    {
        turretIO.setPower
        (
            turretPID.calculate(turretIO.getRotation().getDegrees())
        );

        turretPID.setPID
        (
            Constants.SimConstants.turretP.get(), 
            Constants.SimConstants.turretP.get(), 
            Constants.SimConstants.turretP.get()
        );
    }

    @Override
    public void periodic()
    {
        update();
    }
}
