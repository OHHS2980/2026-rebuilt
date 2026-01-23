package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.Hood.Hood;
import frc.robot.subsystems.shooter.Hood.HoodIO;
import frc.robot.subsystems.shooter.Turret.Turret;
import frc.robot.subsystems.shooter.Turret.TurretIO;

public class Shooter extends SubsystemBase {
    
    Timer timer;

    public Hood hood;

    public Turret turret;
    
    public Shooter(
        TurretIO turretIO,
        HoodIO hoodIO,

        double turretP,
        double turretI,
        double turretD,

        double hoodP,
        double hoodI,
        double hoodD
    )
    {
        hood = new Hood(hoodIO, hoodP, hoodI, hoodD);

        turret = new Turret(turretIO, turretP, turretI, turretD, null);

        timer = new Timer();
        timer.start();
    }

    @Override
    public void periodic()
    {
        turret.update(timer);

        turret.autoalign();
    }
}
