package frc.robot.subsystems.vision;
import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
public class Vision extends SubsystemBase {
    VisionIO visionIO;
    public VisionIOInputs visionIOInputs = new VisionIOInputs();

    public Vision(VisionIO visionIO)
    {
        this.visionIO = visionIO;
    }

    @Override
    public void periodic()
    {
        visionIO.updateInputs(null);
    }
}