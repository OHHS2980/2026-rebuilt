package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;

import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

public class VisionIOSim implements VisionIO {
    
    public PhotonCameraSim sim;

    public VisionIOSim()
    {
        sim = new PhotonCameraSim(
            new PhotonCamera(null)
        );
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) 
    {

    }

}
