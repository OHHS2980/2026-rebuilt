package frc.robot.subsystems.Indexing;

import org.littletonrobotics.junction.AutoLog;


public interface IndexingIO {

    @AutoLog
    class IndexIOInputs {
      
      public boolean motorConnected = false;
      public double indexVoltage = 0.0;
      public double indexCurrent = 0.0;
      public double indexPosition = 0.0;
      public double indexVeloctiy = 0.0;
      public double indexTempC = 0.0;
  
    }
  
    public default void updateInputs(IndexIOInputs inputs) {}
  
    public default void setVoltage(double voltage) {}
  }