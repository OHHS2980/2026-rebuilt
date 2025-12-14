package frc.robot.subsystems.outtake;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {

  @AutoLog
  class OuttakeIOInputs {
    
    public boolean motorConnected = false;
    public double outtakeVoltage = 0.0;
    public double outtakeCurrent = 0.0;
    public double outtakePosition = 0.0;
    public double outtakeVeloctiy = 0.0;
    public double outtakeTempC = 0.0;

  }

  public default void updateInputs(OuttakeIOInputs inputs) {}

  public default void setVoltage(double voltage) {}
}
