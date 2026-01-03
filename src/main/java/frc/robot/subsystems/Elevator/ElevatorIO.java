package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    
        @AutoLog
    public static class ArmIOInputs {

        public double voltage = 0;

        public double current = 0;
    
        public double position = 0;
    
        public double velocity = 0;
    
        public float goal = 0;

        public Object motorConnected;
    
    }


    public default void getPositionDegrees() {}

    public default void updateInputs(ArmIOInputs armIOInputs) {}

    public default void update(ArmIOInputs armIOInputs) { }
    
    //position is in radians
    public default void runToPosition(double position) {}

    public default void setVoltage() {}

    public default void configurePIDF
    (
        double kP, double kI, double kD, 
        double kS, double kG, double kV, double kA
    ) 
        
    {

    }

}
