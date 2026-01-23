package frc.robot.subsystems.Intake;

public interface IntakeIO {
    //intake1: feeds intake
    //intake2: moves intake
    public static class intake1IOInputs {
        public boolean intake1Connected = false;
        public double intake1Voltage = 0.0;
        public double intake1Current = 0.0;
        public double intake1Position = 0.0;
        public double intake1Velocity = 0.0;
        public double intake1TempC = 0.0;
    }

    public void intake1SetPower (double power);
}
