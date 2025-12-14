package frc.robot.subsystems.outtake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import static frc.robot.util.SparkUtil.*;

public class OuttakeMotor implements OuttakeIO{
    SparkMax outtakeMotor;
    RelativeEncoder encoder;

    public OuttakeMotor() {
        outtakeMotor = new SparkMax(0, null);
        encoder = outtakeMotor.getEncoder();
    }

    public void UpdateInputs(OuttakeIOInputs inputs) {

        ifOk(outtakeMotor, () -> outtakeMotor.getBusVoltage(), (value) -> inputs.outtakeVoltage = value);
        ifOk(outtakeMotor, () -> outtakeMotor.getOutputCurrent(), (value) -> inputs.outtakeCurrent = value);
        ifOk(outtakeMotor, () -> encoder.getPosition(), (value) -> inputs.outtakePosition = value);
        ifOk(outtakeMotor, () -> encoder.getVelocity(), (value -> inputs.outtakeVeloctiy = value));
        ifOk(outtakeMotor, () -> outtakeMotor.getMotorTemperature(), (value) -> inputs.outtakeTempC = value);

        inputs.motorConnected = false;
    }

    @Override
    public void setVoltage(double voltage) {
      outtakeMotor.setVoltage(voltage);
    }
}
