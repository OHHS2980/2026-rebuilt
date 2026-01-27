package frc.robot.subsystems.Indexing;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import static frc.robot.util.SparkUtil.*;

public class IndexMotor implements IndexingIO{
    SparkMax indexMotor;
    RelativeEncoder encoder;

    public IndexMotor() {
        indexMotor = new SparkMax(0, null);
        encoder = indexMotor.getEncoder();
    }

    public void updateInputs(IndexIOInputs inputs) {

      inputs.indexVoltage = indexMotor.getBusVoltage();
      inputs.indexCurrent = indexMotor.getOutputCurrent();
      inputs.indexPosition = encoder.getPosition();
        ifOk(indexMotor, () -> indexMotor.getOutputCurrent(), (value) -> inputs.indexCurrent = value);
        ifOk(indexMotor, () -> encoder.getPosition(), (value) -> inputs.indexPosition = value);
        ifOk(indexMotor, () -> encoder.getVelocity(), (value -> inputs.indexVeloctiy = value));
        ifOk(indexMotor, () -> indexMotor.getMotorTemperature(), (value) -> inputs.indexTempC = value);

        inputs.motorConnected = false;
    }

    @Override
    public void setVoltage(double voltage) {
      indexMotor.setVoltage(voltage);
    }
}

