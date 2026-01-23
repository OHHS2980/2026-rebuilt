package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeMotors implements IntakeIO{
    TalonFX intake1;

    public IntakeMotors() {
        intake1 = new TalonFX(0);
    }

    public void intake1UpdateInputs(intake1IOInputs inputs) {
        inputs.intake1Voltage = intake1.getMotorVoltage().getValueAsDouble();
        inputs.intake1Velocity = intake1.getVelocity().getValueAsDouble();
        inputs.intake1Current = intake1.getPosition().getValueAsDouble();
        inputs.intake1TempC = intake1.getDeviceTemp().getValueAsDouble();
        inputs.intake1Connected = false;
    }

    @Override
    public void intake1SetPower(double power) {
        intake1SetPower(power);
    }
}
