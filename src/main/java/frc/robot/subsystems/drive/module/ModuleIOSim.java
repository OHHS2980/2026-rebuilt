package frc.robot.subsystems.drive.module;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ModuleIOSim implements ModuleIO {
    
    public SwerveModuleSimulation moduleSim;

    public SwerveModuleSimulationConfig config;

    private final SimulatedMotorController.GenericMotorController driveMotor;

    private final SimulatedMotorController.GenericMotorController turnMotor;

    

    public SimpleMotorFeedforward feedforward;

    public PIDController turnPID;

    public Rotation2d turnSetpoint;

    public int moduleNumber;

    public ModuleIOSim(SwerveModuleSimulation moduleSim, int moduleNumber)
    {
        feedforward = new SimpleMotorFeedforward(0.1, 0.15);

        config = new SwerveModuleSimulationConfig
        (
            DCMotor.getNEO(1), // Drive motor (neo?)
            DCMotor.getNEO(1), // Steer motor (neo)
            Constants.driveGearRatio, // Drive motor gear ratio
            Constants.turnGearRatio, // Steer motor gear ratio
            Volts.of(0.1), // Drive friction voltage.
            Volts.of(0.1), // Steer friction voltage
            Inches.of(2), // Wheel radius
            KilogramSquareMeters.of(0.03), // Steer MOI
            1.2
        ); // Wheel COF
        

        moduleSim = new SwerveModuleSimulation(config);

        driveMotor = moduleSim.useGenericMotorControllerForDrive();
        turnMotor = moduleSim.useGenericControllerForSteer();

        this.moduleSim = moduleSim;
        this.moduleNumber = moduleNumber;

    }

    public void updateInputs(ModuleIOInputs inputs) 
    {
        
    }


    public void setDriveVoltage(double output) 
    {
        driveMotor.requestVoltage(Voltage.ofBaseUnits(output, Volts));
    }

    public void setTurnVoltage(double output) 
    {
        turnMotor.requestVoltage(Voltage.ofBaseUnits(output, Volts));
    }

    public void setDriveVelocity(double velocityRadPerSec)
    {
        driveMotor.requestVoltage(
            Voltage.ofBaseUnits( 
                feedforward.calculate(velocityRadPerSec), Volts
            )
        );
    }

    public void setTurnPosition(Rotation2d rotation)
    {
        
    }

    public Rotation2d getTurnDegrees()
    {
        return new Rotation2d(moduleSim.getSteerAbsoluteAngle());
    }

    public Distance getDriveDistance()
    {
        return Distance.ofBaseUnits(

            moduleSim.getDriveWheelFinalPosition().baseUnitMagnitude()
            / 6 * Math.PI 
            * Constants.swerveWheelCircumference
        
            , Meter);
    }
}
