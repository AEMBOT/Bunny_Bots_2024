package frc.robot.subsystems.loader;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static edu.wpi.first.wpilibj.Timer.delay;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.LoaderConstants;

public class LoaderIOSparkMax implements LoaderIO {
    private final CANSparkMax motor = new CANSparkMax(LoaderConstants.MOTOR_PORT, kBrushless);

    public LoaderIOSparkMax() {
        motor.restoreFactoryDefaults();

        motor.setSmartCurrentLimit(LoaderConstants.MOTOR_CURRENT_LIMIT);

        delay(0.25);

        // Recommended by REV in order to ensure that new settings are not lost
        // during a brown-out scenario where the Spark Max loses power but the
        // RoboRio does not
        motor.burnFlash();
    }

    @Override
    public void updateInputs(LoaderIOInputs inputs) {
        inputs.motorAppliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.motorCurrentAmps = motor.getOutputCurrent();
    }

    public void setMotorVoltage(double voltage) {
        motor.setVoltage(voltage);
    }
}
