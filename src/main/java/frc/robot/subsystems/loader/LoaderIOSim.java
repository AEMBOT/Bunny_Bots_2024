package frc.robot.subsystems.loader;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LoaderIOSim implements LoaderIO {
    private final DCMotorSim motorSim = new DCMotorSim(DCMotor.getNEO(2), 1, 0.01);
    private final DIOSim beamBreak = new DIOSim(0);

    private double motorAppliedVolts = 0.0;
    
    public LoaderIOSim() {
        new Trigger(() -> motorSim.getAngularVelocityRPM() > 1.0)
            .debounce(0.2)
            .onTrue(Commands.runOnce(() -> beamBreak.setValue(true)));
        new Trigger(() -> motorSim.getAngularVelocityRPM() < -1.0) 
            .debounce(0.2)
            .onTrue(Commands.runOnce(() -> beamBreak.setValue(false)));
    }

    @Override
    public void updateInputs(LoaderIOInputs inputs) {
        motorSim.update(0.02);

        inputs.beamBreakState = !beamBreak.getValue();
        inputs.motorAppliedVolts = motorAppliedVolts;
        inputs.motorCurrentAmps = motorSim.getCurrentDrawAmps();
    }

    public void setMotorVoltage(double voltage) {
        motorSim.setInputVoltage(voltage);
        motorAppliedVolts = voltage;
    }
}
