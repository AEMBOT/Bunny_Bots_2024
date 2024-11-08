package frc.robot.subsystems.loader;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LoaderConstants;

public class Loader extends SubsystemBase {
    
    private final LoaderIO io;
    private final LoaderIOInputsAutoLogged inputs = new LoaderIOInputsAutoLogged();

    public Loader(LoaderIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Loader", inputs);
    }

    /**
     * @return The state of the beam break. `true` if the beam break is obstructed, `false` otherwise.
     */
    public boolean getBeamBreakState() {
        return inputs.beamBreakState;
    }

    /**
     * Runs the motor to pull in balloons at a voltage of `LoaderConstants.MOTOR_VOLTAGE`.
     */
    public void intake() {
        io.setMotorVolts(LoaderConstants.MOTOR_VOLTAGE);
    }

    /**
     * Sets the motor's voltage to zero.
     */
    public void stop() {
        io.setMotorVolts(0.0);
    }

    /**
     * Runs the motor to spit out balloons at a voltage of `-LoaderConstants.MOTOR_VOLTAGE`.
     */
    public void eject() {
        io.setMotorVolts(-LoaderConstants.MOTOR_VOLTAGE);
    }

    public Command intakeCommand() {
        return runOnce(this::intake);
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }

    public Command outtakeCommand() {
        return runOnce(this::eject);
    }
}
