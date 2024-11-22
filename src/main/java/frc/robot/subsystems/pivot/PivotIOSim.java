package frc.robot.subsystems.pivot;

import static frc.robot.Constants.PivotConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotIOSim implements PivotIO {

    private boolean openLoop = false;
    private double appliedVolts = 0.0;
    private final SingleJointedArmSim sim = pivotSim;

    private final
        
    
    public PivotIOSim() {
        pivotGoal = new ExponentialProfile(pivotSimGoalPosition, 0);
        pivotSetpoint = new ExponentialProfile(pivotSimSetpointPosition, 0);
    }

    public void updateInputs(PivotIOInputs inputs) {}

}
