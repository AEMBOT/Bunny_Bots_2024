package frc.robot.subsystems.pivot;

import static frc.robot.Constants.PivotConstants.*;
import static edu.wpi.first.math.MathUtil.clamp;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase;
import edu.wpi.first.wpilibj.Timer;


import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class PivotIOReal implements PivotIO {
    
    private boolean openLoop = true;
    private final CANSparkMax leadingMotor = new CANSparkMax(pivotLeftMotorID, MotorType.kBrushless);
    private final CANSparkMax followingMotor = new CANSparkMax(pivotRightMotorID, MotorType.kBrushless);
    private TrapezoidProfile.State pivotGoal;
    private TrapezoidProfile.State pivotSetpoint;
    private double lastTime;


    public PivotIOReal() {

        leadingMotor.restoreFactoryDefaults();
        followingMotor.restoreFactoryDefaults();

        Timer.delay(0.25);

        leadingMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        followingMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        leadingMotor.setInverted(pivotLeftMotorInverted);
        followingMotor.setInverted(pivotRightMotorInverted);

        leadingMotor.setSmartCurrentLimit(pivotLeftMotorCurrentLimit);
        followingMotor.setSmartCurrentLimit(pivotRightMotorCurrentLimit);

        pivotEncoder.setPositionOffset(pivotEncoderPositionOffset);

    }

    public void updateInputs(PivotIOInputs inputs) {
        inputs.pivotAbsolutePosition = getAbsoluteEncoderPosition();
        inputs.pivotAppliedVolts = leadingMotor.getAppliedOutput() * leadingMotor.getBusVoltage();
        inputs.pivotCurrentAmps = new double[] {leadingMotor.getOutputCurrent(), followingMotor.getOutputCurrent()};
        inputs.pivotAbsoluteVelocity = leadingMotor.getEncoder().getVelocity();
        inputs.pivotGoalPosition = Units.radiansToDegrees(pivotGoal.position);
        inputs.pivotSetpointPosition = Units.radiansToDegrees(pivotSetpoint.position);
        inputs.pivotSetpointVelocity = pivotSetpoint.velocity;
        inputs.openLoopStatus = openLoop;
    }   

    @Override
    public void setAngle(double angle) {
        openLoop = false;

        pivotGoal = new TrapezoidProfile.State(Units.degreesToRadians(angle), 0);

        pivotSetpoint = 
            pivotProfile.calculate(
            (Timer.getFPGATimestamp() - lastTime > 0.25)
                ? (Timer.getFPGATimestamp() - lastTime)
                : 0.02,
            pivotSetpoint,
            pivotGoal);

        double feedForward = pivotFFModel.calculate(
            pivotGoal.position, 
            0);
        double pidOutput = pivotPIDController.calculate(
            Units.degreesToRadians(
                getAbsoluteEncoderPosition()), 
                pivotGoal.position);

        setMotorVoltage(feedForward + pidOutput);

        lastTime = Timer.getFPGATimestamp();
    }

    @Override
    public void setVoltage(double volts) {
        openLoop = true;
        setMotorVoltage(volts);
    }

    private double getAbsoluteEncoderPosition() {
        return Units.radiansToDegrees(pivotEncoder.getAbsolutePosition() - pivotEncoder.getPositionOffset());
    }

    private void setMotorVoltage(double volts) {
        if (getAbsoluteEncoderPosition() < pivotMinAngle) {
            volts = clamp(volts, -1, Double.MAX_VALUE);
        }
        if (getAbsoluteEncoderPosition() > pivotMaxAngle) {
            volts = clamp(volts, Double.MIN_VALUE, 1);
        }

        leadingMotor.setVoltage(volts);
    }

    @Override
    public void resetProfile() {
        pivotGoal = new TrapezoidProfile.State(getAbsoluteEncoderPosition(), 0);
        pivotSetpoint = new TrapezoidProfile.State(getAbsoluteEncoderPosition(), 0);
    }
}
