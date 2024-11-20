// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final double UPDATE_PERIOD = 0.2;

  public static final class PivotConstants { 
    /** Maximum angle for the pivot to move to, in degrees */
    public static final double pivotMaxAngle = 90;
    /** Minimum angle for the pivot to move to, in degrees */
    public static final double pivotMinAngle = 0;
    /** ID of the left pivot sparkmax */
    public static final int pivotLeftMotorID = 0;
    /**  */
    public static final boolean pivotLeftMotorInverted = false;
    /**  */
    public static final int pivotLeftMotorCurrentLimit = 60;
    /** ID of the right pivot sparkmax */
    public static final int pivotRightMotorID = 0;
    /**  */
    public static final boolean pivotRightMotorInverted = false;
    /**  */
    public static final int pivotRightMotorCurrentLimit = 60;
    /**  */
    public static final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(3);
    /**  */
    public static final double pivotEncoderPositionOffset = 4.04433682 / (2 * Math.PI);
    /**  */
    public static final double gearRatio = 93.3333333;
    /**  */
    public static final ArmFeedforward pivotFFModel = new ArmFeedforward(
      0.35, 
      0.35, 
      1.79, 
      0.3);
    /**  */
    public static final PIDController pivotPIDController = new PIDController(
      12, 
      0, 
      0.00);
    /**  */
    public static final TrapezoidProfile pivotProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
      2,
      5));
    /** Ramp Rate of the pivot System ID in volts per second */
    public static final double pivotSysIdRampRate = 0.2;
    /** Setp Voltage of the pivot System ID in volts */
    public static final double pivotSysIdStepVolt = 7;
    /** Timeout of the pivot System ID in volts */
    public static final double pivotSysIdTimeout = 30;
    /** How many degrees the pivot can be off its goal position for it to be sufficient */
    public static final double pivotAngleAllowedDeviance = 1.15;
    /**  */
    public static final Translation3d pivotTranslationFromRobot = new Translation3d(-0.2, 0, 0.255);
    /**  */
    public static final double pivotDefaultAngle = 45;
  }
}
