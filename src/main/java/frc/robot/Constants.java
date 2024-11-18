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

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants{
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class VisionConstants {

    /**
     * April tag field layout, inclues the positions of all static april tags as well as the size of the field.
     */ 
    public static final AprilTagFieldLayout aprilTagFieldLayout;
    static {
      AprilTagFieldLayout layout = null;
      try{
        layout = new AprilTagFieldLayout(Path.of("src\\main\\java\\frc\\robot\\aprilTagFieldLayout.json"));
      }
      catch (IOException e) {
        e.printStackTrace();
      }
      aprilTagFieldLayout = layout;
  }
    

    // TODO determine these values  
    /** The name that connects the front camera in code to the front camera in photonvision on the rio */
    public static final String frontCamName = "front";
    /** Transform from the center of the robot to the front camera */
    public static final Transform3d frontCamFromRobot = new Transform3d(
      new Translation3d(0, 0, 0),
      new Rotation3d(0, 0, 0));

    // Following Values are only used for sim
    /* *Vertical resolution of the front camera */
    public static final int frontCamVertRes = 1000;
    /** Horizontal resolution of the front camera */
    public static final int frontCamHorizRes = 1000;
    /** Front camera rotation offset. */
    public static final Rotation2d frontCamRotation = Rotation2d.fromDegrees(90);
    /** Front camera pixel error */
    public static final double frontCamPxErr = 0;
    /** Standard Deviation in the pixel error of the front camera */
    public static final double frontCamPxErrStdDev = 0;
    /** How many frames the front camera provides per second */
    public static final double frontCamFPS = 60;
    /** Average latency from the front camera */
    public static final double frontCamAvgLatency = 0;
    /** Standard deviation in the latency fro the front camera */
    public static final double frontCamLatencyStdDev = 0;

  }
}
