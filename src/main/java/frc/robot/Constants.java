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

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
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

  public static final class VisionConstants {

    // List of static april tags on the field, not the ones on the totes
    // Field layout can also be imported from a json file if we decide to do that
    // TODO get exact values of april tag positions
    private static final List<AprilTag> fieldAprilTags = List.of(
      new AprilTag(13, 
        new Pose3d(0, 0, 0, 
          new Rotation3d(0, 0, 0))),
     new AprilTag(14, 
        new Pose3d(0, 0, 0, 
          new Rotation3d(0, 0, 0))),
     new AprilTag(15, 
        new Pose3d(0, 0, 0, 
          new Rotation3d(0, 0, 0))),
      new AprilTag(16, 
        new Pose3d(0, 0, 0, 
          new Rotation3d(0, 0, 0))),
     new AprilTag(17, 
        new Pose3d(0.0, 0.0, 0.0, 
          new Rotation3d(0.0, 0.0, 0.0))),
     new AprilTag(18, 
        new Pose3d(0, 0, 0, 
          new Rotation3d(0, 0, 0))),
     new AprilTag(19, 
        new Pose3d(0, 0, 0, 
          new Rotation3d(0, 0, 0))),
     new AprilTag(20, 
        new Pose3d(0, 0, 0, 
          new Rotation3d(0, 0, 0))));

    // All information pertaining to the front camera
    // TODO get distance from cam to center of bot
    public static final String frontCamName = "front";
    public static final Transform3d frontCamToRobot = new Transform3d(
      new Translation3d(0, 0, 0),
      new Rotation3d(0, 0, 0));

    // April tag field layout, inclues the positions of all static april tags as well as the size of the field
    // TODO confirm that length is 54 and width is 27 and not vice versa
    public static final AprilTagFieldLayout aprilTagFieldLayout =
    new AprilTagFieldLayout(fieldAprilTags, 54, 27);

  }
}
