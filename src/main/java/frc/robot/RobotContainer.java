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

import static frc.robot.Constants.currentRobot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.List;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.loader.LoaderIO;
import frc.robot.subsystems.loader.LoaderIOSparkMax;
import frc.robot.subsystems.loader.LoaderIOSim;
import frc.robot.subsystems.loader.Loader;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOReal;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOReal;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.commands.DriveCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private final Pivot pivot;
    private final Loader loader;
    private final Vision vision;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                drive = new Drive(
                  new GyroIONavX(),
                  new ModuleIOTalonFX(0),
                  new ModuleIOTalonFX(1),
                  new ModuleIOTalonFX(2),
                  new ModuleIOTalonFX(3)
                );
                // Ensures that the pivot and loader subsystems arent used while running on lightcycle
                pivot = new Pivot(switch (currentRobot) {
                                    case BUNNYBOT -> new PivotIOReal();
                                    case LIGHTCYCLE -> new PivotIO() {};
                                  });
                loader = new Loader(switch (currentRobot) {
                                    case BUNNYBOT -> new LoaderIOSparkMax();
                                    case LIGHTCYCLE -> new LoaderIO() {};
                                  });
                vision = new Vision(new VisionIOReal());
                break;
            
            case SIM:
                drive = new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim()
                  );
                pivot = new Pivot(new PivotIOSim());
                loader = new Loader(new LoaderIOSim());
                vision = new Vision(new VisionIOSim());
                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                  new GyroIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {},
                  new ModuleIO() {}
                );
                pivot = new Pivot(new PivotIO() {});
                loader = new Loader(new LoaderIO() {});
                vision = new Vision(new VisionIO() {});
                break;
        }

        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        /* AUTO */
        PathConstraints autoConstraints = new PathConstraints(5, 2.5, Math.PI, 1); // Arbitrary. Probably adjust.

        // Mobility (Useless =( )
        List<Translation2d> mobilityAutoWaypoints = PathPlannerPath.bezierFromPoses(
          new Pose2d(5.0, 4.0, Rotation2d.fromDegrees(0))
        );

        PathPlannerPath mobilityAutoPath = new PathPlannerPath(
          mobilityAutoWaypoints,
          autoConstraints,
          new GoalEndState(0.0, null)
        );

        NamedCommands.registerCommand(
            "MobilityAuto",
            Commands.runOnce(() -> drive.setPose(new Pose2d(7, 4, new Rotation2d(Math.PI))))
            .andThen(AutoBuilder.followPath(mobilityAutoPath))
          );

        // Relative to blue, upper = left and lower = right
        // Upper 1st Tote
        NamedCommands.registerCommand(
          "UpperFirstToteAuto",
          autoBunnyCommand(new Pose2d(1.7, 6, Rotation2d.fromDegrees(0)), autoConstraints, false)
        );
        // Lower 1st Tote
        NamedCommands.registerCommand(
          "LowerFirstToteAuto",
          autoBunnyCommand(new Pose2d(1.7, 2, Rotation2d.fromDegrees(0)), autoConstraints, true)
        );
        // Upper 2nd
        NamedCommands.registerCommand(
          "UpperSecondToteAuto",
          autoBunnyCommand(new Pose2d(4.5, 6, Rotation2d.fromDegrees(0)), autoConstraints, false)
        );
        // Lower 2nd
        NamedCommands.registerCommand(
          "LowerSecondToteAuto",
          autoBunnyCommand(new Pose2d(4.5, 2, Rotation2d.fromDegrees(0)), autoConstraints, true)
        );
        // Upper 3rd
        NamedCommands.registerCommand(
          "UpperThirdToteAuto",
          autoBunnyCommand(new Pose2d(7, 6, Rotation2d.fromDegrees(0)), autoConstraints, false)
        );
        // Lower 3rd
        NamedCommands.registerCommand(
          "LowerThirdToteAuto",
          autoBunnyCommand(new Pose2d(7, 2, Rotation2d.fromDegrees(0)), autoConstraints, true)
        );

        // Add autos to autoChooser
        autoChooser.addOption("Upper 1st Tote", NamedCommands.getCommand("UpperFirstToteAuto"));
        autoChooser.addOption("Lower 1st Tote", NamedCommands.getCommand("LowerFirstToteAuto"));
        autoChooser.addOption("Upper 2nd Tote", NamedCommands.getCommand("UpperSecondToteAuto"));
        autoChooser.addOption("Lower 2nd Tote", NamedCommands.getCommand("LowerSecondToteAuto"));
        autoChooser.addOption("Upper 3rd Tote", NamedCommands.getCommand("UpperThirdToteAuto"));
        autoChooser.addOption("Lower 3rd Tote", NamedCommands.getCommand("LowerThirdToteAuto"));

        // Configure the button bindings
        configureButtonBindings();
    }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
   // loader.setDefaultCommand(loader.intakeCommand());
    pivot.setDefaultCommand(pivot.getDefault());

    controller.rightBumper().whileTrue(loader.ejectCommand())
    .onFalse(loader.stopCommand());
    controller.leftBumper().whileTrue(loader.intakeCommand())
    .onFalse(loader.stopCommand());

    drive.setDefaultCommand(
      DriveCommands.joystickDrive(
          drive,
          () -> -controller.getLeftY(),
          () -> -controller.getLeftX(),
          () -> -controller.getRightX(),
          () -> controller.getLeftTriggerAxis() > 0.5)); // Trigger locks make trigger boolean, rather than analog.

    // Manual Pivot Movement
    controller
      .povUp()
      .whileTrue(pivot.changeGoalPosition(20))
      .whileFalse(pivot.changeGoalPosition(0));

    controller
      .povDown()
      .whileTrue(pivot.changeGoalPosition(-20))
      .whileFalse(pivot.changeGoalPosition(0));

  }

  /**
   * Returns a PathPlanner path to the nearest tote.
   * @param constraints
   * @param side The side of the field we're on. `false` is upper and `true` is lower.
   * @return a PathPlanner path to the nearest tote.
   */
  public PathPlannerPath pathToClosestTote(PathConstraints constraints, Boolean side) {
    Transform3d toteDistance = vision.getTransformToClostestToteTag();

    Rotation2d alignRotation = side ? Rotation2d.fromDegrees(-90) : Rotation2d.fromDegrees(90);

    List<Translation2d> waypoint = PathPlannerPath.bezierFromPoses(
          new Pose2d(toteDistance.getX(), toteDistance.getY(), alignRotation)
    );

    PathPlannerPath path = new PathPlannerPath(
      waypoint,
      constraints,
      new GoalEndState(0, alignRotation)
    );
    path.preventFlipping = true;

    return path;
  }

  /**
   * Returns a command to automatically drop off a bunny at a given tote.
   * @param toteApproachPoint A point near the tote. From this point we will path to the nearest tote.
   * @param constraints PathConstraints
   * @param side The side of the field we're on. `false` is upper and `true` is lower.
   * @return a command to automatically drop off a bunny at a given tote.
   */
  public Command autoBunnyCommand(Pose2d toteApproachPoint, PathConstraints constraints, Boolean side) {
    PathPlannerPath approachPath = new PathPlannerPath(
          PathPlannerPath.bezierFromPoses(toteApproachPoint),
          constraints,
          new GoalEndState(0, new Rotation2d(90))
    );

    return loader.intakeCommand() // Just in case.
      .andThen(pivot.setPositionCommand(null)) // TODO: Make angle not null
      .andThen(AutoBuilder.followPath(approachPath))
      .andThen(() -> AutoBuilder.followPath(pathToClosestTote(constraints, side)))
      .andThen(loader.ejectCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
