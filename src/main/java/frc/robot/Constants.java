// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  /**
   * This is where we setup all the constants for our swerve modules
   */
  public static final class SDSConstants {
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.2794;
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.2794;

    public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.14528;

    public static final int DRIVETRAIN_PIGEON_ID = 16;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 6;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 5;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 10;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(218.671);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 8;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 9;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(156.269);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 3;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 4;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(91.142);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 1;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 2;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(337.236);
  }

  /**
   * Anything PathPlanner needs for Constraints
   */
  public static final class PathPlannerConstants {
    //Swerve Drive (Holonomic) Pathplanner Configuration
    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
      new PIDConstants(5.0, 0, 0), // Translation constants 
      new PIDConstants(5.0, 0, 0), // Rotation constants 
      SDSConstants.MAX_VELOCITY_METERS_PER_SECOND, 
      new Translation2d(SDSConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, SDSConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0).getNorm(), // Drive base radius (distance from center to furthest module) 
      new ReplanningConfig()
    );
  }
}
