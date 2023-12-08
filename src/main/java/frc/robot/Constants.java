// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double TAU = 2*Math.PI;
  public static final String CANBUS_NAME = "drivet";

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Joystick{
    public static final int RIGHT_JOYSTICK_PORT = 0;
    public static final int LEFT_JOYSTICK_PORT = 1;
    public static final int OPERATOR_PORT = 2;

    public static final double MOVE_DEAD_ZONE = 0.18;
    public static final double MOVE_SENSITIVITY = 1.5;

    public static final double TURN_DEAD_ZONE = 0.1;
    public static final double TURN_SENSITIVITY = 1;

    public static final double ANGLE_DEAD_ZONE = Constants.TAU/36;

    public static final double MANUAL_EXTEND_DEADZONE = 0;
    public static final double MANUAL_PIVOT_DEADZONE = 0;
    public static final double MANUAL_PIVOT_DOWN_SENSITIVITY = 0;
    public static final double MANUAL_PIVOT_UP_SENSITIVITY = 0;
  }

  public static final class Swerve {
    public static final double TURN_GEAR_RATIO = 150. / 7.;
    public static final double MOVE_GEAR_RATIO = 8.14;

    public static final double WIDTH = Units.inchesToMeters(26 - 2.625*2);
    public static final double LENGTH = Units.inchesToMeters(26 - 2.625*2);
    public static final double DIAGONAL = Math.sqrt(WIDTH*WIDTH + LENGTH*LENGTH)/2;

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(WIDTH/2, -LENGTH/2),
      new Translation2d(WIDTH/2, LENGTH/2),
      new Translation2d(-WIDTH/2, LENGTH/2),
      new Translation2d(-WIDTH/2, -LENGTH/2)
    );

    public static final double WHEELRADIUS = Units.inchesToMeters(1.925);
    
    public static final double MAX_NATIVE_VELOCITY = 103;

    public static final int FRONT_RIGHT_TURN_PORT = 4;
    public static final int FRONT_LEFT_TURN_PORT = 5;
    public static final int BACK_LEFT_TURN_PORT = 6;
    public static final int BACK_RIGHT_TURN_PORT = 7;

    public static final int FRONT_RIGHT_MOVE_PORT = 0;
    public static final int FRONT_LEFT_MOVE_PORT = 1;
    public static final int BACK_LEFT_MOVE_PORT = 2;
    public static final int BACK_RIGHT_MOVE_PORT = 3;

    public static final int FRONT_RIGHT_SENSOR_PORT = 0;
    public static final int FRONT_LEFT_SENSOR_PORT = 1;
    public static final int BACK_LEFT_SENSOR_PORT = 2;
    public static final int BACK_RIGHT_SENSOR_PORT = 3;
    
    public static final double FRONT_RIGHT_OFFSET_NU = -0.609619 + 0.5;
    public static final double FRONT_LEFT_OFFSET_NU = -0.535400 + 0.5;
    public static final double BACK_LEFT_OFFSET_NU = -0.447266+0.5;
    public static final double BACK_RIGHT_OFFSET_NU = -0.290527+0.5;

    public static final double TURN_KF = 0.01;
    public static final double TURN_KP = 0.5;
    public static final double TURN_KI = 0;
    public static final double TURN_KD = 0.0;

    public static final double MOVE_KF = 0.01;
    public static final double MOVE_KP = 0.01;
    public static final double MOVE_KI = 0.0;
    public static final double MOVE_KD = 0.0;

    public static final double MOD0_AFF = 0.0;
    public static final double MOD1_AFF = 0.0;
    public static final double MOD2_AFF = 0.0;
    public static final double MOD3_AFF = 0.0;
  
    public static final class Balance{
      public static final double FAST_K_P = 0.01;
      public static final double FAST_K_I = 0.0;
      public static final double FAST_K_D = 0.0;

      public static final double SLOW_K_P = 0.005;
      public static final double SLOW_K_I = 0.0;
      public static final double SLOW_K_D = 0.0;

      // public static final double MAX_SPEED_PERCENT = 0.3;
    }
  }

  public static class Field{
    public static final double K_CARPET = 0;
    public static final Rotation2d ANGLE_OF_RESISTANCE = new Rotation2d(0);
  }

  public static class Misc{
    public static final int GYRO_PORT = 0;
  }
}
