// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = 49.8952; // Mass in kilos
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(16);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 0.75;
  }
  public static class ShooterConstants {
    public static final int shooterBotID = 20;
    public static final int shooterTopID = 19;
    public static final int ampMechID = 25;
    public static final double kP = 0.002;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.000156;

    public static final double MAX_RPM = 5676;

    public static final int colourSensorSensedProximity = 145;

    public enum Position {
        // Use Rotation2d.fromDegrees for more intuitive degree handling
        REST(0, 0), // DO NOT CHANGE
        REST180(0, 180),
        DRIVE(0, -95),
        SUBWOOFERCENTER(77.6, 180),
        SUBWOOFERLEFT(77.6, 95),
        SUBWOOFERRIGHT(77.6, -95),
        WINGLINERED(35, 42.3),
        WINGLINEBLUE(35, -42.3),
        PODIUMRED(44.9316, 25),
        PODIUMBLUE(44.9316, -25),
        NOTECENTER(51.9326, 0),
        AMP(72.4453, 0);

        private final Rotation2d pivot;
        private final Rotation2d traverse;

        // Overloaded constructor to accept doubles in degrees
        Position(double pivotDegrees, double traverseDegrees) {
            this.pivot = Rotation2d.fromDegrees(pivotDegrees);
            this.traverse = Rotation2d.fromDegrees(traverseDegrees);
        }
    }


    public static class PivotConstants {
        public static final int CAN_ID = 21;

        public static final double gearRatio = 80.0/3;
        public static final double degreeConversionFactor = 1/gearRatio;
        public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
        public static final double kP = 0.75;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kG = 0;
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;

        public static final double motionMagicVelocity = 450;
        public static final double motionMagicAccel = 800;
        public static final double motionMagicJerk = 2000;
        public static double currentLimit = 60;
        public static final double maxManualSpeed = 0.15;
    }

    public static class TraverseConstants {
        public static final int CAN_ID = 16;

        public static final double gearRatio = 14/160.0;
        public static final double degreeConversionFactor = 1/(gearRatio);
        public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
        public static final double kP = 75; 
        public static final double kI = 2;
        public static final double kD = 0;
        public static final double kG = 0;
        public static final double kS = 0;
        public static final double kV = 0;
        public static final double kA = 0;

        public static final double motionMagicVelocity = 20;
        public static final double motionMagicAccel = 5;
        public static final double motionMagicJerk = 5;
        public static double currentLimit = 60;
        public static final double maxManualSpeed = 0.1;
    }
}
}