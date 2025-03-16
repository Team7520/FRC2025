package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {
    public static class RampConstants {
        public static final int RampID = 43;
    }

    public static final class TagCoods {
        public TagCoods(double midX, double midY, double rightX, double rightY, double leftX, double leftY, Rotation2d angle) {
            LeftX = leftX;
            LeftY = leftY;
            RightX = rightX;
            RightY = rightY;
            BotAngle = angle;
            MidX = midX;
            MidY = midY;
        }
        public double LeftX = -1;
        public double LeftY = -1;
        public double RightX = -1;
        public double RightY = -1;
        public Rotation2d BotAngle;
        public double MidX = -1;
        public double MidY = -1;
    }

    public static final class AutoMoveConstants {
        public static final double a = 0.09; // 0.2 // 0.305 0.185
        public static final double b = 0.175; //0.165
        public static final double c = a /2;
        public static final double d = Math.sqrt(3) * (a/2);
        public static final double e = b / 2;
        public static final double f = Math.sqrt(3) * (b/2);
    }

    public static class ElevatorConstants {
        public static final int LIMIT_SWITCH_ID = 9;
        public static final int LEFT_MOTOR_ID = 41;
        public static final int RIGHT_MOTOR_ID = 42;
        public static final double SENSOR_TO_MECHANISM_RATIO = 4d/11d; // Units will be in inches
        public static final double MAX_HEIGHT = 60; // 60 inches

        // PID Constants
        public static final double kP = 0.45; // 0.675
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kIz = 0.0;
        public static final double kFF = 0.09;
        public static final double kG = 0.312;
        public static final double kA = 0.002;

        // Motion Magic Constants
        public static final double MAX_VELOCITY = 40000; // 10 inches per second
        public static final double MAX_ACCELERATION = MAX_VELOCITY*1.5; // 20 inches per second squared
        public static final double MAX_JERK = MAX_ACCELERATION; // 60 inches per second cubed
        public static final double ALLOWABLE_ERROR = 0.5; // 0.5 inches

        public static final int CURRENT_LIMIT = 200;
        public static enum ElevatorPosition {
            GROUND(0),
            LOW(11.41455078125),
            MID(26.8),
            HIGH(53), //52
            LOWALG(18.9),
            HIGHALG(34.75);
            // INTAKE(1.7197265625); old elev pos for intake at centennial, unused with new mechanical changes

            private final double height;

            ElevatorPosition(double height) {
                this.height = height;
            }

            public double getHeight() {
                return height;
            }
        }
    }

    public static class EndEffectorConstants {
        public static final int PIVOT_ID = 23;
        public static final int CONVEYOR_ID = 20;
        public static final double SENSOR_TO_MECHANISM_RATIO = 14.0625;
        public static final double MAX_ANGLE = 0;
        public static final double MIN_ANGLE = -220;

        // PID Constants
        public static final double kP = 0.03;
        public static final double kI = 0;
        public static final double kD = 0.01;
        public static final double kIz = 0.0;
        public static final double kFF = 0.0;

        public static final double kP_CONVEYOR = 0.2;
        public static final double kI_CONVEYOR = 0.0;
        public static final double kD_CONVEYOR = 0.0;

        // MAX motion constants
        public static final double MAX_VELOCITY = 30000;
        public static final double MAX_ACCELERATION = MAX_VELOCITY*1.5;
        public static final double MAX_JERK = MAX_ACCELERATION*3;
        public static final double ALLOWABLE_ERROR = 0.5;
        // Old values (starting from floor)
        public static enum PivotPosition {
            UP(-246),
            DOWN(-190),
            DUNK(-122),
            ALG(-128.57);

            private final double angle;

            PivotPosition(double angle) {
                this.angle = angle;
            }

            public double getAngle() {
                return angle;
            }
        }

        // public static enum PivotPosition {
        //     UP(30),
        //     DOWN(55),
        //     DUNK(123),
        //     ALG(116.43);

        //     private final double angle;

        //     PivotPosition(double angle) {
        //         this.angle = angle;
        //     }

        //     public double getAngle() {
        //         return angle;
        //     }
        // }

        // Current Limiting Constants
        public static final int PIVOT_CURRENT_LIMIT = 10;
        public static final int CONVEYOR_CURRENT_LIMIT = 40;


    }

    public static class TuskConstants {
        public static final int PIVOT_ID = 24;
        public static final double SENSOR_TO_MECHANISM_RATIO = 25;
        public static final double MAX_ANGLE = 0;
        public static final double MIN_ANGLE = -220;

        public static final double kP = 0.01;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kIz = 0.0;
        public static final double kFF = 0.0;

        public static final double MAX_VELOCITY = 20000;
        public static final double MAX_ACCELERATION = MAX_VELOCITY*1.5;
        public static final double MAX_JERK = MAX_ACCELERATION*3;
        public static final double ALLOWABLE_ERROR = 0.5;
        public static enum PivotPosition {
            UP(0),
            DOWN(126.78);

            private final double angle;

            PivotPosition(double angle) {
                this.angle = angle;
            }

            public double getAngle() {
                return angle;
            }
        }

        // Current Limiting Constants
        public static final int PIVOT_CURRENT_LIMIT = 30;
        public static final int CONVEYOR_CURRENT_LIMIT = 40;
    }
}
