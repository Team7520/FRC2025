package frc.robot;

public class Constants {
    public static class ElevatorConstants {
        public static final int LEFT_MOTOR_ID = 41;
        public static final int RIGHT_MOTOR_ID = 42;
        public static final double SENSOR_TO_MECHANISM_RATIO = 27d/11d; // Units will be in inches
        public static final double MAX_HEIGHT = 60; // 60 inches

        // PID Constants
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kIz = 0.0;
        public static final double kFF = 0.0;

        // Motion Magic Constants
        public static final double MAX_VELOCITY = 10; // 10 inches per second
        public static final double MAX_ACCELERATION = 20; // 20 inches per second squared
        public static final double MAX_JERK = 60; // 60 inches per second cubed
        public static final double ALLOWABLE_ERROR = 0.5; // 0.5 inches

        public static enum ElevatorPosition {
            GROUND(0),
            LOW(12),
            MID(36),
            HIGH(58);

            private final double height;

            ElevatorPosition(double height) {
                this.height = height;
            }

            public double getHeight() {
                return height;
            }
        }
    }
}
