package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PointTowardsZone;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
//import com.revrobotics.AnalogInput;
//import com.revrobotics.spark.SparkFlex;
//import com.pathplanner.lib.path.GoalEndState;
//import com.pathplanner.lib.path.PathConstraints;
//import com.pathplanner.lib.path.PathPlannerPath;
//import com.pathplanner.lib.path.Waypoint;
//import edu.wpi.first.math.filter.SlewRateLimiter;

import edu.wpi.first.math.Matrix;
//import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
//import edu.wpi.first.util.sendable.Sendable;
//import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.TagCoods;
import frc.robot.LimelightHelpers;
//import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    int counter = 0;
    public static boolean UpdatedPose = false;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    public static ArrayList<TagCoods> TagArray = new ArrayList<>();

    public static boolean pathActive = false;
    public static boolean KeepPath = true;

    
    
    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(7, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(3, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    /**
     * Get the autonomous command for the robot.
     * @param autoName       Name of the auto file.
     * @param setOdomToStart Set the odometry position to the start of the path.
     * @return {@link PathPlannerAuto} command.
     */
    public Command getPPAutoCommand(String autoName, boolean setOdomToStart) {
        if (setOdomToStart) {
            SmartDashboard.putNumber("HeadingFromFile", -1);
//            resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile(autoName));
        }
        return new PathPlannerAuto(autoName);
    }

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder(); //14P, 22.5P
        LimeTags();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
        LimeTags();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
        LimeTags();
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public double limitSpeed(double speed, boolean speedCutOff) {
        return speedCutOff ? speed * 0.25 : speed;
    }

    public double changeDeadband(double deadband, boolean change) {
        return change ? deadband * 0.25 : deadband;
    }

    public Command LimelightStatus(boolean update) {
        if(update) {
            SmartDashboard.putBoolean("isWorking?", true);
            return runOnce(() -> UpdatedPose = true);
        } else {
            SmartDashboard.putBoolean("isWorking?", false);
            return runOnce(() -> UpdatedPose = false);
        }
    }

    public void LimeTags() {
        //Distance between sides is 33.02 cm

        //ID 0 Apriltag
        TagArray.add(new TagCoods(-1, -1, -1, -1, -1, -1, Rotation2d.fromDegrees(0))); //Not used for this season
        //ID 1 Apriltag
        TagArray.add(new TagCoods(-1, -1, -1, -1, -1, -1, Rotation2d.fromDegrees(0))); //Not used for this season
        //ID 2 Apriltag
        TagArray.add(new TagCoods(-1, -1, -1, -1, -1, -1, Rotation2d.fromDegrees(0))); //Not used for this season
        //ID 3 Apriltag
        TagArray.add(new TagCoods(-1, -1, -1, -1, -1, -1, Rotation2d.fromDegrees(0))); //Not used for this season
        //ID 4 Apriltag
        TagArray.add(new TagCoods(-1, -1, -1, -1, -1, -1, Rotation2d.fromDegrees(0))); //Not used for this season
        //ID 5 Apriltag
        TagArray.add(new TagCoods(-1, -1, -1, -1, -1, -1, Rotation2d.fromDegrees(0))); //Not used for this season
        //ID 6 Apriltag
        TagArray.add(new TagCoods(13.474 + Constants.AutoMoveConstants.c, 3.306 - Constants.AutoMoveConstants.d, 
                    13.474 + Constants.AutoMoveConstants.c + Constants.AutoMoveConstants.f, 
                    3.306 - Constants.AutoMoveConstants.d + Constants.AutoMoveConstants.e, 
                    13.474 + Constants.AutoMoveConstants.c - Constants.AutoMoveConstants.f, 
                    3.306 - Constants.AutoMoveConstants.d - Constants.AutoMoveConstants.e, Rotation2d.fromDegrees(120)));
        //ID 7 Apriltag
        TagArray.add(new TagCoods(13.89 + Constants.AutoMoveConstants.a, 4.026, 
                    13.89 + Constants.AutoMoveConstants.a, 4.026 + Constants.AutoMoveConstants.b, 
                    13.89 + Constants.AutoMoveConstants.a, 4.026 - Constants.AutoMoveConstants.b, Rotation2d.fromDegrees(180)));
        //ID 8 Apriltag
        TagArray.add(new TagCoods(13.474 + Constants.AutoMoveConstants.c, 4.745 + Constants.AutoMoveConstants.d, 
                    13.474 + Constants.AutoMoveConstants.c - Constants.AutoMoveConstants.f, 
                    4.745 + Constants.AutoMoveConstants.d + Constants.AutoMoveConstants.e, 
                    13.474 + Constants.AutoMoveConstants.c + Constants.AutoMoveConstants.f, 
                    4.745 + Constants.AutoMoveConstants.d - Constants.AutoMoveConstants.e, Rotation2d.fromDegrees(240)));
        //ID 9 Apriltag
        TagArray.add(new TagCoods(12.643 - Constants.AutoMoveConstants.c, 4.745 + Constants.AutoMoveConstants.d, 
                    12.643 - Constants.AutoMoveConstants.c - Constants.AutoMoveConstants.f, 
                    4.745 + Constants.AutoMoveConstants.d - Constants.AutoMoveConstants.e, 
                    12.643 - Constants.AutoMoveConstants.c + Constants.AutoMoveConstants.f, 
                    4.745 + Constants.AutoMoveConstants.d + Constants.AutoMoveConstants.e, Rotation2d.fromDegrees(300)));
        //ID 10 Apriltag
        TagArray.add(new TagCoods(12.227 - Constants.AutoMoveConstants.a, 4.026, 
                    12.227 - Constants.AutoMoveConstants.a, 4.026 - Constants.AutoMoveConstants.b, 
                    12.227 - Constants.AutoMoveConstants.a, 4.026 + Constants.AutoMoveConstants.b, Rotation2d.fromDegrees(0)));
        //ID 11 Apriltag
        TagArray.add(new TagCoods(12.643 - Constants.AutoMoveConstants.c, 3.306 - Constants.AutoMoveConstants.d, 
                    12.643 - Constants.AutoMoveConstants.c + Constants.AutoMoveConstants.f, 
                    3.306 - Constants.AutoMoveConstants.d - Constants.AutoMoveConstants.e, 
                    12.643 - Constants.AutoMoveConstants.c - Constants.AutoMoveConstants.f, 
                    3.306 - Constants.AutoMoveConstants.d + Constants.AutoMoveConstants.e, Rotation2d.fromDegrees(60)));
        //ID 12 Apriltag
        TagArray.add(new TagCoods(-1, -1, -1, -1, -1, -1, Rotation2d.fromDegrees(0))); //Not used for this season
        //ID 13 Apriltag
        TagArray.add(new TagCoods(-1, -1, -1, -1, -1, -1, Rotation2d.fromDegrees(0))); //Not used for this season
        //ID 14 Apriltag
        TagArray.add(new TagCoods(-1, -1, -1, -1, -1, -1, Rotation2d.fromDegrees(0))); //Not used for this season
        //ID 15 Apriltag
        TagArray.add(new TagCoods(-1, -1, -1, -1, -1, -1, Rotation2d.fromDegrees(0))); //Not used for this season
        //ID 16 Apriltag
        TagArray.add(new TagCoods(-1, -1, -1, -1, -1, -1, Rotation2d.fromDegrees(0))); //Not used for this season
        //ID 17 Apriltag
        TagArray.add(new TagCoods(4.074 - Constants.AutoMoveConstants.c, 3.306 - Constants.AutoMoveConstants.d, 
                    4.074 - Constants.AutoMoveConstants.c + Constants.AutoMoveConstants.f, 
                    3.306 - Constants.AutoMoveConstants.d - Constants.AutoMoveConstants.e, 
                    4.074 - Constants.AutoMoveConstants.c - Constants.AutoMoveConstants.f, 
                    3.306 - Constants.AutoMoveConstants.d + Constants.AutoMoveConstants.e, Rotation2d.fromDegrees(60)));
        //ID 18 Apriltag
        TagArray.add(new TagCoods(3.658 - Constants.AutoMoveConstants.a, 4.026, 
                    3.658 - Constants.AutoMoveConstants.a, 4.026 - Constants.AutoMoveConstants.b, 
                    3.658 - Constants.AutoMoveConstants.a, 4.026 + Constants.AutoMoveConstants.b, Rotation2d.fromDegrees(0)));
        //ID 19 Apriltag
        TagArray.add(new TagCoods(4.074 - Constants.AutoMoveConstants.c, 4.745 + Constants.AutoMoveConstants.d, 
                    4.074 - Constants.AutoMoveConstants.c - Constants.AutoMoveConstants.f, 
                    4.745 + Constants.AutoMoveConstants.d - Constants.AutoMoveConstants.e, 
                    4.074 - Constants.AutoMoveConstants.c + Constants.AutoMoveConstants.f, 
                    4.745 + Constants.AutoMoveConstants.d + Constants.AutoMoveConstants.e, Rotation2d.fromDegrees(300)));
        //ID 20 Apriltag
        TagArray.add(new TagCoods(4.905 + Constants.AutoMoveConstants.c, 4.745 + Constants.AutoMoveConstants.d, 
                    4.905 + Constants.AutoMoveConstants.c - Constants.AutoMoveConstants.f, 
                    4.745 + Constants.AutoMoveConstants.d + Constants.AutoMoveConstants.e, 
                    4.905 + Constants.AutoMoveConstants.c + Constants.AutoMoveConstants.f, 
                    4.745 + Constants.AutoMoveConstants.d - Constants.AutoMoveConstants.e, Rotation2d.fromDegrees(240)));
        //ID 21 Apriltag
        TagArray.add(new TagCoods(5.321 + Constants.AutoMoveConstants.a, 4.026, 
                    5.321 + Constants.AutoMoveConstants.a, 4.026 + Constants.AutoMoveConstants.b, 
                    5.321 + Constants.AutoMoveConstants.a, 4.026 - Constants.AutoMoveConstants.b, Rotation2d.fromDegrees(180)));
        //ID 22 Apriltag
        TagArray.add(new TagCoods(4.905 + Constants.AutoMoveConstants.c, 3.306 - Constants.AutoMoveConstants.d, 
                    4.905 + Constants.AutoMoveConstants.c + Constants.AutoMoveConstants.f, 
                    3.306 - Constants.AutoMoveConstants.d + Constants.AutoMoveConstants.e, 
                    4.905 + Constants.AutoMoveConstants.c - Constants.AutoMoveConstants.f, 
                    3.306 - Constants.AutoMoveConstants.d - Constants.AutoMoveConstants.e, Rotation2d.fromDegrees(120)));

        LimelightHelpers.setCameraPose_RobotSpace("", 
        -0.181,    // Forward offset (meters)
            0,    // Side offset (meters)
            0,    // Height offset (meters)
            0,    // Roll (degrees)
            20,   // Pitch (degrees)
            0     // Yaw (degrees)
        );

        //sparkFlex = new SparkFlex(20, SparkFlex.MotorType.kBrushless);

        // Initialize Analog Input (Pin 3 on Data Port)
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    public static double deadband(double value) {
        return Math.abs(value) > 0.05 ? value : 0;
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );  
                m_hasAppliedOperatorPerspective = true;
            });
        }
        //System.out.printf("updatedPose X is: %f updatedPose Y is: %f updatedPose Z is: %f\n", LimelightHelpers.getBotPose2d_wpiBlue("").getX(), LimelightHelpers.getBotPose2d_wpiBlue("").getY(), LimelightHelpers.getBotPose2d_wpiBlue("").getRotation().getDegrees());
        if(LimelightHelpers.getBotPose2d_wpiBlue("") == null) {
            System.out.println("NULLLLLLLLSGADUFAYFYFGWY*HFUHFYWFYFYWYWFVUFUSJFHJSFHJFHUISHFUUIFHJSHFJFHJSHF\n");
        }
        SmartDashboard.putBoolean("Is Lime Updating?", !UpdatedPose);
        if(LimelightHelpers.getTV("")) {
            SmartDashboard.putBoolean("Limelight Can See?", true);
        } else {
            SmartDashboard.putBoolean("Limelight Can See?", false);
        }
        if(LimelightHelpers.getFiducialID("") == 0) {
            SmartDashboard.putBoolean("Lime Connected?", false);
        } else {
            SmartDashboard.putBoolean("Lime Connected?", true);
        }
        if(UpdatedPose) {
            Pose2d updatedPose = LimelightHelpers.getBotPose2d_wpiBlue("");
            if(updatedPose == null) {
                System.out.println("updatedPose is Null!! _____________________________________________________________\n");
                return;
            }
            if(updatedPose.getRotation().getCos() == 0.0 && updatedPose.getRotation().getSin() == 0.0) {
                System.out.println("Rotation is invalid!! _____________________________________________________________\n");
                return;
            }
            if (updatedPose.getX() != 0.0 && updatedPose.getY() != 0.0 && updatedPose.getRotation().getDegrees() != 0.0 && counter > 0) {
                counter = 0;
                resetPose(updatedPose);
                //System.out.printf("Updated X:%f Updated Y:%f, Updated Rotate:%f\n", updatedPose.getX(), updatedPose.getY(), updatedPose.getRotation().getDegrees());
            } else {
                counter++;
                //System.out.println("ran in null");
            }   
        }  
        SmartDashboard.putNumber("RobotX_POSE", getState().Pose.getX());
        SmartDashboard.putNumber("RobotY_POSE", getState().Pose.getY());
        SmartDashboard.putNumber("Apriltag ID", LimelightHelpers.getFiducialID(""));
        SmartDashboard.putNumber("Gyro Angle", getState().Pose.getRotation().getDegrees());   
        SmartDashboard.putNumber("Velocity", Math.sqrt(Math.pow(getState().Speeds.vxMetersPerSecond, 2) + Math.pow(getState().Speeds.vyMetersPerSecond, 2)));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    public PathPlannerPath GoRight(int mode) {
        double id = LimelightHelpers.getFiducialID(""); 
        Pose2d updatedPose = LimelightHelpers.getBotPose2d_wpiBlue("");  

        /*Failsafes */
        if(updatedPose == null || (updatedPose.getX() == 0.0 && updatedPose.getY() == 0.0 && updatedPose.getRotation().getDegrees() == 0.0)) {
            System.out.println("Pose was Null or zero when making path!! Gonna cancel path now\n");
            mode = -1;
        } 
        if(id < 1 || id > 22) {
            System.out.printf("ID wasn't within proper range!! The ID was: %f\n", id);
            mode = -1;
        }
        if(LimelightHelpers.getTV("") == false) {
            System.out.println("Limelight Could not see target after pressing!!\n");
            mode = -1;
        }
        if(updatedPose.getRotation().getCos() == 0.0 && updatedPose.getRotation().getSin() == 0.0) {
            System.out.println("Rotation was invalid while creating path!!\n");
            mode = -1;
        }

        /*Cancel Path OR Failsafe Met */
        if(mode == -1) {
            List<Waypoint> wayPoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(getState().Pose.getX(), getState().Pose.getY(), getState().Pose.getRotation()),
                new Pose2d(getState().Pose.getX() + 0.01, getState().Pose.getY(), getState().Pose.getRotation())
            );
                
            PathConstraints constraints = new PathConstraints(0.5, 0.5, 2 * Math.PI, 2 * Math.PI); // The constraints for this path.
            
            PathPlannerPath path = new PathPlannerPath( 
                wayPoints,
                constraints,
                null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                new GoalEndState(0.0, getState().Pose.getRotation()) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );
                
            path.preventFlipping =true;
            pathActive = false;
            UpdatedPose = false;
            return path;
        } 

        /*Failsafes NOT Met, Auto Align */
        pathActive = true;
        UpdatedPose = true;
            
        resetPose(updatedPose);
        List<Waypoint> wayPoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(getState().Pose.getX(), getState().Pose.getY(), getState().Pose.getRotation()),
            new Pose2d(TagArray.get((int)id).RightX, TagArray.get((int)id).RightY, TagArray.get((int)id).BotAngle)
        );        

        PathConstraints constraints = new PathConstraints(2, 2, 2 * Math.PI, 2 * Math.PI); // The constraints for this path.
                
        EventMarker signalEnd = new EventMarker("ChangeBool", 0.95, -1, new InstantCommand(() -> {pathActive = false;})); // THIS COMMAND IS TERMINATED WHEN THE PATH ENDS
        EventMarker turnofflie = new EventMarker("ChangeLime", 0.99, -1, new InstantCommand(() -> {UpdatedPose = false;}));
        List<EventMarker> lst_em = Arrays.asList(signalEnd, turnofflie);
        List<RotationTarget> lst_rt = Arrays.asList();
        List<ConstraintsZone> lst_cz = Arrays.asList();
        List<PointTowardsZone> lst_ptz = Arrays.asList();

        PathPlannerPath path = new PathPlannerPath( 
            wayPoints,
            lst_rt,
            lst_ptz, 
            lst_cz,
            lst_em,
            constraints,
            null, 
            new GoalEndState(0.0, TagArray.get((int)id).BotAngle), // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            false
        );
        
        path.preventFlipping =true;
        return path;    
    } 

    public PathPlannerPath GoLeft(int mode) {
        double id = LimelightHelpers.getFiducialID(""); 
        Pose2d updatedPose = LimelightHelpers.getBotPose2d_wpiBlue("");  

        /*Failsafes */
        if(updatedPose == null || (updatedPose.getX() == 0.0 && updatedPose.getY() == 0.0 && updatedPose.getRotation().getDegrees() == 0.0)) {
            System.out.println("Pose was Null or zero when making path!! Gonna cancel path now\n");
            mode = -1;
        } 
        if(id < 1 || id > 22) {
            System.out.printf("ID wasn't within proper range!! The ID was: %f\n", id);
            mode = -1;
        }
        if(LimelightHelpers.getTV("") == false) {
            System.out.println("Limelight Could not see target after pressing!!\n");
            mode = -1;
        }
        if(updatedPose.getRotation().getCos() == 0.0 && updatedPose.getRotation().getSin() == 0.0) {
            System.out.println("Rotation was invalid while creating path!!\n");
            mode = -1;
        }

        /*Cancel Path OR Failsafe Met */
        if(mode == -1) {
            List<Waypoint> wayPoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(getState().Pose.getX(), getState().Pose.getY(), getState().Pose.getRotation()),
                new Pose2d(getState().Pose.getX() + 0.01, getState().Pose.getY(), getState().Pose.getRotation())
            );
                
            PathConstraints constraints = new PathConstraints(0.5, 0.5, 2 * Math.PI, 2 * Math.PI); // The constraints for this path.
            
            PathPlannerPath path = new PathPlannerPath( 
                wayPoints,
                constraints,
                null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                new GoalEndState(0.0, getState().Pose.getRotation()) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );
                
            path.preventFlipping =true;
            pathActive = false;
            UpdatedPose = false;
            return path;
        } 

        /*Failsafes NOT Met, Auto Align */
        pathActive = true;
        UpdatedPose = true;
            
        resetPose(updatedPose);
        List<Waypoint> wayPoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(getState().Pose.getX(), getState().Pose.getY(), getState().Pose.getRotation()),
            new Pose2d(TagArray.get((int)id).LeftX, TagArray.get((int)id).LeftY, TagArray.get((int)id).BotAngle)
        );        

        PathConstraints constraints = new PathConstraints(2, 2, 2 * Math.PI, 2 * Math.PI); // The constraints for this path.
                
        EventMarker signalEnd = new EventMarker("ChangeBool", 0.95, -1, new InstantCommand(() -> {pathActive = false;})); // THIS COMMAND IS TERMINATED WHEN THE PATH ENDS
        EventMarker turnofflie = new EventMarker("ChangeLime", 0.99, -1, new InstantCommand(() -> {UpdatedPose = false;}));
        List<EventMarker> lst_em = Arrays.asList(signalEnd, turnofflie);
        List<RotationTarget> lst_rt = Arrays.asList();
        List<ConstraintsZone> lst_cz = Arrays.asList();
        List<PointTowardsZone> lst_ptz = Arrays.asList();

        PathPlannerPath path = new PathPlannerPath( 
            wayPoints,
            lst_rt,
            lst_ptz, 
            lst_cz,
            lst_em,
            constraints,
            null, 
            new GoalEndState(0.0, TagArray.get((int)id).BotAngle), // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            false
        );
        
        path.preventFlipping =true;
        return path;    
    }

    public PathPlannerPath GoMid4Algae() {
        List<Waypoint> wayPoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(getState().Pose.getX(), getState().Pose.getY(), getState().Pose.getRotation()),
            new Pose2d(4.914, 5.092, Rotation2d.fromDegrees(240))
            // 11 Right is 12.634, 2.96, 60 degRree
            // 6 Right is 13.77, 3.125, 120 degree
            // 8 Right is 13.484, 5.092, 240 degree
            // 9 Right is 12.348, 4.927, 300 degrees
            // 7 Right is 14.195, 4.191, 180 degrees
        );

        PathConstraints constraints = new PathConstraints(0.15, 0.15, 2 * Math.PI, 2 * Math.PI); // The constraints for this path.

        PathPlannerPath path = new PathPlannerPath(
            wayPoints,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
            new GoalEndState(0.0, Rotation2d.fromDegrees(240)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );
        
        path.preventFlipping =true;
        return path;
    }

    public PathPlannerPath TestCrash() {
        Pose2d currentPose = LimelightHelpers.getBotPose2d_wpiBlue("");
        resetPose(currentPose);
        List<Waypoint> wayPoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(getState().Pose.getX(), getState().Pose.getY(), getState().Pose.getRotation()),
            new Pose2d(1, 0, Rotation2d.fromDegrees(0))
        );

        PathConstraints constraints = new PathConstraints(0.15, 0.15, 2 * Math.PI, 2 * Math.PI); // The constraints for this path.

        PathPlannerPath path = new PathPlannerPath(
            wayPoints,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
            new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );

        return path;
    }
}
