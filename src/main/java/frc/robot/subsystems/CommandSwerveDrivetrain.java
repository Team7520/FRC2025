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
import com.revrobotics.AnalogInput;
import com.revrobotics.spark.SparkFlex;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.filter.SlewRateLimiter;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.TagCoods;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
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
    public static boolean UpdatedPose = true;

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

    private void configureAutoBuilder(double driveP, double driveI, double driveD, double turnP, double turnI, double turnD) {
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
                    new PIDConstants(driveP, driveI, driveD),
                    // PID constants for rotation
                    new PIDConstants(turnP, turnI, turnD)
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
        configureAutoBuilder(14, 2, 0, 22.5, 5, 0);
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
        configureAutoBuilder(14, 2, 0, 22.5, 5, 0);
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
        configureAutoBuilder(14, 2, 0, 22.5, 5, 0);
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
        if(UpdatedPose) {
            Pose2d updatedPose = LimelightHelpers.getBotPose2d_wpiBlue("");
            if (updatedPose.getX() != 0.0 && updatedPose.getY() != 0.0 && updatedPose.getRotation().getDegrees() != 0.0 && counter > 0) {
                counter = 0;
                resetPose(updatedPose);
                SmartDashboard.putBoolean("Can See?", true);
                //System.out.printf("Updated X:%f Updated Y:%f, Updated Rotate:%f\n", updatedPose.getX(), updatedPose.getY(), updatedPose.getRotation().getDegrees());
            } else {
                counter++;
                //System.out.println("ran in null");
                SmartDashboard.putBoolean("Can See?", false);
            }   
            SmartDashboard.putNumber("RobotX_POSE", getState().Pose.getX());
            SmartDashboard.putNumber("RobotY_POSE", getState().Pose.getY());
            SmartDashboard.putNumber("Apriltag ID", LimelightHelpers.getFiducialID(""));
        }     
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

    public PathPlannerPath GoLeft(int mode) {
        //if Mode is 1, run normally. If -1, don't move
        if(mode == -1) {
            configureAutoBuilder(10, 0, 0, 5, 0, 0);
            List<Waypoint> wayPoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(getState().Pose.getX(), getState().Pose.getY(), getState().Pose.getRotation()),
                new Pose2d(getState().Pose.getX() + 0.01, getState().Pose.getY(), getState().Pose.getRotation())
            );
            
            PathConstraints constraints = new PathConstraints(0.75, 0.75, 2 * Math.PI, 2 * Math.PI); // The constraints for this path.
            
            PathPlannerPath path = new PathPlannerPath( 
                wayPoints,
                constraints,
                null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                new GoalEndState(0.0, getState().Pose.getRotation()) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );
            
            path.preventFlipping =true;
            return path;
        }
        //Create a path from current pose to the Left of seen tag
        pathActive = true;
        configureAutoBuilder(10, 0, 0, 5, 0, 0);
        double id = LimelightHelpers.getFiducialID("");
        List<Waypoint> wayPoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(getState().Pose.getX(), getState().Pose.getY(), getState().Pose.getRotation()),
            new Pose2d(TagArray.get((int)id).LeftX, TagArray.get((int)id).LeftY, TagArray.get((int)id).BotAngle)
        );

        PathConstraints constraints = new PathConstraints(1, 1, 2 * Math.PI, 2 * Math.PI); // The constraints for this path.

        EventMarker signalEnd = new EventMarker("ChangeBool", 0.95, -1, new InstantCommand(() -> {pathActive = false;})); // THIS COMMAND IS TERMINATED WHEN THE PATH ENDS
        List<EventMarker> lst_em = Arrays.asList(signalEnd);
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

    public PathPlannerPath GoRight(int mode) {
        if(mode == -1) {
            configureAutoBuilder(10, 0, 0, 5, 0, 0);
            List<Waypoint> wayPoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(getState().Pose.getX(), getState().Pose.getY(), getState().Pose.getRotation()),
                new Pose2d(getState().Pose.getX() + 0.01, getState().Pose.getY(), getState().Pose.getRotation())
            );
            
            PathConstraints constraints = new PathConstraints(0.75, 0.75, 2 * Math.PI, 2 * Math.PI); // The constraints for this path.
            
            PathPlannerPath path = new PathPlannerPath( 
                wayPoints,
                constraints,
                null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                new GoalEndState(0.0, getState().Pose.getRotation()) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
            );
            
            path.preventFlipping =true;
            pathActive = false;
            return path;
        }
        configureAutoBuilder(10, 0, 0, 5, 0, 0);
        pathActive = true;

        double id = LimelightHelpers.getFiducialID("");        
        List<Waypoint> wayPoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(getState().Pose.getX(), getState().Pose.getY(), getState().Pose.getRotation()),
                new Pose2d(TagArray.get((int)id).RightX, TagArray.get((int)id).RightY, TagArray.get((int)id).BotAngle)
            );        

        PathConstraints constraints = new PathConstraints(1, 1, 2 * Math.PI, 2 * Math.PI); // The constraints for this path.
        
        EventMarker signalEnd = new EventMarker("ChangeBool", 0.95, -1, new InstantCommand(() -> {pathActive = false;})); // THIS COMMAND IS TERMINATED WHEN THE PATH ENDS
        List<EventMarker> lst_em = Arrays.asList(signalEnd);
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
            // 11 Right is 12.634, 2.96, 60 degree
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
}
