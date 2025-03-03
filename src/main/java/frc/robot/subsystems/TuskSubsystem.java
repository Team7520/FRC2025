package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import com.revrobotics.RelativeEncoder;
//import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
//import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TuskConstants;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TuskSubsystem extends SubsystemBase {
    private final static TuskSubsystem INSTANCE = new TuskSubsystem();

    private final SparkMax pivotMotor;
    private final SparkClosedLoopController pivotController;
    private final RelativeEncoder pivotEncoder;
    @SuppressWarnings("unused")
    private double lastPosition;


    public static TuskSubsystem getInstance() {
        return INSTANCE;
    }

    private TuskSubsystem() {
        // Initialize pivot motor with motion control
        pivotMotor = new SparkMax(TuskConstants.PIVOT_ID, MotorType.kBrushless);
        pivotController = pivotMotor.getClosedLoopController();
        pivotEncoder = pivotMotor.getEncoder();

        // Initialize conveyor motor with basic control
        

        // Configure pivot motor
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.encoder
        .positionConversionFactor(TuskConstants.SENSOR_TO_MECHANISM_RATIO)
        .velocityConversionFactor(TuskConstants.SENSOR_TO_MECHANISM_RATIO);

        pivotConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(TuskConstants.kP)
        .i(TuskConstants.kI)
        .d(TuskConstants.kD)
        .outputRange(-1, 1);

        pivotConfig.closedLoop.maxMotion
        .maxVelocity(TuskConstants.MAX_VELOCITY)
        .maxAcceleration(TuskConstants.MAX_ACCELERATION)
        .allowedClosedLoopError(TuskConstants.ALLOWABLE_ERROR);

        pivotConfig.softLimit.apply(new SoftLimitConfig()
        .forwardSoftLimitEnabled(false)
        .forwardSoftLimit(TuskConstants.MAX_ANGLE)
        .reverseSoftLimitEnabled(false)
        .reverseSoftLimit(TuskConstants.MIN_ANGLE));

        pivotConfig.smartCurrentLimit(TuskConstants.PIVOT_CURRENT_LIMIT);
        pivotConfig.idleMode(IdleMode.kBrake);
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

//        pivotEncoder.setPosition(0);
        
    }

    public void setPivotPosition(TuskConstants.PivotPosition position) {
        lastPosition  = position.getAngle();
        pivotController.setReference(position.getAngle(), ControlType.kMAXMotionPositionControl);
    }

    public Command setPivotPositionCommand(TuskConstants.PivotPosition position) {
        return this.runOnce(() -> setPivotPosition(position));
    }

    public void setPivotPosition(Angle angle) {
        pivotController.setReference(angle.in(Rotation), ControlType.kMAXMotionPositionControl);
    }

    public Command setPivotPositionCommand(Angle angle) {
        return this.runOnce(() -> setPivotPosition(angle));
    }

    public void setSpeed(double speed) {
        pivotMotor.set(speed);
    }

    public void stop() {
        setSpeed(0);

    }

    public Command run(double speed) {
        return this.run(() -> setSpeed(speed));
    }


    

    @Override
    public void periodic() {
        // Update dashboard with current positions and states
        SmartDashboard.putNumber("Tusk Angle", pivotEncoder.getPosition());
        SmartDashboard.putNumber("Tusk Velocity", pivotEncoder.getVelocity());
    }
}

