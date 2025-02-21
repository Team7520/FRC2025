package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EndEffectorSubsystem extends SubsystemBase {
    private final static EndEffectorSubsystem INSTANCE = new EndEffectorSubsystem();

    private final SparkMax pivotMotor;
    private final SparkFlex conveyorMotor;
    private final SparkClosedLoopController pivotController;
    private final RelativeEncoder pivotEncoder;
    private final SparkClosedLoopController conveyorController;
    private double lastPivotPosition;
    private double holdPosition = 0;
    private boolean isHoldingPosition = false;


    public static EndEffectorSubsystem getInstance() {
        return INSTANCE;
    }

    private EndEffectorSubsystem() {
        // Initialize pivot motor with motion control
        pivotMotor = new SparkMax(EndEffectorConstants.PIVOT_ID, MotorType.kBrushless);
        pivotController = pivotMotor.getClosedLoopController();
        pivotEncoder = pivotMotor.getEncoder();

        SparkFlexConfig conveyorConfig = new SparkFlexConfig();
        conveyorConfig.smartCurrentLimit(EndEffectorConstants.CONVEYOR_CURRENT_LIMIT);

        // Initialize conveyor motor with basic control
        
        conveyorMotor = new SparkFlex(EndEffectorConstants.CONVEYOR_ID, MotorType.kBrushless);
        conveyorController = conveyorMotor.getClosedLoopController();
        conveyorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(EndEffectorConstants.kP_CONVEYOR)
        .i(EndEffectorConstants.kI_CONVEYOR)
        .d(EndEffectorConstants.kD_CONVEYOR)
        .outputRange(-1, 1);
        conveyorConfig.idleMode(IdleMode.kBrake);
        conveyorMotor.configure(conveyorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // Configure pivot motor
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.encoder
        .positionConversionFactor(EndEffectorConstants.SENSOR_TO_MECHANISM_RATIO)
        .velocityConversionFactor(EndEffectorConstants.SENSOR_TO_MECHANISM_RATIO);

        pivotConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(EndEffectorConstants.kP)
        .i(EndEffectorConstants.kI)
        .d(EndEffectorConstants.kD)
        .outputRange(-1, 1);

        pivotConfig.closedLoop.maxMotion
        .maxVelocity(EndEffectorConstants.MAX_VELOCITY)
        .maxAcceleration(EndEffectorConstants.MAX_ACCELERATION)
        .allowedClosedLoopError(EndEffectorConstants.ALLOWABLE_ERROR);

        pivotConfig.softLimit.apply(new SoftLimitConfig()
        .forwardSoftLimitEnabled(false)
        .forwardSoftLimit(EndEffectorConstants.MAX_ANGLE)
        .reverseSoftLimitEnabled(false)
        .reverseSoftLimit(EndEffectorConstants.MIN_ANGLE));

        pivotConfig.smartCurrentLimit(EndEffectorConstants.PIVOT_CURRENT_LIMIT);
        pivotConfig.idleMode(IdleMode.kBrake);
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

//        pivotEncoder.setPosition(0);
        
    }

    public void setPivotPosition(EndEffectorConstants.PivotPosition position) {
        lastPivotPosition  = position.getAngle();
        pivotController.setReference(position.getAngle(), ControlType.kMAXMotionPositionControl);
    }

    public Command setPivotPositionCommand(EndEffectorConstants.PivotPosition position) {
        return Commands.runOnce(() -> setPivotPosition(position), this);
    }


    public void setSpeed(double speed) {
        pivotMotor.set(speed);
    }

    public void stop() {
        setSpeed(0);
        stopConveyor();

    }

    public Command run(double speed) {
        return this.run(() -> setSpeed(speed));
    }

    public void holdConveyorPosition() {
        if (!isHoldingPosition) {
            isHoldingPosition = true;
            holdPosition = conveyorMotor.getEncoder().getPosition(); // Get current position
        }
        conveyorController.setReference(holdPosition, ControlType.kPosition);
    }


    // Command to hold the conveyor position

    public void setConveyorSpeed(double speed) {
        isHoldingPosition = false;
        conveyorMotor.set(speed);
    }

    public Command setConveyorSpeedCommand(double speed) {
        return this.run(() -> setConveyorSpeed(speed))
               .finallyDo((interrupted) -> setConveyorSpeed(0)); // Explicitly require this subsystem
    }

    public void stopConveyor() {
        //conveyorMotor.set(0);
        holdConveyorPosition();
        pivotController.setReference(lastPivotPosition, ControlType.kMAXMotionPositionControl);
    }

    public Command stopConveyorCommand() {
        return this.run(() -> stopConveyor());
    }
    

    @Override
    public void periodic() {
        // Update dashboard with current positions and states
        SmartDashboard.putNumber("Pivot Angle", pivotEncoder.getPosition());
        SmartDashboard.putNumber("Pivot Velocity", pivotEncoder.getVelocity());
    }
}

