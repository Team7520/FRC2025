package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EndEffectorSubsystem extends SubsystemBase {
    private final static EndEffectorSubsystem INSTANCE = new EndEffectorSubsystem();
    
    private final SparkMax pivotMotor;
    private final SparkMax conveyorMotor;
    private final SparkClosedLoopController pivotController;
    private final RelativeEncoder pivotEncoder;
    
    public static EndEffectorSubsystem getInstance() {
        return INSTANCE;
    }
    
    private EndEffectorSubsystem() {
        // Initialize pivot motor with motion control
        pivotMotor = new SparkMax(EndEffectorConstants.PIVOT_ID, MotorType.kBrushless);
        pivotController = pivotMotor.getClosedLoopController();
        pivotEncoder = pivotMotor.getEncoder();
        
        SparkMaxConfig conveyorConfig = new SparkMaxConfig();
        conveyorConfig.smartCurrentLimit(EndEffectorConstants.CONVEYOR_CURRENT_LIMIT);
        
        // Initialize conveyor motor with basic control
        conveyorMotor = new SparkMax(EndEffectorConstants.CONVEYOR_ID, MotorType.kBrushless);
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
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(EndEffectorConstants.MAX_ANGLE)
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(EndEffectorConstants.MIN_ANGLE));
        
        pivotConfig.smartCurrentLimit(EndEffectorConstants.PIVOT_CURRENT_LIMIT);
        
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public void setPivotPosition(EndEffectorConstants.PivotPosition position) {
        pivotController.setReference(position.getAngle(), ControlType.kMAXMotionPositionControl);
    }
    
    public Command setPivotPositionCommand(EndEffectorConstants.PivotPosition position) {
        return this.runOnce(() -> setPivotPosition(position));
    }
    
    public void setPivotPosition(Angle angle) {
        pivotController.setReference(angle.in(Rotation), ControlType.kMAXMotionPositionControl);
    }
    
    public Command setPivotPositionCommand(Angle angle) {
        return this.runOnce(() -> setPivotPosition(angle));
    }
    
    public void setConveyorSpeed(double speed) {
        conveyorMotor.set(speed);
    }
    
    public Command setConveyorSpeedCommand(double speed) {
        return this.runOnce(() -> setConveyorSpeed(speed));
    }
    
    public void stopConveyor() {
        conveyorMotor.set(0);
    }
    
    public Command stopConveyorCommand() {
        return this.runOnce(() -> stopConveyor());
    }
    
    @Override
    public void periodic() {
        // Update dashboard with current positions and states
        SmartDashboard.putNumber("Pivot Angle", pivotEncoder.getPosition());
        SmartDashboard.putNumber("Pivot Velocity", pivotEncoder.getVelocity());
    }
}

