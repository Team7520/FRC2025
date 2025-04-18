package frc.robot.subsystems;

import java.util.function.Supplier;

//import com.revrobotics.AbsoluteEncoder;

//import static edu.wpi.first.units.Units.Rotation;

//import com.revrobotics.AnalogInput;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAnalogSensor;
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
//import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.EndEffectorConstants.PivotPosition;
//import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EndEffectorSubsystem extends SubsystemBase {
    private final static EndEffectorSubsystem INSTANCE = new EndEffectorSubsystem();

    private final DigitalInput sensorInput = new DigitalInput(0);

    private final SparkMax pivotMotor;
    private final SparkFlex conveyorMotor;
    private final SparkClosedLoopController pivotController;
    private final RelativeEncoder pivotEncoder;
    private final SparkClosedLoopController conveyorController;
    private PivotPosition lastPivotPosition;
    private double holdPosition = 0;
    private boolean isHoldingPosition = false;
    //private AbsoluteEncoder absoluteEncoder;
    private SparkAnalogSensor analoginput;
    private SparkAnalogSensor absoluteAnalogInput;
    private double kFF = -0.28;
    private double encoderToDegrees = 150d/90d;
    private double handZeroDegree = -60d; 
    private double handAngle;                                
    private double referenceHandAngle;

    public static EndEffectorSubsystem getInstance() {
        return INSTANCE;
    }

    private EndEffectorSubsystem() {
        // Initialize pivot motor with motion control
        pivotMotor = new SparkMax(EndEffectorConstants.PIVOT_ID, MotorType.kBrushless);
        pivotController = pivotMotor.getClosedLoopController();
        pivotEncoder = pivotMotor.getEncoder();
        //absoluteEncoder = pivotMotor.getAbsoluteEncoder();

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
        .forwardSoftLimit(EndEffectorConstants.MAX_ANGLE)
        .forwardSoftLimitEnabled(false)
        .reverseSoftLimit(EndEffectorConstants.MIN_ANGLE))
        .reverseSoftLimitEnabled(false);

        pivotConfig.smartCurrentLimit(EndEffectorConstants.PIVOT_CURRENT_LIMIT);
        pivotConfig.idleMode(IdleMode.kBrake);
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        analoginput = conveyorMotor.getAnalog();
        absoluteAnalogInput = pivotMotor.getAnalog();
//      pivotEncoder.setPosition(0);

    SmartDashboard.putNumber("Pivot FF", kFF);
    }

    public void test() {
        handAngle = (pivotEncoder.getPosition()-handZeroDegree)/encoderToDegrees;
        //pivotController.setReference(position.getAngle(), ControlType.kMAXMotionPositionControl);
        pivotController.setReference(pivotEncoder.getPosition(), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, kFF*Math.abs(Math.sin(handAngle)));
    }

    public void ninety() {
        pivotController.setReference(pivotEncoder.getPosition()-90, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, kFF);
    }

    public void setPivotPosition(EndEffectorConstants.PivotPosition position) {
        lastPivotPosition  = position;
        handAngle = (pivotEncoder.getPosition()-handZeroDegree)/encoderToDegrees;
        referenceHandAngle = (position.getAngle()-handZeroDegree)/encoderToDegrees;
        double referenceAngleRadians = Math.toRadians(referenceHandAngle);
        //pivotController.setReference(position.getAngle(), ControlType.kMAXMotionPositionControl);
        pivotController.setReference(position.getAngle(), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, kFF*Math.cos(referenceAngleRadians));
    }

    public PivotPosition getLastPivotPosition() {
        return lastPivotPosition;
    }
        

    public Command setPivotPositionCommand(PivotPosition down) {
        return Commands.runOnce(() -> setPivotPosition(down), this);
    }

    public void manual(double addRotations) {
        pivotController.setReference(pivotEncoder.getPosition() + addRotations, ControlType.kPosition);
    }

    public void resetEncoder() {
        pivotEncoder.setPosition(0);
    }

    public Command resetEncoderCommand() {
        return Commands.runOnce(() -> resetEncoder(), this);
    }

    public double AnalogOutput() {
        return analoginput.getVoltage();
    }

    public void setSpeed(double speed) {
        pivotMotor.set(speed);
    }

    public void stop() {
        setSpeed(0);
        stopConveyor();
    }

    public boolean StopWithSensor() {
        if(analoginput.getVoltage() <= 1.5) {
            return true;
        } else {
            return false;
        }
    }

    public boolean handOut() {
        return pivotEncoder.getPosition() >= -215;
    }
    
    // public double getAbsoluteEncoder(){
    //     return absoluteEncoder.getPosition();
    // }

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

    public Command setConveyorSpeedCommand(Supplier<ElevatorPosition> positionSup) {
        double speed = positionSup.get().getSpeed();
        SmartDashboard.putNumber("outspeed", speed);
        return this.run(() -> setConveyorSpeed(speed))
               .finallyDo((interrupted) -> setConveyorSpeed(0)); // Explicitly require this subsystem
    }

    public void stopConveyor() {
        //conveyorMotor.set(0);
        holdConveyorPosition();
    }

    public Command stopConveyorCommand() {
        return this.run(() -> stopConveyor());
    }
    
    public boolean getSensor() {
        return sensorInput.get();
    }

    @Override
    public void periodic() {    
        // Update dashboard with current positions and states
        SmartDashboard.putNumber("Pivot Angle", pivotEncoder.getPosition());
        //SmartDashboard.putNumber("Absolute Pivot Angle", getAbsoluteEncoder());
        SmartDashboard.putNumber("Pivot Velocity", pivotEncoder.getVelocity());
        SmartDashboard.putBoolean("Sensor", getSensor());
        SmartDashboard.putNumber("Analog Voltage", AnalogOutput());
        //SmartDashboard.putNumber("AbsoluteEncoderPosition", getAbsoluteEncoder());
        SmartDashboard.putNumber("AbsoluteEncoderVoltage", absoluteAnalogInput.getVoltage());
        SmartDashboard.putNumber("Pivot Voltage", pivotMotor.getBusVoltage());
        kFF = SmartDashboard.getNumber("Pivot FF", 0);
    }
}

