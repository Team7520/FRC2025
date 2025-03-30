package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
//import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
//import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ElevatorSubsystem extends SubsystemBase {
    private final static ElevatorSubsystem INSTANCE = new ElevatorSubsystem();
    
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final MotionMagicVoltage motionMagic;
    private final StrictFollower follower;
    private final DigitalInput limitSwitch;
    
    private ElevatorSubsystem() {
        limitSwitch = new DigitalInput(ElevatorConstants.LIMIT_SWITCH_ID);
        leftMotor = new TalonFX(ElevatorConstants.LEFT_MOTOR_ID);
        rightMotor = new TalonFX(ElevatorConstants.RIGHT_MOTOR_ID);
        
        // Configure motors
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = ElevatorConstants.kP;
        config.Slot0.kI = ElevatorConstants.kI;
        config.Slot0.kD = ElevatorConstants.kD;
        config.Slot0.kV = ElevatorConstants.kFF;
        config.Slot0.kG = ElevatorConstants.kG;
        config.Slot0.kA = ElevatorConstants.kA;
        config.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.MAX_VELOCITY;
        config.MotionMagic.MotionMagicAcceleration = ElevatorConstants.MAX_ACCELERATION;
        config.MotionMagic.MotionMagicJerk = ElevatorConstants.MAX_JERK;
        config.Feedback.SensorToMechanismRatio = ElevatorConstants.SENSOR_TO_MECHANISM_RATIO;
        // Current Limits
        CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
        limitConfigs.StatorCurrentLimit = 120;
        limitConfigs.StatorCurrentLimitEnable = false;
        limitConfigs.SupplyCurrentLimit = Constants.ElevatorConstants.CURRENT_LIMIT;
        limitConfigs.SupplyCurrentLimitEnable = false;
        config.withCurrentLimits(limitConfigs);
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        leftMotor.getConfigurator().apply(config);
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightMotor.getConfigurator().apply(config);
        
        // Configure right motor to follow left motor (inverted)
        
        // Set brake mode
        leftMotor.setNeutralMode(NeutralModeValue.Brake);
        rightMotor.setNeutralMode(NeutralModeValue.Brake);
        
        leftMotor.setPosition(0);
        rightMotor.setPosition(0);
               
        follower = new StrictFollower(leftMotor.getDeviceID());
        rightMotor.setControl(follower);
        
        motionMagic = new MotionMagicVoltage(0);
    }
    
    public static ElevatorSubsystem getInstance() {
        return INSTANCE;
    }
    
    /**
    * Sets the elevator to a specific position using motion magic
    * @param position The ElevatorPosition to move to
    */
    public void setPosition(ElevatorPosition position) {
        leftMotor.setControl(motionMagic.withPosition(position.getHeight()));
    }

    public boolean getLimitSwitch() {
        return limitSwitch.get();
    }

    public void resetEncoder() {
        leftMotor.setPosition(0);
        rightMotor.setPosition(0);
    }

    public Command resetEncoderCommand(){
        return Commands.runOnce(() -> resetEncoder());
    }
    
    /**
    * Creates a command to move the elevator to a specific position
    * @param position The ElevatorPosition to move to
    * @return A command that will move the elevator to the specified position
    */
    public Command moveToPosition(ElevatorPosition position) {
        SmartDashboard.putNumber("ElevatorSetpos", position.getHeight());
        return Commands.runOnce(() -> setPosition(position), this);
    }
    
    public Angle getCurrentPosition() {
        return leftMotor.getPosition().getValue();
    }
    
    /**
    * Adds to the current position while maintaining position control
    * @param addRotations amount to add to current position
    */
    public void addToPosition(double addRotations) {
        Angle currentPos = leftMotor.getPosition().getValue();
        leftMotor.setControl(motionMagic.withPosition(currentPos.in(Rotation) + addRotations));
    }
    
    @Override
    public void periodic() {
        // Add any periodic checks or logging here
        double currentPos = leftMotor.getPosition().getValue().magnitude();
        SmartDashboard.putNumber("ElevatorPos", currentPos);
        double setPointPos = leftMotor.getClosedLoopReference().getValue();
        SmartDashboard.putNumber("ElevatorSetPoint", setPointPos);
    }
}

