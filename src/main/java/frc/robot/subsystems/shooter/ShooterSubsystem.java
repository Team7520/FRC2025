package frc.robot.subsystems.shooter;


import javax.management.relation.Relation;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.revrobotics.CANSparkBase;
// import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import frc.team7520.robot.Constants.IntakeConstants;
// import frc.team7520.robot.Constants.ShooterConstants;
// import frc.team7520.robot.Constants.ShooterConstants.PivotConstants;
// import frc.team7520.robot.Constants.ShooterConstants.TraverseConstants;

public class ShooterSubsystem extends SubsystemBase {

    // With eager singleton initialization, any static variables/fields used in the
    // constructor must appear before the "INSTANCE" variable so that they are initialized
    // before the constructor is called when the "INSTANCE" variable initializes.

    private TalonFX botShooterMotor;
    private TalonFX topShooterMotor;
    // private TalonFX pivotMotor;
    
    private TalonFX traverseMotor;
    // private SparkPIDController botShooterPID;
    // private SparkPIDController topShooterPID;

    // private RelativeEncoder pivotEncoder;
    private RelativeEncoder botShooterEncoder;
    private RelativeEncoder topShooterEncoder;

    public Rotation2d desiredPivotPos = new Rotation2d(0);

    private SlewRateLimiter mSpeedLimiter = new SlewRateLimiter(100);

    /**
     * The Singleton instance of this shooterSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static ShooterSubsystem INSTANCE = new ShooterSubsystem();

    /**
     * Returns the Singleton instance of this shooterSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code shooterSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static ShooterSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this shooterSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private ShooterSubsystem() {
        // pivotMotor = new TalonFX(PivotConstants.CAN_ID);
        // traverseMotor = new TalonFX(TraverseConstants.CAN_ID);
        // var tlnfxConfigs = new TalonFXConfiguration();
        // var motorConfigs = new MotorOutputConfigs();
        // var slot0Configs = new Slot0Configs();
        // var motionMagicConfigs = tlnfxConfigs.MotionMagic;

        // pivotMotor.getConfigurator().apply(new TalonFXConfiguration());
        // motorConfigs.NeutralMode = NeutralModeValue.Coast;
        // tlnfxConfigs.Feedback.SensorToMechanismRatio = PivotConstants.degreeConversionFactor;

        // slot0Configs.kP = PivotConstants.kP;
        // slot0Configs.kI = PivotConstants.kI;
        // slot0Configs.kD = PivotConstants.kD;
        // slot0Configs.kG = PivotConstants.kG;
        // slot0Configs.kS = PivotConstants.kS;
        // slot0Configs.kV = PivotConstants.kV;
        // slot0Configs.kA = PivotConstants.kA;

        // motionMagicConfigs.MotionMagicCruiseVelocity = PivotConstants.motionMagicVelocity;
        // motionMagicConfigs.MotionMagicAcceleration = PivotConstants.motionMagicAccel;
        // motionMagicConfigs.MotionMagicJerk = PivotConstants.motionMagicJerk;

        // pivotMotor.getConfigurator().apply(motorConfigs);
        // pivotMotor.getConfigurator().apply(tlnfxConfigs);
        // pivotMotor.getConfigurator().apply(slot0Configs);
        // pivotMotor.getConfigurator().apply(motionMagicConfigs);

        // pivotMotor.setPosition(0);

        botShooterMotor = new TalonFX(25); // left = bot
        topShooterMotor = new TalonFX(26);
        //pivotMotor = new TalonFX(PivotConstants.CAN_ID);
        
        botShooterMotor.setInverted(true);
        topShooterMotor.setInverted(true);
        // traverseMotor.setInverted(true);
        // pivotMotor.setInverted(true);
    }
    public void setHorizontalSpeed(double speed){
        traverseMotor.set(speed);
    }
    // public void setPivotSpeed(double speed) {
    //     pivotMotor.set(speed);
    // }
    
    public void setSpeed(double speed, boolean closedLoop) {
        speed = mSpeedLimiter.calculate(speed);
        botShooterMotor.set(speed);
        topShooterMotor.set(speed);
    }

    // public void setPosition(ShooterConstants.Position position) {
    //     desiredPivotPos = position.getPivot();
    //     // PositionDutyCycle psdc = new PositionDutyCycle(10);
    //     // final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
    //     // pivotMotor.setControl(m_request.withPosition(desiredPivotPos.getDegrees()));
    //     // pivotMotor.setControl(psdc);
    //     final MotionMagicVoltage moveRequest = new MotionMagicVoltage(0).withSlot(0);
    //     pivotMotor.setControl(moveRequest.withPosition(desiredPivotPos.getDegrees()));
    // }

    public void stopShooting() {
        botShooterMotor.stopMotor();
        topShooterMotor.stopMotor();
    }

    // public void stopPivot() {
    //     pivotMotor.stopMotor();
    // }

    // public double getEncoder() {
    //     return pivotMotor.getPosition().getValueAsDouble();
    // }

    public Rotation2d getDesiredPivotPosition() {
        return desiredPivotPos;
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("pivotEncoder", pivotMotor.getPosition().getValueAsDouble());
        // SmartDashboard.putNumber("pivotVel", pivotMotor.getVelocity().getValueAsDouble());
        // SmartDashboard.putNumber("pivotVoltage", pivotMotor.getMotorVoltage().getValueAsDouble());
        // SmartDashboard.putNumber("pivotCurr", pivotMotor.getStatorCurrent().getValueAsDouble());
        // SmartDashboard.putNumber("desiredPos", desiredPivotPos.getDegrees());
        // SmartDashboard.putNumber("closedLoopError", pivotMotor.getClosedLoopError().getValueAsDouble());
        // SmartDashboard.putNumber("DesiredDeg", desiredPosition.getDegrees());
        // SmartDashboard.putNumber("DesiredRot", desiredPosition.getRotations());
        // SmartDashboard.putNumber("diffedEncoder", getDiffedEncoder());
        // SmartDashboard.putNumber("PivotAbsEncoder", pivotAbsEncoder.get());
        // SmartDashboard.putNumber("wheelsAbsEncoder", wheelAbsEncoder.get());
    }
}