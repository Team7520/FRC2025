package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.PivotConstants;
import frc.robot.Constants.ShooterConstants.TraverseConstants;
public class TurretSubsystem extends SubsystemBase {
    private final static TurretSubsystem INSTANCE = new TurretSubsystem();
    private final TalonFX pivotMotor;
    private final TalonFX traverseMotor;
    private double lastManualPivotPosition = 0;
    private boolean pivotInManualMode = false;
    private double currentTraversePosition = 0;
    private double minTraversePosition = -1;      // Will be set to your min value
    private double maxTraversePosition = 1;      // Will be set to your max value
    

    public static TurretSubsystem getInstance() {
        return INSTANCE;
    }

    private void configPivot(TalonFX pivotMotor) {
        var tlnfxConfigs = new TalonFXConfiguration();
        var motorConfigs = new MotorOutputConfigs();
        var slot0Configs = new Slot0Configs();
        var motionMagicConfigs = tlnfxConfigs.MotionMagic;
        var currentLimitConfigs = new CurrentLimitsConfigs();

        pivotMotor.getConfigurator().apply(new TalonFXConfiguration());
        motorConfigs.NeutralMode = PivotConstants.neutralMode;
        tlnfxConfigs.Feedback.SensorToMechanismRatio = PivotConstants.degreeConversionFactor;
        tlnfxConfigs.withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitThreshold(77.6)
        );

        slot0Configs.kP = PivotConstants.kP;
        slot0Configs.kI = PivotConstants.kI;
        slot0Configs.kD = PivotConstants.kD;
        slot0Configs.kG = PivotConstants.kG;
        slot0Configs.kS = PivotConstants.kS;
        slot0Configs.kV = PivotConstants.kV;
        slot0Configs.kA = PivotConstants.kA;


        motionMagicConfigs.MotionMagicCruiseVelocity = PivotConstants.motionMagicVelocity;
        motionMagicConfigs.MotionMagicAcceleration = PivotConstants.motionMagicAccel;
        motionMagicConfigs.MotionMagicJerk = PivotConstants.motionMagicJerk;

        currentLimitConfigs.SupplyCurrentLimitEnable = true;
        currentLimitConfigs.SupplyCurrentLimit = PivotConstants.currentLimit;

        pivotMotor.getConfigurator().apply(motorConfigs);
        pivotMotor.getConfigurator().apply(tlnfxConfigs);
        pivotMotor.getConfigurator().apply(slot0Configs);
        pivotMotor.getConfigurator().apply(motionMagicConfigs);
        pivotMotor.getConfigurator().apply(currentLimitConfigs);

        pivotMotor.setInverted(true);

        pivotMotor.setPosition(0);
    }

    private void configTraverse(TalonFX traverseMotor) {
        var tlnfxConfigs = new TalonFXConfiguration();
        var motorConfigs = new MotorOutputConfigs();
        var slot0Configs = new Slot0Configs();
        var motionMagicConfigs = tlnfxConfigs.MotionMagic;
        var currentLimitConfigs = new CurrentLimitsConfigs();

        traverseMotor.getConfigurator().apply(new TalonFXConfiguration());
        motorConfigs.NeutralMode = TraverseConstants.neutralMode;
        tlnfxConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        tlnfxConfigs.Feedback.SensorToMechanismRatio = TraverseConstants.degreeConversionFactor;


        slot0Configs.kP = TraverseConstants.kP;
        slot0Configs.kI = TraverseConstants.kI;
        slot0Configs.kD = TraverseConstants.kD;
        slot0Configs.kG = TraverseConstants.kG;
        slot0Configs.kS = TraverseConstants.kS;
        slot0Configs.kV = TraverseConstants.kV;
        slot0Configs.kA = TraverseConstants.kA;

        motionMagicConfigs.MotionMagicCruiseVelocity = TraverseConstants.motionMagicVelocity;
        motionMagicConfigs.MotionMagicAcceleration = TraverseConstants.motionMagicAccel;
        motionMagicConfigs.MotionMagicJerk = TraverseConstants.motionMagicJerk;

        currentLimitConfigs.SupplyCurrentLimitEnable = true;
        currentLimitConfigs.SupplyCurrentLimit = TraverseConstants.currentLimit;

        traverseMotor.getConfigurator().apply(motorConfigs);
        traverseMotor.getConfigurator().apply(tlnfxConfigs);
        traverseMotor.getConfigurator().apply(slot0Configs);
        traverseMotor.getConfigurator().apply(motionMagicConfigs);
        traverseMotor.getConfigurator().apply(currentLimitConfigs);

        traverseMotor.setInverted(true);

        traverseMotor.setPosition(0);
    }

    private TurretSubsystem() {
        // Initialize pivot motor with motion control
        pivotMotor = new TalonFX(24);
        traverseMotor = new TalonFX(20);
        configPivot(pivotMotor);
        configTraverse(traverseMotor);

    }
    public double getPivotPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

    public void holdCurrentPosition() {
        if (!pivotInManualMode) {
            return;
        }
        lastManualPivotPosition = getPivotPosition();
        pivotInManualMode = false;
        pivotMotor.setControl(new PositionVoltage(lastManualPivotPosition));
    }

    public void setPivotSpeed(double speed) {
        if (!pivotInManualMode) {
            lastManualPivotPosition = getPivotPosition();
            pivotInManualMode = true;
        }
        pivotMotor.set(speed);
    }

    
    public void setTraverseSpeed(double speed) {
        // Get current position
        double currentPos = getTraversePosition();
        
        // Check limits
        if ((currentPos >= maxTraversePosition && speed > 0) || 
            (currentPos <= minTraversePosition && speed < 0)) {
            speed = 0;  // Stop movement if at limit
        }
        
        // Apply the speed
        traverseMotor.set(speed);
    }

    public void stopAll() {
        pivotMotor.stopMotor();
        traverseMotor.stopMotor();
    }

    public double getTraversePosition() {
        return currentTraversePosition;
    }
    public void setTraverseLimits(double minPos, double maxPos) {
        this.minTraversePosition = minPos;
        this.maxTraversePosition = maxPos;
    }
    @Override
    public void periodic() {
        if (!pivotInManualMode) {
            // Small position updates to maintain position
            pivotMotor.setControl(new PositionVoltage(lastManualPivotPosition));
        }
        currentTraversePosition = traverseMotor.getPosition().getValueAsDouble();
        SmartDashboard.putNumber("traverse pos",traverseMotor.getPosition().getValueAsDouble());
    }
}
