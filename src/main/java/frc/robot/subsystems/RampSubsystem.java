package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RampSubsystem extends SubsystemBase{
    private final TalonFX RampMotor = new TalonFX(Constants.RampConstants.RampID);
    // private final SlewRateLimiter speedLimiter = new SlewRateLimiter(100);

    private final static RampSubsystem INSTANCE = new RampSubsystem();

    @SuppressWarnings("WeakerAccess")
    public static RampSubsystem getInstance() {
        return INSTANCE;
    }

    private RampSubsystem(){
        //RampMotor.setInverted(true);
    }

    public void stop() {
        setSpeed(0);
    }

    public void setSpeed(double speed) {
        RampMotor.set(speed);
    }

}
