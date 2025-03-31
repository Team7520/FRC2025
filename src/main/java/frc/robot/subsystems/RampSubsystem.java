package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

//import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RampSubsystem extends SubsystemBase {
    private final TalonFX RampMotor = new TalonFX(Constants.RampConstants.RampID);
    private final TalonFX StarWheelMotor = new TalonFX(Constants.RampConstants.StarWheelID);
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
        StarWheelMotor.set(speed);
    }

    public void setSeparateSpeeds(double rampSpeed, double starWheelSpeed) {
        RampMotor.set(rampSpeed);
        StarWheelMotor.set(starWheelSpeed);
    }

    public Command run(double speed) {
        return this.run(() -> setSpeed(speed));
    }

    public Command runSeparately(double rampSpeed, double starWheelSpeed) {
        return this.run(() -> setSeparateSpeeds(rampSpeed, starWheelSpeed));
    }
}
