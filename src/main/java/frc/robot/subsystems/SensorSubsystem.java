package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class SensorSubsystem extends SubsystemBase {
    private final static SensorSubsystem INSTANCE = new SensorSubsystem();

    // private final I2C.Port i2cPort = I2C.Port.kOnboard;

    // private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

    DigitalInput IntakeBeamBreakInput = new DigitalInput(0);

    DigitalInput ShooterBeamBreakInput = new DigitalInput(1);

    @SuppressWarnings("WeakerAccess")
    public static SensorSubsystem getInstance() {
        return INSTANCE;
    }

    private SensorSubsystem(){

    }

    public boolean getColorSensorProximity(){
        boolean noteDetected = ShooterBeamBreakInput.get();
        SmartDashboard.putBoolean("NoteDetectedIntake", !noteDetected);
        return !noteDetected;
    }

    public boolean getBeamBreak(){
        boolean noteDetected = IntakeBeamBreakInput.get();
        SmartDashboard.putBoolean("NoteDetectedIntake", !noteDetected);
        return !noteDetected;
    }
}
