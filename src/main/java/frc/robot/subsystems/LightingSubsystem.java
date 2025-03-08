package frc.robot.subsystems;// Define the package

import java.util.Optional;

import javax.print.attribute.standard.ColorSupported;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;// Import the CANdle libraries and the LED animation libraries
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* 
 * Class that controls robot lighting through the CANdle (excluding the RSL)
 * Note: Until CTRE implements CANdle support in Phoenix 6, Phoenix 5 is required.
 */
// Flashing white, fire animation
public class LightingSubsystem extends SubsystemBase {// Define a class
    private final static LightingSubsystem INSTANCE = new LightingSubsystem();// Set INSTANCE to a new instance of LightingSubsystem
    public final CANdle candle = new CANdle(17); // creates a new CANdle with ID 17
    public final int numLED = 32;// Change to adjust number of LEDs
    public int alliance = 0;

    public static LightingSubsystem getInstance() {// Return the instance when getInstance is called in the robot code
      return INSTANCE;
    }

    public void clearAnimation() {// Clears all animations
      candle.clearAnimation(0);
      candle.clearAnimation(1);
    }

    public void ColourFlowAnimate(int r, int g, int b) {// Colour flow animation
      ColorFlowAnimation colourFlowAnim = new ColorFlowAnimation(r, g, b);
      candle.animate(colourFlowAnim);
    }

    private LightingSubsystem() {// Startup code
      clearAnimation();
      ColourFlowAnimate(127, 127, 127);
    }
  
    public void setLEDs(int r, int g, int b) {// Function to change LED colours
      candle.setLEDs(r, g, b);
    }

    public void RainbowAnimate() {// Rainbow animation
      clearAnimation();
      // create a rainbow animation:

      // - max brightness

      // - half speed

      // - numLED LEDs

      RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.5, numLED);

      candle.animate(rainbowAnim, 0);
    }

    public void StrobeAnimate(int r, int g, int b) {// Strobe animation
      StrobeAnimation strobeAnim = new StrobeAnimation(r, g, b, 0, 0.1, numLED);

      candle.animate(strobeAnim, 0);
    }
  
    public void AnimateTeam() {
      clearAnimation();
      // Set LEDs to team colour
      StrobeAnimate(255, 0, 0);
    }

    public void FlashingWhite() {// Flash all LEDs white
      ColourFlowAnimate(55, 55, 55);
    }

    public Command lighting(EndEffectorSubsystem endEffectorSubsystem) {
      return Commands.run(
        () -> {
          if (endEffectorSubsystem.AnalogOutput() <= 1.8) {
            this.StrobeAnimate(0, 55, 0);
          }
          else {
            this.FlashingWhite();
          }
        }
      );
    }

    public void FireAnimate() {// Fire animation
      clearAnimation();
      FireAnimation fireAnimL = new FireAnimation(1, 0.1, numLED, 1, 1, false, 0);// Left side animation
      FireAnimation fireAnimR = new FireAnimation(1, 0.1, numLED, 1, 1, true, 20);// Right side animation (needs offset and reverse)
      //TwinkleAnimation fireAnimL = new TwinkleAnimation(242, 145, 12, 0, 1, numLED, null);
      //TwinkleAnimation fireAnimR = new TwinkleAnimation(242, 145, 12, 0, 1, numLED, null);
      candle.animate(fireAnimL, 0);
      candle.animate(fireAnimR, 1);
    }
    
    public void AllianceColour() {// Gets alliance colour from DS and adjusts colour based on alliance (rainbow if no alliance is detected)
      clearAnimation();
      Optional<Alliance> ally = DriverStation.getAlliance();
      if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
          //if (alliance != 1) {
            alliance = 1;
            candle.animate(new ColorFlowAnimation(100, 0, 0));
          //}
        }
        if (ally.get() == Alliance.Blue) {
          //if (alliance != 2) {
            alliance = 2;
            candle.animate(new ColorFlowAnimation(0, 0, 100));
          //}
        }
      }
      else {
        //if (alliance != 0) {
          alliance = 0;
          RainbowAnimate();
        //}
      }
    }
}