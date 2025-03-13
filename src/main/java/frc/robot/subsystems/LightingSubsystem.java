package frc.robot.subsystems;// Define the package
import java.util.Optional;

import com.ctre.phoenix.led.CANdle;// Import the CANdle libraries, the WPILib Addressable LED libaries, and the LED animation libraries
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
//import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;

/* 
 * Class that controls robot lighting through the CANdle (excluding the RSL)
 * Note: Until CTRE implements CANdle support in Phoenix 6, Phoenix 5 is required.
 */
// Flashing white, fire animation
public class LightingSubsystem {// Define a class
    private final static LightingSubsystem INSTANCE = new LightingSubsystem();// Set INSTANCE to a new instance of LightingSubsystem
    public final CANdle candle = new CANdle(17); // creates a new CANdle with ID 17
    public final int numLED = 32;// Change to adjust number of LEDs
    public final AddressableLED sideLED = new AddressableLED(0);// Initialize side LED strips
    public final AddressableLEDBuffer sideLEDBuffer = new AddressableLEDBuffer(40);
    /*
    public final AddressableLEDBufferView sideLeftLED = sideLEDBuffer.createView(0, 40);
    public final AddressableLEDBufferView sideRightLED = sideLEDBuffer.createView(21, 40);
    */
    public static LightingSubsystem getInstance() {// Return the instance when getInstance is called in the robot code
      return INSTANCE;
    }

    public void ColourFlowAnimate(int r, int g, int b) {// Colour flow animation
      ColorFlowAnimation colourFlowAnim = new ColorFlowAnimation(r, g, b);
      candle.animate(colourFlowAnim);
    }

    private LightingSubsystem() {// Startup code
      /* Side LEDs setup(roboRIO PWM port) */
      sideLED.setLength(sideLEDBuffer.getLength());
      // Set the data
      sideLED.setData(sideLEDBuffer);
      sideLED.start();
      /* CANdle setup */
      candle.clearAnimation(0);// Clear animations
      candle.clearAnimation(1);
      ColourFlowAnimate(127, 127, 127);
      setSideLEDs(127, 127, 127);
    }

    public void setLEDs(int r, int g, int b) {// Function to change LED colours on intake and CANdle
      candle.setLEDs(r, g, b);
    }

    public void setSideLEDs(int r, int g, int b) {// Function to change LED colours on side
      for (var i = 0; i < sideLEDBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        sideLEDBuffer.setRGB(i, r, g, b);
      }
      sideLED.setData(sideLEDBuffer);
    }
    
    public void RainbowAnimateSide() {
      // all hues at maximum saturation and half brightness
      LEDPattern rainbow = LEDPattern.rainbow(255, 128);

      // Our LED strip has a density of 60 LEDs per meter
      Distance kLedSpacing = Meters.of(1 / 60.0);

      // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a speed
      // of 1 meter per second.
      LEDPattern scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), kLedSpacing);
      
      // Apply the pattern to the strip
      scrollingRainbow.applyTo(sideLEDBuffer);
      sideLED.setData(sideLEDBuffer);
    }

    /*
    public void setSideLeftLEDs(int r, int g, int b) {// Function to change LED colours on side strips
      LEDPattern colour = LEDPattern.solid(null);
      colour.applyTo(sideLeftLED);
      sideLED.setData(sideLeftLED);
    }

    public void setSideRightLEDs(int r, int g, int b) {// Function to change LED colours on side strips

    }*/
    public void RainbowAnimate() {// Rainbow animation
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
      // Set LEDs to team colour
      StrobeAnimate(255, 0, 0);
    }

    public void FlashingWhite() {// Flash all LEDs white
      StrobeAnimate(191, 191, 191);
    }

    public void FireAnimate() {// Fire animation
      FireAnimation fireAnimL = new FireAnimation(1, 0.1, numLED, 1, 1, false, 0);// Left side animation
      FireAnimation fireAnimR = new FireAnimation(1, 0.1, numLED, 1, 1, true, 20);// Right side animation (needs offset and reverse)
      //TwinkleAnimation fireAnimL = new TwinkleAnimation(242, 145, 12, 0, 1, numLED, null);
      //TwinkleAnimation fireAnimR = new TwinkleAnimation(242, 145, 12, 0, 1, numLED, null);
      candle.animate(fireAnimL, 0);
      candle.animate(fireAnimR, 1);
    }

    public void FlashAllianceColour() {// Gets alliance colour from DS and adjusts colour based on alliance
      Optional<Alliance> ally = DriverStation.getAlliance();
      if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
          StrobeAnimate(255, 0, 0);
        }
        if (ally.get() == Alliance.Blue) {
          StrobeAnimate(0, 0, 255);
        }
      }
      else {
        RainbowAnimate();
      }
    }
}