package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

public class LightingSubsystem {
    private final static LightingSubsystem INSTANCE = new LightingSubsystem();
    public final CANdle candle = new CANdle(17); // creates a new CANdle with ID 17
    
    public static LightingSubsystem getInstance() {
      return INSTANCE;
    }
    
    private LightingSubsystem() {
      candle.setLEDs(255, 255, 255); // set the CANdle LEDs to white      
    }

    public void setLEDs(int r, int g, int b) {
      candle.setLEDs(r, g, b);
    }

    public void RainbowAnimate() {
      // create a rainbow animation:

      // - max brightness

      // - half speed

      // - 32 LEDs

      RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.5, 32);

      candle.animate(rainbowAnim);
    }

    public void StrobeAnimate(int r, int g, int b) {
      StrobeAnimation strobeAnim = new StrobeAnimation(r, g, b);

      candle.animate(strobeAnim);
    }

    public void AnimateTeam() {
      // Set LEDs to strobe team colour
      StrobeAnimate(255, 0, 0);
    }
}
