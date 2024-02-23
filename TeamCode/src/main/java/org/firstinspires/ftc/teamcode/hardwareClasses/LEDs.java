package org.firstinspires.ftc.teamcode.hardwareClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardwareClasses.FTC_Addons.AdafruitNeopixelSeesaw;
import org.firstinspires.ftc.teamcode.hardwareClasses.FTC_Addons.tests_and_examples.NeopixelTest;

public class LEDs implements subsystem{
    AdafruitNeopixelSeesaw led1;
    short num_leds = 35;

    int[] colors = new int[num_leds];
    MODES CURRENT_MODE;

    public LEDs(HardwareMap hwmp){
        led1 = hwmp.get(AdafruitNeopixelSeesaw.class, "led1");
        led1.setBufferLength(num_leds); // 36 LEDs
        led1.setPixelType(AdafruitNeopixelSeesaw.ColorOrder.NEO_GRB);
        led1.init_neopixels();
        led1.clearAllPixels();
    }

    public void setSolidColor(int color){
        for (int i = 0; i < num_leds; i++) {
            colors[i] = color;
        }
    }

    public void setAlternatingColor(int color, int color2){
        for (int i = 0; i < num_leds; i++) {
            if (i % 2 == 0){
                colors[i] = color;
            } else{
                colors[i] = color2;
            }
        }
    }

    /**
     * @param color color to twinkle around
     * @param maxSparkle range of brightness values
     * @param brightnessDampen amount to compensate for sparkle to make it look slightly acceptable
     * call every loop
     */
    public void twinkle(int color, byte maxSparkle, byte brightnessDampen){
        for (short i = 0; i < num_leds; i++) {
            byte red = (byte) ((color >> (8*2)) & 0xfe);
            byte green = (byte) ((color >> (8*1)) & 0xfe);
            byte blue = (byte) ((color >> (8*0)) & 0xfe);

            red += (Math.random() * maxSparkle - brightnessDampen);
            green += (Math.random() * maxSparkle - brightnessDampen);
            blue += (Math.random() * maxSparkle - brightnessDampen);

            byte[] combo = {0x00, red, green, blue};
            int newColor = TypeConversion.byteArrayToInt(combo);

            colors[i] = newColor;
        }
    }

    public void twinkle(int color, int maxSparkle, int brightnessDampen){
        twinkle(color, (byte) maxSparkle, (byte) brightnessDampen);
    }
    public void clearAll(){
        for (int i = 0; i < num_leds; i++) {
            colors[i] = 0;
        }
        led1.clearAllPixels();
    }
    @Override
    public void update() {
        for (short i = 0; i < num_leds; i++) {
            led1.setColor(colors[i], i);
        }
    }

    public void setNumLEDs(short num){
        led1.clearAllPixels();
        num_leds = num;
        colors = new int[num_leds];
        led1.setBufferLength(num_leds); // 36 LEDs
        led1.init_neopixels();
    }


    @Override
    public void doTelemetry(Telemetry tele) {
        tele.addData("num leds = ", num_leds);
    }

    enum MODES{
        SOLID,
        TWINKLE;
    }
}
