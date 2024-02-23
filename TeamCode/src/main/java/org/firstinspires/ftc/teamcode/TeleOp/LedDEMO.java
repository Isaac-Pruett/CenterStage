package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardwareClasses.LEDs;

@TeleOp
public class LedDEMO extends LinearOpMode {
    LEDs ledController;
    @Override
    public void runOpMode() throws InterruptedException {
        ledController = new LEDs(hardwareMap);

        ledController.clearAll();
        int counter = 0;
        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            if (counter >= 30){
                ledController.setAlternatingColor(0xff00ff, 0x6600ee);
            } else{
                ledController.twinkle(0x6600ee, 26, 14);
            }
            //ledController.setAlternatingColor(0xff00ff, 0x6600ee);
            ledController.update();
            counter++;
            sleep(1000);



        }
        ledController.clearAll();
    }
}
