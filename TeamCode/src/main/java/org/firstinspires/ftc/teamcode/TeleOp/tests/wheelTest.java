package org.firstinspires.ftc.teamcode.TeleOp.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

@TeleOp
public class wheelTest extends LinearOpMode {
    MecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            drive.setMotorPowers(1,0,0,0);
            sleep(3000);
            drive.setMotorPowers(0,1,0,0);
            sleep(3000);
            drive.setMotorPowers(0,0,1,0);
            sleep(3000);
            drive.setMotorPowers(0,0,0,1);
            sleep(3000);
        }

    }
}
