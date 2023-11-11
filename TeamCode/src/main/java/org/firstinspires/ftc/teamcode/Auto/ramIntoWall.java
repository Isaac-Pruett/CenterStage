package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

@Autonomous(name="garbage", group="aaaaa")
public class ramIntoWall extends LinearOpMode {

    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (!isStopRequested() && opModeIsActive()){
            drive.setWeightedDrivePower(new Pose2d(0, 1.0));
            sleep(3000);
            drive.setWeightedDrivePower(new Pose2d(0,0));
        }
    }
}
