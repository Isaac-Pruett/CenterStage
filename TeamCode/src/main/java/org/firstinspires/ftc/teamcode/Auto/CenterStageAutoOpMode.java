package org.firstinspires.ftc.teamcode.Auto;



import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardwareClasses.ArmAndWristManager;
import org.firstinspires.ftc.teamcode.hardwareClasses.ClawManager;
import org.firstinspires.ftc.teamcode.hardwareClasses.PoseStorage;
import org.firstinspires.ftc.teamcode.hardwareClasses.SlidesManager;

@Autonomous(preselectTeleOp = "CenterStageTeleOp")
public class CenterStageAutoOpMode extends OpMode {
    MecanumDrive drive;
    ArmAndWristManager armAndWrist;
    SlidesManager slides;
    ClawManager claw;
    Pose2d poseEstimate;


    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap);
        armAndWrist = new ArmAndWristManager(hardwareMap);
        slides = new SlidesManager(hardwareMap);
        claw = new ClawManager(hardwareMap);

        init_motion();
    }

    public void init_motion(){
        claw.setCosmetic();
        slides.setTarget(DistanceUnit.MM, 0);

    }

    @Override
    public void init_loop(){
        slides.update();
        drive.update();

    }

    @Override
    public void loop() {
        drive.update();
        poseEstimate = drive.getPoseEstimate();
    }

    @Override
    public void stop() {


        drive.update();
        poseEstimate = drive.getPoseEstimate();
        PoseStorage.storedPose = poseEstimate;
    }
}
