package org.firstinspires.ftc.teamcode.Auto;



import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.gamepadExpansions.ButtonExpanded;
import org.firstinspires.ftc.teamcode.hardwareClasses.Alliance;
import org.firstinspires.ftc.teamcode.hardwareClasses.ArmAndWristManager;
import org.firstinspires.ftc.teamcode.hardwareClasses.ClawManager;
import org.firstinspires.ftc.teamcode.hardwareClasses.IntakeManager;
import org.firstinspires.ftc.teamcode.hardwareClasses.LEDs;
import org.firstinspires.ftc.teamcode.hardwareClasses.LauncherManager;
import org.firstinspires.ftc.teamcode.hardwareClasses.PoseStorage;
import org.firstinspires.ftc.teamcode.hardwareClasses.SlidesManager;

import java.util.concurrent.TimeUnit;

@Autonomous(preselectTeleOp = "CenterStageTeleOp")
public class CenterStageAutoOp extends OpMode {
    MecanumDrive drive;
    ArmAndWristManager armAndWrist;
    SlidesManager slides;

    IntakeManager intake;

    LauncherManager launcher;
    HuskyLens frontLens;
    Pose2d poseEstimate;

    LEDs leDs;

    boolean hasReadProp = false;
    boolean allianceIsBlue = false;
    boolean allianceIsRed = false;

    boolean block_is_left = false;
    boolean block_is_right = false;
    boolean block_is_center = false;

    boolean hasMovedArm = false;

    double threshhold = 2; // in inches
    boolean hasMovedForward = false;
    boolean hasTurned = false;

    ButtonExpanded debounceLED = new ButtonExpanded();

    @Override
    public void init() {

        frontLens = hardwareMap.get(HuskyLens.class, "frontLens");

        leDs = new LEDs(hardwareMap);
        intake = new IntakeManager(hardwareMap);
        drive = new MecanumDrive(hardwareMap);
        armAndWrist = new ArmAndWristManager(hardwareMap);
        slides = new SlidesManager(hardwareMap);
        launcher = new LauncherManager(hardwareMap);

        init_motion();
    }

    public void init_motion(){
        frontLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        launcher.lock();
        intake.open();
        intake.update();
        slides.zeroSlides();

    }

    @Override
    public void init_loop(){
        HuskyLens.Block[] blocks;
        if (!frontLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + frontLens.getDeviceName());
        } else {
            blocks = frontLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
            }
            /*
            if(blocks[0].id == 2){ // blue
                allianceIsRed = false;
                allianceIsBlue = true;
            } else if (blocks[0].id == 1) {// red
                allianceIsRed = true;
                allianceIsBlue = false;
            }
            */
            if (blocks.length == 1){
                for (int i = 0; i < blocks.length; i++) {
                    HuskyLens.Block current = blocks[i];
                    if ((current.y > 150 && current.y < 180) && (current.height > 28 || current.width > 10)) {
                        //damps out the stage door and enemy blocks
                        //valid block
                        if (current.id == 2) { // blue
                            allianceIsRed = false;
                            allianceIsBlue = true;
                            Alliance.state = Alliance.list.BLUE;
                        } else if (current.id == 1) {// red
                            allianceIsRed = true;
                            allianceIsBlue = false;
                            Alliance.state = Alliance.list.RED;
                        }

                    }
                }
            }

        }
        telemetry.addData("ALLIANCE = ", Alliance.state);

        Alliance.ARE_LEDS_ENABLED = debounceLED.isToggled();
        telemetry.addData("ARE LEDS ENABLED = ", Alliance.ARE_LEDS_ENABLED);

        debounceLED.update(gamepad1.a);
        drive.update();
        telemetry.update();
    }

    @Override
    public void loop() {
        ElapsedTime time = new ElapsedTime();


       while (!hasReadProp){

            intake.close();
            intake.update();

            HuskyLens.Block[] blocks = frontLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
            }
            if (blocks.length >= 1){
                for (int i = 0; i < blocks.length; i++) {
                    HuskyLens.Block current = blocks[i];

                    if ((current.y > 150 && current.y < 180) && (current.height > 28 || current.width > 10)){
                        //damps out the stage door and enemy blocks
                        //valid block

                        if(current.id == 2){ // blue
                            allianceIsRed = false;
                            allianceIsBlue = true;
                            Alliance.state = Alliance.list.BLUE;
                        } else if (current.id == 1) {// red
                            allianceIsRed = true;
                            allianceIsBlue = false;
                            Alliance.state = Alliance.list.RED;
                        }

                        if (current.x >= 0 && current.x <= 60){
                            block_is_left = true;
                        } else if (current.x > 60 && current.x <= (320-60)){
                            block_is_center = true;
                        } else if (current.x > (320-60) && current.x <= 320) {
                            block_is_right = true;
                        }

                        if (block_is_center) {
                            telemetry.addData("position = ", "center");
                        } else if (block_is_left) {
                            telemetry.addData("position = ", "left");
                        } else if (block_is_right) {
                            telemetry.addData("position = ", "right");
                        }

                        hasReadProp = true;
                        if (Alliance.ARE_LEDS_ENABLED && Alliance.state == Alliance.list.BLUE){
                            leDs.setAlternatingColor(0x0000ff, 0x3311ff);
                        } else if (Alliance.ARE_LEDS_ENABLED && Alliance.state == Alliance.list.RED) {
                            leDs.setAlternatingColor(0xff0000, 0xff2200);
                        } else if (gamepad1.a){
                            leDs.clearAll();
                        }
                    }
                }
            }
            telemetry.update();
        }
        if (block_is_center) {
            telemetry.addData("position = ", "center");
        } else if (block_is_left) {
            telemetry.addData("position = ", "left");
        } else if (block_is_right) {
            telemetry.addData("position = ", "right");
        }
        telemetry.update();
        time.reset();

        poseEstimate = drive.getPoseEstimate();
        Pose2d target = new Pose2d(poseEstimate.getX() + 27, poseEstimate.getY());

        while (!hasMovedForward){
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            if (Math.hypot(poseEstimate.getX() - target.getX(), poseEstimate.getY() - target.getY()) > threshhold){
                Vector2d input = new Vector2d(
                        poseEstimate.getX() - target.getX(),
                        poseEstimate.getY() - target.getY()
                ).rotated(poseEstimate.getHeading());
                telemetry.addData("raw input x", input.getX());
                telemetry.addData("raw input y", input.getY());
                double norm = input.norm();
                input = input.div(norm);
                input = input.times(0.5);

                telemetry.addData("Target x", target.getX());
                telemetry.addData("Target y", target.getY());
                telemetry.addData("input x", input.getX());
                telemetry.addData("input y", input.getY());


                drive.setWeightedDrivePower(
                        new Pose2d(
                                -input.getX(),
                                -input.getY()
                        )
                );
                drive.update();
                poseEstimate = drive.getPoseEstimate();
            } else {
                hasMovedForward = true;
                drive.setWeightedDrivePower(new Pose2d(0,0));
                drive.update();
            }
            telemetry.update();
        }

        poseEstimate = drive.getPoseEstimate();

        if(block_is_center){
            target = new Pose2d(poseEstimate.getX() + 2, poseEstimate.getY() - 0, poseEstimate.getHeading()+0);
        } else if (block_is_right){
            target = new Pose2d(poseEstimate.getX(), poseEstimate.getY() - 11, poseEstimate.getHeading()+0);
        } else if (block_is_left){
            target = new Pose2d(poseEstimate.getX(), poseEstimate.getY() + 11, poseEstimate.getHeading()+0);
        }

        while (!hasTurned){

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addLine("AAASDSDSDGSDFSA");
            if (Math.hypot(poseEstimate.getX() - target.getX(), poseEstimate.getY() - target.getY()) > 1){
                Vector2d input = new Vector2d(
                        poseEstimate.getX() - target.getX(),
                        poseEstimate.getY() - target.getY()

                ).rotated(poseEstimate.getHeading());

                telemetry.addData("has reached target = ", Math.hypot(poseEstimate.getX() - target.getX(), poseEstimate.getY() - target.getY()) > 1);
                telemetry.addData("target hypot", Math.hypot(poseEstimate.getX() - target.getX(), poseEstimate.getY() - target.getY()));

                telemetry.addData("raw input x", input.getX());
                telemetry.addData("raw input y", input.getY());
                telemetry.addData("raw input heading = ", (poseEstimate.getHeading()-target.getHeading())/(Math.PI));

                double norm = input.norm();
                input = input.div(norm);
                input = input.times(0.5);

                telemetry.addData("Target x", target.getX());
                telemetry.addData("Target y", target.getY());
                telemetry.addData("Target heading", target.getHeading());
                telemetry.addData("input x", input.getX());
                telemetry.addData("input y", input.getY());
                telemetry.addData("input heading", (poseEstimate.getHeading()-target.getHeading())/(Math.PI) * .3);
                telemetry.addLine("sdljfaslhdfoisdfuasbdrubsdufbsad");


                drive.setWeightedDrivePower(
                        new Pose2d(
                                -input.getX(),
                                -input.getY(),
                                (poseEstimate.getHeading()-target.getHeading())/(Math.PI)*.3
                        )
                );
                drive.update();
                poseEstimate = drive.getPoseEstimate();
            } else {
                time.reset();
                intake.open();
                while (time.milliseconds() < 2000){
                    drive.update();
                }

                time.reset();
                drive.setWeightedDrivePower(new Pose2d(-1, 0));
                while (time.milliseconds() < 300){
                    drive.update();
                }

                hasTurned = true;
                drive.setWeightedDrivePower(new Pose2d(0,0, 0));
                drive.update();
            }
            telemetry.update();
        }


        if (block_is_center) {
            telemetry.addData("position = ", "center");
        } else if (block_is_left) {
            telemetry.addData("position = ", "left");
        } else if (block_is_right) {
            telemetry.addData("position = ", "right");
        }
        drive.update();
        poseEstimate = drive.getPoseEstimate();
        PoseStorage.storedPose = poseEstimate;
        telemetry.update();
    }


    @Override
    public void stop() {


        drive.update();
        poseEstimate = drive.getPoseEstimate();
        PoseStorage.storedPose = poseEstimate;
    }
}
