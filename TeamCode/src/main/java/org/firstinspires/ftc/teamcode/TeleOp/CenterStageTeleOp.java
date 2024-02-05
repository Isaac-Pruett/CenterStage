package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.gamepadExpansions.ButtonExpanded;
import org.firstinspires.ftc.teamcode.hardwareClasses.ArmManager;
import org.firstinspires.ftc.teamcode.hardwareClasses.ClawManager;
import org.firstinspires.ftc.teamcode.hardwareClasses.FieldRelativeControls;

import org.firstinspires.ftc.teamcode.hardwareClasses.LauncherManager;
import org.firstinspires.ftc.teamcode.hardwareClasses.PlungerManager;
import org.firstinspires.ftc.teamcode.hardwareClasses.PoseStorage;
import org.firstinspires.ftc.teamcode.hardwareClasses.SlidesManager;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardwareClasses.WristManager;


//////////////////////////////////////////////////////////////////////////////
// CenterStage TeleOp mode
//////////////////////////////////////////////////////////////////////////////

@TeleOp
@Config
public class CenterStageTeleOp extends OpMode {

    //////////////////////////////////////////////////////////////////////////////
    //fields and variables
    //////////////////////////////////////////////////////////////////////////////

    MecanumDrive drive;
    Pose2d poseEstimate;
    SlidesManager slides;
    FieldRelativeControls stick = new FieldRelativeControls();
    WristManager wrist;
    ArmManager arm;
    LauncherManager launcher;
    //ClawManager claw;
    public static double thresh = 10.0;// in mm again

    public double armANG = 0;

    public double armINCR = 2.5;
    public double wristANG = 0;
    public double wristINCR = 2.5;
    public double slidesTarget = 0;

    public double slidesINCR = 42.0;  // 84.0/2.0 // in mm

    double speed = 0.7;
    PlungerManager plunger;


    ButtonExpanded slidesDownButton = new ButtonExpanded();
    ButtonExpanded slidesUpButton = new ButtonExpanded();
    ButtonExpanded armUpButton = new ButtonExpanded();
    ButtonExpanded armDownButton = new ButtonExpanded();

    ButtonExpanded wristUpButton = new ButtonExpanded();
    ButtonExpanded wristDownButton = new ButtonExpanded();
    ButtonExpanded plungerOutButton = new ButtonExpanded();
    ButtonExpanded plungerInButton = new ButtonExpanded();

    //////////////////////////////////////////////////////////////////////////////
    //TeleOp functions
    //////////////////////////////////////////////////////////////////////////////

    //code to run once driver hits init
    @Override
    public void init() {

        drive = new MecanumDrive(hardwareMap);

        launcher = new LauncherManager(hardwareMap);

        slides = new SlidesManager(hardwareMap);

        wrist = new WristManager(hardwareMap);

        arm = new ArmManager(hardwareMap);

        plunger = new PlungerManager(hardwareMap);

        //claw = new ClawManager(hardwareMap);

        slides.setThreshold(DistanceUnit.MM, thresh);

        movementInit();
    }

    /**
     * A function to call movement initializations such as zeroing. literally a final init before init_loop()
     */
    public void movementInit(){
        plunger.extendFully();
        plunger.update();

        //claw.setCosmetic();

        launcher.lock();

        slides.zeroSlides();
    }

    //code to run repeatedly once driver hits init, before driver hits play
    @Override
    public void init_loop(){
        /*
        if (gamepad2.x){
            claw.setCosmetic();
        } else if (gamepad2.left_stick_button){
            claw.close();
        }*/

        if (gamepad2.right_stick_button){
            launcher.fire();
        } else if (gamepad2.start) {
            launcher.lock();
        }

        drive.update();
        //wrist.update();
        //arm.update();
        //claw.update();

        slides.doTelemetry(telemetry);
        telemetry.update();
    }

    //code to run once driver hits play
    @Override
    public void loop() {

        drive.update();
        poseEstimate = drive.getPoseEstimate();

        /*
        if (gamepad2.b){
            slides.setTarget(DistanceUnit.MM, 400);
        } else if (gamepad2.a) {
            slides.setTarget(DistanceUnit.MM, 0);
        } else if (gamepad2.x){
            slides.zeroSlides();
        } else if (gamepad2.y){
            slides.setTarget(DistanceUnit.MM, 300);
        }
        */

        slidesDownButton.update(gamepad2.right_trigger != 0);
        slidesUpButton.update(gamepad2.right_bumper);
        armDownButton.update(gamepad2.a);
        armUpButton.update(gamepad2.y);
        wristDownButton.update(gamepad2.dpad_down);
        wristUpButton.update(gamepad2.dpad_up);
        plungerOutButton.update(gamepad2.left_trigger != 0);
        plungerInButton.update(gamepad2.left_bumper);



        if (gamepad2.right_stick_button){
            launcher.fire();
        } else if (gamepad2.start) {
            launcher.lock();
        }
        if (gamepad2.left_bumper && plungerInButton.isChanged(gamepad2.left_bumper)){
            plunger.retract();
        } else if (gamepad2.left_trigger != 0 && plungerOutButton.isChanged(gamepad2.left_trigger != 0)){
            plunger.extend();
        }
        /*
        if (gamepad2.y){
            claw.setCosmetic();
        } else */
        /*
        if (gamepad2.left_trigger != 0){
            claw.open(ClawManager.CLAWS.OUTER);
        } else if (gamepad2.left_bumper) {
            claw.open(ClawManager.CLAWS.INNER);
        } else if (gamepad2.left_stick_button){
            claw.close();
        } */
        slidesTarget = slides.dU.toMm(slides.getTarget());
        if (gamepad2.right_trigger != 0 && slidesDownButton.isChanged(gamepad2.right_trigger != 0) && ((slidesTarget - slidesINCR) >= 0)){
            slidesTarget -= slidesINCR;
            slides.setTarget(DistanceUnit.MM, slidesTarget);
        } else if (gamepad2.right_bumper && slidesUpButton.isChanged(gamepad2.right_bumper)){
            slidesTarget += slidesINCR;
            slides.setTarget(DistanceUnit.MM, slidesTarget);
        } else if (gamepad2.x) {
            slides.setTarget(DistanceUnit.MM, 0);
        } else if (gamepad2.b){
            slides.zeroSlides();
        }



        wristANG = wrist.getWristAngle();
        if (arm.armAngle >= 0) {
            if (gamepad2.dpad_up && wristUpButton.isChanged(gamepad2.dpad_up)) {
                wristANG += wristINCR;
            } else if (gamepad2.dpad_down && wristDownButton.isChanged(gamepad2.dpad_down)) {
                wristANG -= wristINCR;
            }
        } else {
            if (gamepad2.dpad_up && wristUpButton.isChanged(gamepad2.dpad_up)) {
                wristANG -= wristINCR;
            } else if (gamepad2.dpad_down && wristDownButton.isChanged(gamepad2.dpad_down)) {
                wristANG += wristINCR;
            }
        }
        wrist.setWristAngle(wristANG);



        armANG = arm.armAngle;
        if (gamepad2.y && armUpButton.isChanged(gamepad2.y)){
            armANG += armINCR;
        } else if (gamepad2.a && armUpButton.isChanged(gamepad2.a)) {
            armANG -= armINCR;
        }
        arm.setArmAngle(armANG);


        //claw.innerPos = ((gamepad2.left_stick_y + 1) / 2);
        //claw.outerPos = ((gamepad2.right_stick_y + 1) / 2);
        //arm.armAngle = gamepad2.right_stick_y * (arm.angleRange/2);

        if (gamepad1.right_trigger != 0 && gamepad1.left_trigger != 0){
            speed = .8;
        }else{
            speed = .6;
        }


        //claw.update();
        wrist.update();
        arm.update();
        slides.update();
        launcher.update();
        //updates the drive power here
        stick.fieldRelativeMovement(drive, gamepad1, poseEstimate, speed);



        //claw.doTelemetry(telemetry);
        wrist.doTelemetry(telemetry);
        arm.doTelemetry(telemetry);
        slides.doTelemetry(telemetry);
        launcher.doTelemetry(telemetry);
        //drive.doTelemetry() more or less.
        driveTelemetry();

        //update call
        telemetry.update();
    }

    public void stopMovement(){
        //arm.setArmAngle(0.0);
    }

    //code to run once driver hits stop
    @Override
    public void stop(){
        stopMovement();
        drive.update();
        PoseStorage.storedPose = drive.getPoseEstimate();

    }

    public void driveTelemetry(){
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
    }

    //////////////////////////////////////////////////////////////////////////////
    //
    //////////////////////////////////////////////////////////////////////////////
}
