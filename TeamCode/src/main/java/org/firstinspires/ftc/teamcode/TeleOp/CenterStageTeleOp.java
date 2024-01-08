package org.firstinspires.ftc.teamcode.TeleOp;



import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardwareClasses.ArmManager;
import org.firstinspires.ftc.teamcode.hardwareClasses.ClawManager;
import org.firstinspires.ftc.teamcode.hardwareClasses.FieldRelativeControls;

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

    Servo launcher;

    WristManager wrist;
    ArmManager arm;

    ClawManager claw;



    public static double position1 = 400; // in mm
    public static double position2 = 0; // in mm

    public static double thresh = 10.0;// in mm again
    boolean goToTop = false;

    //////////////////////////////////////////////////////////////////////////////
    //TeleOp functions
    //////////////////////////////////////////////////////////////////////////////

    //code to run once driver hits init
    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap);

        launcher = hardwareMap.get(Servo.class, "launcher");
        launcher.setDirection(Servo.Direction.FORWARD);

        slides = new SlidesManager(hardwareMap);

        wrist = new WristManager(hardwareMap);

        arm = new ArmManager(hardwareMap);

        claw = new ClawManager(hardwareMap);

        slides.setThreshold(DistanceUnit.MM, thresh);

        movementInit();
    }

    /**
     * A function to call movement initializations such as zeroing. literally a final init
     */
    public void movementInit(){

        claw.close();
        launcher.setPosition(0.5);
        slides.zeroSlides();
        //wrist.wristAngle = -150;
        //claw.close();
        //wrist.setWristAngle(-120);
        //slides.setHeightAsync(DistanceUnit.INCH, 4.0);
    }

    //code to run repeatedly once driver hits init, before driver hits play
    @Override
    public void init_loop(){
        drive.update();
        wrist.update();
        arm.update();
        claw.update();
        //slides.setHeightAsync(DistanceUnit.MM, 200.0);
        //slides.setHeightAsync(DistanceUnit.INCH, 4.0);
        slides.doTelemetry(telemetry);
        telemetry.update();
    }

    //code to run once driver hits play
    @Override
    public void loop() {
        //slides.setHeightAsync(DistanceUnit.MM, position2);
        drive.update();



        poseEstimate = drive.getPoseEstimate();
        if (gamepad2.a) {
            goToTop = true;
        } else if (gamepad2.b) {
            goToTop = false;
        } else {
            //slides.stop();
        }

        if (gamepad2.b){
            slides.setTarget(DistanceUnit.MM, 400);
        } else if (gamepad2.a) {
            slides.setTarget(DistanceUnit.MM, 0);
        } else if (gamepad2.x){
            slides.zeroSlides();
        } else if (gamepad2.y){
            slides.setTarget(DistanceUnit.MM, 300);
        }



        /*
        if (goToTop){
            slides.setTarget(DistanceUnit.MM, position1);
        } else {
            slides.setTarget(DistanceUnit.MM, position2);
        }
        */
        if (gamepad2.right_trigger != 0){ // pick up mode
            arm.armAngle = 90;
            //wrist.wristAngle = 180 - arm.armAngle;
        } else if (gamepad2.right_bumper) { // place mode
            arm.armAngle = 45;
            //wrist.wristAngle = 90 - arm.armAngle;
        } else {
            arm.armAngle = 0; // neutral mode
            //wrist.wristAngle = 160 - arm.armAngle;
        }

        if (gamepad2.left_bumper){
            claw.open(ClawManager.CLAWS.OUTER);
        } else {
            claw.close();
        }

        if (gamepad2.y){
            launcher.setPosition(0);
        } else{
            launcher.setPosition(0.5);
        }
        /*
        if (gamepad2.y){
            claw.setCosmetic();
        } else if (gamepad2.left_trigger != 0){
            claw.open(ClawManager.CLAWS.INNER);
        } else if (gamepad2.left_bumper) {
            claw.open(ClawManager.CLAWS.OUTER);
        } else if (gamepad2.x){
            claw.close();
        }
        */

        arm.armAngle = gamepad2.right_stick_y * (arm.angleRange/2);
        //arm.armLeft.setPosition((gamepad2.right_stick_y+1.0)/2);
        //arm.armRight.setPosition((gamepad2.right_stick_y+1.0)/2);
        arm.update();
        wrist.update();
        claw.update();
        slides.update();
        stick.fieldRelativeMovement(drive, gamepad1, poseEstimate);


        //slides.left.setPower(-gamepad2.left_stick_y);
        //slides.right.setPower(-gamepad2.right_stick_y);


        arm.doTelemetry(telemetry);
        claw.doTelemetry(telemetry);
        driveTelemetry();
        slides.doTelemetry(telemetry);
        telemetry.update();
    }

    public void stopMovement(){

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
