package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.gamepadExpansions.ButtonExpanded;
import org.firstinspires.ftc.teamcode.hardwareClasses.Alliance;
import org.firstinspires.ftc.teamcode.hardwareClasses.ArmAndWristManager;
import org.firstinspires.ftc.teamcode.hardwareClasses.ArmManager;
//import org.firstinspires.ftc.teamcode.hardwareClasses.ClawManager;
import org.firstinspires.ftc.teamcode.hardwareClasses.FieldRelativeControls;

import org.firstinspires.ftc.teamcode.hardwareClasses.IntakeManager;
import org.firstinspires.ftc.teamcode.hardwareClasses.LEDs;
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
    ArmAndWristManager armAndWrist;
    LauncherManager launcher;

    LEDs leDs;

    //ClawManager claw;
    public static double thresh = 10.0;// in mm again


    public double armINCR = 5;

    public double slidesTarget = 0;

    public double slidesINCR = 42.0;  // 84.0/2.0 // in mm

    double speed = 0.7;
    PlungerManager plunger;
    IntakeManager intake;


    ButtonExpanded slidesDownButton = new ButtonExpanded();
    ButtonExpanded slidesUpButton = new ButtonExpanded();
    ButtonExpanded plungerOutButton = new ButtonExpanded();
    ButtonExpanded plungerInButton = new ButtonExpanded();
    ButtonExpanded depositModeButton = new ButtonExpanded();
    ButtonExpanded collectModeButton = new ButtonExpanded();

    ButtonExpanded intakeMoveButton = new ButtonExpanded();

    ButtonExpanded hangButton = new ButtonExpanded();

    //////////////////////////////////////////////////////////////////////////////
    //TeleOp functions
    //////////////////////////////////////////////////////////////////////////////

    //code to run once driver hits init
    @Override
    public void init() {

        drive = new MecanumDrive(hardwareMap);

        launcher = new LauncherManager(hardwareMap);

        slides = new SlidesManager(hardwareMap);

        armAndWrist = new ArmAndWristManager(hardwareMap);

        plunger = new PlungerManager(hardwareMap);

        //claw = new ClawManager(hardwareMap);

        slides.setThreshold(DistanceUnit.MM, thresh);

        intake = new IntakeManager(hardwareMap);

        leDs = new LEDs(hardwareMap);

        movementInit();
    }

    /**
     * A function to call movement initializations such as zeroing. literally a final init before init_loop()
     */
    public void movementInit(){
        plunger.extendFully();
        plunger.update();
        intake.update();
        armAndWrist.arm.setArmAngle(0.0);

        leDs.clearAll();
        //claw.setCosmetic();

        launcher.lock();

        slides.zeroSlides();

        armAndWrist.setMode(ArmAndWristManager.MODE.POKE);
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

        if (gamepad1.x){
            leDs.setAlternatingColor(0x0000ff, 0x3311ff);
            Alliance.ARE_LEDS_ENABLED = true;
            Alliance.state = Alliance.list.BLUE;
        } else if (gamepad1.b) {
            leDs.setAlternatingColor(0xff0000, 0xff2200);
            Alliance.state = Alliance.list.RED;
        } else if (gamepad1.a){
            leDs.clearAll();
            Alliance.ARE_LEDS_ENABLED = false;
        } else if (Alliance.ARE_LEDS_ENABLED && Alliance.state == Alliance.list.BLUE){
            leDs.setAlternatingColor(0x0000ff, 0x3311ff);
        } else if (Alliance.ARE_LEDS_ENABLED && Alliance.state == Alliance.list.RED) {
            leDs.setAlternatingColor(0xff0000, 0xff2200);
        }
        leDs.update();


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

        armAndWrist.arm.setArmAngle(armAndWrist.arm.armAngle + (armINCR * (gamepad2.left_stick_y * -1.0)));

        if (gamepad2.right_stick_button){
            launcher.fire();
        } else if (gamepad2.start) {
            launcher.lock();
        }

        if (gamepad2.left_bumper && plungerInButton.isChanged(gamepad2.left_bumper)){ //&& plungerInButton.isChanged(gamepad2.left_bumper)
            plunger.retract();
        } else if (gamepad2.left_trigger != 0 && plungerOutButton.isChanged(gamepad2.left_trigger != 0)){ //&& plungerOutButton.isChanged(gamepad2.left_trigger != 0)
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

        if (!intakeMoveButton.isToggled()){
            intake.open();
        } else{
            intake.close();
        }

        if (gamepad2.a){
            setupForPoke();
        }

        slidesTarget = slides.dU.toMm(slides.getTarget());
        if (gamepad2.right_trigger != 0 && (slidesTarget - slidesINCR) >= 0 && slidesDownButton.isChanged(gamepad2.right_trigger != 0)){ //  && slidesDownButton.isChanged(gamepad2.right_trigger != 0))
            slidesTarget -= slidesINCR;
            slides.setTarget(DistanceUnit.MM, slidesTarget);
        } else if (gamepad2.right_bumper && slidesUpButton.isChanged(gamepad2.right_bumper)){ //  && slidesUpButton.isChanged(gamepad2.right_bumper)
            slidesTarget += slidesINCR;
            slides.setTarget(DistanceUnit.MM, slidesTarget);
        } else if (gamepad2.x) {
            slides.setTarget(DistanceUnit.MM, 0);
        } else if (gamepad2.b){
            slides.zeroSlides();
        } else if (gamepad2.y && hangButton.isChanged(gamepad2.y)){
            slides.setTarget(DistanceUnit.MM, 400);
        }

        if (depositModeButton.isChanged(gamepad2.dpad_up) && gamepad2.dpad_up){
            armAndWrist.setMode(ArmAndWristManager.MODE.PLACING);
            //armAndWrist.arm.setArmAngle(30.0);
        } else if (collectModeButton.isChanged(gamepad2.dpad_down) && gamepad2.dpad_down){
            armAndWrist.setMode(ArmAndWristManager.MODE.FLOOR_FRONT);
            //armAndWrist.arm.setArmAngle(-105.0);
        }



        //claw.innerPos = ((gamepad2.left_stick_y + 1) / 2);
        //claw.outerPos = ((gamepad2.right_stick_y + 1) / 2);
        //arm.armAngle = gamepad2.right_stick_y * (arm.angleRange/2);

        if (gamepad1.right_trigger != 0 && gamepad1.left_trigger != 0){
            speed = .8;
        }else{
            speed = .65;
        }


        //claw.update();

        updateSubsystems();
        updateControls();

        //claw.doTelemetry(telemetry);
        doAllTelemetry();
        //update call
        telemetry.update();
    }

    public void updateSubsystems(){
        armAndWrist.update();
        slides.update();
        launcher.update();
        plunger.update();
        intake.update();
        //updates the drive power here
        stick.fieldRelativeMovement(drive, gamepad1, poseEstimate, speed);
    }

    public void updateControls(){
        plungerOutButton.update(gamepad2.left_trigger != 0);
        plungerInButton.update(gamepad2.left_bumper);
        slidesDownButton.update(gamepad2.right_trigger != 0);
        slidesUpButton.update(gamepad2.right_bumper);
        depositModeButton.update(gamepad2.dpad_up);
        collectModeButton.update(gamepad2.dpad_down);
        intakeMoveButton.update(gamepad1.y);
        hangButton.update(gamepad2.y);
    }

    public void doAllTelemetry(){
        armAndWrist.doTelemetry(telemetry);
        slides.doTelemetry(telemetry);
        launcher.doTelemetry(telemetry);
        plunger.doTelemetry(telemetry);
        intake.doTelemetry(telemetry);
        //drive.doTelemetry() more or less.
        driveTelemetry();
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

    public void setupForPoke(){
        if (!intakeMoveButton.isToggled()){
            intakeMoveButton.update(true);
        }
        intake.close();
        //
        //plunger.retractFully();
        intake.update();
        plunger.update();

        drive.setWeightedDrivePower(new Pose2d(0,0));

        slides.setTarget(DistanceUnit.MM, 312);
        while (!slides.isAtTarget()){
            telemetry.addLine("IM IN A LOOP AAAAAA");
            updateSubsystems();
        }
        armAndWrist.setMode(ArmAndWristManager.MODE.POKE);

        int num_iterations = 200;
        double target = -165;
        double current_angle = armAndWrist.arm.armAngle;
        double difference = current_angle - target;
        double incr = difference/num_iterations;
        ElapsedTime time = new ElapsedTime();
        time.reset();
        for (int i = 0; i < num_iterations; i++) {
            armAndWrist.arm.setArmAngle(current_angle - incr*i);
            while(time.milliseconds() < 10){
                time.reset();
                doAllTelemetry();
                drive.update();
            }
            updateSubsystems();
        }
            armAndWrist.update();
        plunger.extendFully();
        plunger.update();

    }

}
