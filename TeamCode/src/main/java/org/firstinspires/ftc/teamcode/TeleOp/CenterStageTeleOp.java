package org.firstinspires.ftc.teamcode.TeleOp;



import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardwareClasses.FieldRelativeControls;

import org.firstinspires.ftc.teamcode.hardwareClasses.PoseStorage;
import org.firstinspires.ftc.teamcode.hardwareClasses.SlidesManager;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

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
    DcMotorEx left;
    DcMotorEx right;
    SlidesManager lift;
    FieldRelativeControls stick = new FieldRelativeControls();

    Servo launcher;



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

        right = hardwareMap.get(DcMotorEx.class, "right");
        left = hardwareMap.get(DcMotorEx.class, "left");

        lift = new SlidesManager(left, right, 700);


        movementInit();
    }

    /**
     * A function to call movement initializations such as zeroing. literally a final init
     */
    public void movementInit(){

        launcher.setPosition(1.0);
    }

    //code to run repeatedly once driver hits init, before driver hits play
    @Override
    public void init_loop(){
        drive.update();
        //lift.setHeightAsync(DistanceUnit.MM, 200.0);
        lift.doTelemetry(telemetry);
        telemetry.update();
    }

    //code to run once driver hits play
    @Override
    public void loop() {
        drive.update();

        poseEstimate = drive.getPoseEstimate();


        if (gamepad2.a) {
            goToTop = true;
        } else if (gamepad2.b) {
            goToTop = false;
        } else {
            //lift.stop();
        }
        if (goToTop){
            lift.setHeightAsync(DistanceUnit.MM, position1);
        } else{
            lift.setHeightAsync(DistanceUnit.MM, position2);
        }


        if (gamepad2.right_trigger != 0){
            launcher.setPosition(0.5);
        } else{
            launcher.setPosition(1.0);
        }


        stick.fieldRelativeMovement(drive, gamepad1, poseEstimate);

        //lift.left.setPower(-gamepad2.left_stick_y);
        //lift.right.setPower(-gamepad2.right_stick_y);




        driveTelemetry();
        lift.doTelemetry(telemetry);
        telemetry.update();
    }

    //code to run once driver hits stop
    @Override
    public void stop(){
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
