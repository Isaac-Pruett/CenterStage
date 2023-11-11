package org.firstinspires.ftc.teamcode.TeleOp.opmodes;

import android.transition.Slide;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TeleOp.FieldRelativeControls;

import org.firstinspires.ftc.teamcode.TeleOp.PoseStorage;
import org.firstinspires.ftc.teamcode.TeleOp.SlidesManager;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

//////////////////////////////////////////////////////////////////////////////
// CenterStage TeleOp mode
//////////////////////////////////////////////////////////////////////////////

@TeleOp
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


    //////////////////////////////////////////////////////////////////////////////
    //TeleOp functions
    //////////////////////////////////////////////////////////////////////////////

    //code to run once driver hits init
    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap);

        right = hardwareMap.get(DcMotorEx.class, "right");
        left = hardwareMap.get(DcMotorEx.class, "left");

        lift = new SlidesManager(left, right, 700);
        lift.setThreshold(DistanceUnit.MM, 6.0);


        movementInit();
    }

    /**
     * A function to call movement initializations such as zeroing. literally a final init
     */
    public void movementInit(){

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
        /*
        if (gamepad2.a) {
            lift.setHeightAsync(DistanceUnit.INCH, 10);
        } else if (gamepad2.b) {
            lift.setHeightAsync(DistanceUnit.INCH, 0);
        }
         */

        stick.fieldRelativeMovement(drive, gamepad1, poseEstimate);

        lift.left.setPower(-gamepad2.left_stick_y);
        lift.right.setPower(-gamepad2.right_stick_y);




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
