package org.firstinspires.ftc.teamcode.TeleOp.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.TeleOp.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;

@TeleOp
@Disabled
public class FieldRelativeDriverControls extends LinearOpMode {

    MecanumDrive drive;
    Pose2d poseEstimate;

    DcMotorEx left;
    DcMotorEx right;
    double pulley_diameter = 38.2;
    double counts_per_rev = 384.5;


    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap);
        right = hardwareMap.get(DcMotorEx.class, "right");
        left = hardwareMap.get(DcMotorEx.class, "left");
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        waitForStart();
        if (opModeIsActive()){
            while (opModeIsActive() && !isStopRequested()) {
                drive.update();

                poseEstimate = drive.getPoseEstimate();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", poseEstimate.getHeading());

                fieldRelativeMovement();

                left.setPower(-gamepad2.left_stick_y);
                right.setPower(gamepad2.right_stick_y);
                telemetry.addData("l:", (pulley_diameter * Math.PI / counts_per_rev) * (left.getCurrentPosition()));
                telemetry.addData("r:", (pulley_diameter * Math.PI / counts_per_rev) * (-right.getCurrentPosition()));
                telemetry.update();

            }
        }
        PoseStorage.storedPose = poseEstimate;
    }

    public void fieldRelativeMovement(){
        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x
        );//.rotated(-poseEstimate.getHeading());


        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x
                )
        );
    }
}
