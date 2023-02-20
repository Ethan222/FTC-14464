package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.RevControlHub;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive", name = "Showcase Tele")
public class SimpleTele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Set up drive train motors and its properties
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Initiate rest of the controls except drive motors
        RevControlHub intake = new RevControlHub();
        intake.init(hardwareMap);

        // Setting up motors to run without Encoders
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {

            // Drive robot with game pad 1
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -.3 * (gamepad1.left_stick_y),
                            -.3 * gamepad1.left_stick_x,
                            -.3 * gamepad1.right_stick_x
                    )
            );

            // 4 stage git. run with gamepad 2
            // Setup motor speed to 0 if it is completely collasped and you are moving it further down.
            if(intake.getArmMotorRotations() < 5 && gamepad1.left_trigger > 0)
            {
                intake.setArmMotorSpeed(0);
            }
            else {
                intake.setArmMotorSpeed(gamepad1.left_trigger > 0 ? gamepad1.left_trigger : -gamepad1.right_trigger);
            }
            // intake
            if(gamepad1.left_bumper) {
                intake.close();
            }
            else if(gamepad1.right_bumper) {
                intake.open();
            }

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("gm2", gamepad2.left_stick_y );
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Rotation of Arm  Motor", intake.getArmMotorRotations());
            telemetry.addData("Arm Y", gamepad2.left_stick_y);
            telemetry.update();
        }
    }
}
