package org.firstinspires.ftc.teamcode.IMUAutoTest;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class Imu3DeadWheelTest extends LinearOpMode {


    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        // vision here that outputs position
        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .turn(Math.toRadians(360))
                .waitSeconds(3)
                .turn(Math.toRadians(360))
                .waitSeconds(3)
                .turn(Math.toRadians(360))
                .waitSeconds(3);


        // actions that need to happen on init; for instance, a claw tightening.


        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;

        trajectoryActionChosen = tab1.build();
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen
                )
        );

   while(opModeIsActive()) {
       drive.updatePoseEstimate();

       Pose2d pose = drive.localizer.getPose();
       telemetry.addData("x", pose.position.x);
       telemetry.addData("y", pose.position.y);
       telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
       telemetry.update();

       TelemetryPacket packet = new TelemetryPacket();
       packet.fieldOverlay().setStroke("#3F51B5");
       Drawing.drawRobot(packet.fieldOverlay(), pose);
       FtcDashboard.getInstance().sendTelemetryPacket(packet);


   }
    }
}

