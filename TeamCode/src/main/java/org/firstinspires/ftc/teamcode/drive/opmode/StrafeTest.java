package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.localizer.T265Localizer;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Disabled
@Autonomous(group = "drive")
public class StrafeTest extends LinearOpMode {
    public static double DISTANCE = 60; // in

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        IntakeExtension intake = new IntakeExtension(hardwareMap, telemetry);
        intake.retract();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        /*T265Localizer.slamera.setOdometryInfo((float) T265Localizer.cameraRobotOffset.getX(), (float) T265Localizer.cameraRobotOffset.getY(), (float) T265Localizer.cameraRobotOffset.getHeading(), 0.0);
        ((T265Localizer) drive.getLocalizer()).setEnableMSE(false);*/


        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(DISTANCE)
                .build();
        /*T265Localizer loc = (T265Localizer) drive.getLocalizer();
        while(!isStarted()){
            telemetry.addData("Confidence", loc.getPoseConfidence());
            telemetry.update();
        }*/
        waitForStart();

        drive.followTrajectory(trajectory);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;

        if(isStopRequested()){
            T265Localizer.slamera.stop();
        }
    }
}
