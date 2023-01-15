package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.localizer.T265Localizer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Disabled
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        T265Localizer loc = (T265Localizer) drive.getLocalizer();
        while(!isStarted()){
            telemetry.addData("Confidence", loc.getPoseConfidence());
            telemetry.update();
        }
        waitForStart();

        drive.turn(Math.toRadians(ANGLE));

        T265Localizer.slamera.stop();
        if(isStopRequested()){
            T265Localizer.slamera.stop();
        }
    }
}
