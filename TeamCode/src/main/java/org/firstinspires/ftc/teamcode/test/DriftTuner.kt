package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.util.InterpLUT
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.Lift
import org.firstinspires.ftc.teamcode.util.OpModeType

@TeleOp
@Config
//@Disabled
class DriftTuner: LinearOpMode() {
    lateinit var drive: SampleMecanumDrive
    override fun runOpMode() {
        var dashboard = FtcDashboard.getInstance()
        drive = SampleMecanumDrive(hardwareMap)
        var yTab = InterpLUT()
        var hTab = InterpLUT()
        var timer = ElapsedTime()
        telemetry = MultipleTelemetry(this.telemetry, dashboard.telemetry)
        waitForStart()
        drive.poseEstimate = Pose2d(0.0, 0.0, 0.0)
        yTab.add(drive.poseEstimate.x, drive.poseEstimate.y)
        hTab.add(drive.poseEstimate.x, Math.toDegrees(drive.poseEstimate.heading))
        while(timer.milliseconds()<3000) {
            drive.setWeightedDrivePower(Pose2d(1.0, 0.0, 0.0))
            yTab.add(drive.poseEstimate.x, drive.poseEstimate.y)
            hTab.add(drive.poseEstimate.x, Math.toDegrees(drive.poseEstimate.heading))
            telemetry.addData("y", drive.poseEstimate.y)
            telemetry.addData("h", Math.toDegrees(drive.poseEstimate.heading))
            telemetry.update()
        }
        drive.setWeightedDrivePower(Pose2d(0.0, 0.0, 0.0))
    }

}