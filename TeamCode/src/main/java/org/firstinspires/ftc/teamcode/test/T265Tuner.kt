package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.arcrobotics.ftclib.geometry.Translation2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.spartronics4915.lib.T265Camera.CameraUpdate
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.drive.localizer.T265Localizer
import org.firstinspires.ftc.teamcode.drive.localizer.T265Localizer.Companion.cameraRobotOffset
import org.firstinspires.ftc.teamcode.drive.localizer.T265Localizer.Companion.slamera
import org.firstinspires.ftc.teamcode.drive.localizer.toFtcLib


/*
* This is a simple routine to test turning capabilities.
*/
@Config //@Disabled
@Autonomous
class T265Tuner : LinearOpMode() {
    val dashboard = FtcDashboard.getInstance()
    override fun runOpMode() {
        val drive = SampleMecanumDrive(hardwareMap)
        val loc = drive.localizer as T265Localizer
        loc.enableMSE = false
        if (isStopRequested) {
            slamera!!.stop()
        }
        waitForStart()
        if (isStopRequested) {
            slamera!!.stop()
        }
        while (opModeIsActive()&&!isStopRequested) {
            drive.turn(Math.toRadians(360.0))
            if (loc.lastCameraRobotOffset != cameraRobotOffset) {
                val cameraRobotOffsetPose = cameraRobotOffset.toFtcLib()
                slamera!!.setOdometryInfo(
                    cameraRobotOffsetPose.translation.x.toFloat(),
                    cameraRobotOffsetPose.translation.y.toFloat(),
                    cameraRobotOffsetPose.rotation.radians.toFloat(),
                    1.0
                    )
                loc.lastCameraRobotOffset = cameraRobotOffset
            }
            val robotRadius = 9.0 // inches


            val packet = TelemetryPacket()
            val field = packet.fieldOverlay()

            val up: CameraUpdate = slamera!!.lastReceivedCameraUpdate

            // We divide by 0.0254 to convert meters to inches

            // We divide by 0.0254 to convert meters to inches
            val translation =
                Translation2d(up.pose.translation.x / 0.0254, up.pose.translation.y / 0.0254)
            val rotation = up.pose.rotation

            field.strokeCircle(translation.x, translation.y, robotRadius)
            val arrowX = rotation.cos * robotRadius
            val arrowY = rotation.sin * robotRadius
            val x1 = translation.x + arrowX / 2
            val y1 = translation.y + arrowY / 2
            val x2 = translation.x + arrowX
            val y2 = translation.y + arrowY
            field.strokeLine(x1, y1, x2, y2)

            dashboard.sendTelemetryPacket(packet)


        }
        if (isStopRequested) {
            slamera!!.stop()
        }
        slamera!!.stop()
    }
}