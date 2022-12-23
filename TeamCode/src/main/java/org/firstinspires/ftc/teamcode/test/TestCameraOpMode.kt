package org.firstinspires.ftc.teamcode.test

import android.os.Build
import android.util.Log
import androidx.annotation.RequiresApi
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.arcrobotics.ftclib.geometry.Transform2d
import com.arcrobotics.ftclib.geometry.Translation2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.spartronics4915.lib.T265Camera
import java.nio.file.Paths

@TeleOp(name = "Test T265", group = "Iterative Opmode")
class TestCameraOpMode : OpMode() {
    private val dashboard = FtcDashboard.getInstance()
    override fun init() {
        if (slamra == null) {
            slamra = T265Camera(Transform2d(), 0.1, hardwareMap.appContext)
        }
    }

    override fun init_loop() {}
    override fun start() {
        slamra!!.start()
    }

    override fun loop() {
        val robotRadius = 9 // inches
        val packet = TelemetryPacket()
        val field = packet.fieldOverlay()
        val up = slamra!!.lastReceivedCameraUpdate ?: return

        // We divide by 0.0254 to convert meters to inches
        val translation =
            Translation2d(up.pose.translation.x / 0.0254, up.pose.translation.y / 0.0254)
        val rotation = up.pose.rotation
        field.strokeCircle(translation.x, translation.y, robotRadius.toDouble())
        val arrowX = rotation.cos * robotRadius
        val arrowY = rotation.sin * robotRadius
        val x1 = translation.x + arrowX / 2
        val y1 = translation.y + arrowY / 2
        val x2 = translation.x + arrowX
        val y2 = translation.y + arrowY
        field.strokeLine(x1, y1, x2, y2)
        dashboard.sendTelemetryPacket(packet)
    }

    override fun stop() {
        slamra!!.stop()
    }

    companion object {
        // We treat this like a singleton because there should only ever be one object per camera
        private var slamra: T265Camera? = null
    }
}