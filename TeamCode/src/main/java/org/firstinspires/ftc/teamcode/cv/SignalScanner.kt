package org.firstinspires.ftc.teamcode.cv

import android.util.Log
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.autonomous.Positions.P1
import org.firstinspires.ftc.teamcode.autonomous.Positions.P2
import org.firstinspires.ftc.teamcode.autonomous.Positions.P3
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam


class SignalScanner(hardwareMap: HardwareMap, telemetry: Telemetry){
    var webcam: OpenCvWebcam
    var hardwareMap: HardwareMap
    var telemetry: Telemetry
    var fx = 578.272
    var fy = 578.272
    var cx = 402.145
    var cy = 221.506
    var tagsize = 0.166
    private val pipeline = AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
    init {

        this.hardwareMap = hardwareMap
        this.telemetry = telemetry
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName
        )
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap.get(
                WebcamName::class.java, "Webcam 1"
            ), cameraMonitorViewId
        )


        webcam.setPipeline(pipeline)


        webcam.setMillisecondsPermissionTimeout(2500) // Timeout for obtaining permission is configurable. Set before opening.
        webcam.setMillisecondsPermissionTimeout(2500) // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(
            object : OpenCvCamera.AsyncCameraOpenListener {
                override fun onOpened() {
                    webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT)
                }

                override fun onError(errorCode: Int) {
                    /*
                 * This will be called if the camera could not be opened
                 */
                    Log.d("CameraError", "Error: $errorCode")
                }
            })
        FtcDashboard.getInstance().startCameraStream(webcam, 60.0)
    }

    fun scanBarcode(): Pair<Vector2d, Int> {
        var detections = pipeline.latestDetections
        for(detection in detections){
            if (detection.id == 1){
                return Pair(P1, 1)
            }
            if (detection.id == 2){
                return Pair(P2, 2)
            }
            if (detection.id == 3){
                return Pair(P3, 3)
            }
        }
        return Pair(P1, 0)
    }
    fun stop() {
        FtcDashboard.getInstance().stopCameraStream()
        this.webcam.stopStreaming()
        this.webcam.closeCameraDevice()
    }
}