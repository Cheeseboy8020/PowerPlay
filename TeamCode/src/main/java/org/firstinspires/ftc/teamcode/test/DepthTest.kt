package org.firstinspires.ftc.teamcode.test

import android.util.Log
import com.acmerobotics.dashboard.FtcDashboard
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.geometry.Transform2d
import com.arcrobotics.ftclib.geometry.Translation2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.HardwareMap
import com.spartronics4915.lib.T265Camera
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.cv.SignalScanner
import org.firstinspires.ftc.teamcode.drive.localizer.T265Localizer
import org.firstinspires.ftc.teamcode.drive.localizer.toFtcLib
import org.opencv.core.Mat
import org.openftc.easyopencv.*

@Autonomous
//@Disabled
class DepthTest: LinearOpMode() {
    lateinit var webcam: OpenCvWebcam
    override fun runOpMode() {

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
        webcam.setPipeline(DepthPipeline(hardwareMap))

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

        FtcDashboard.getInstance().startCameraStream(webcam, 10.0)
        if(isStopRequested){
            T265Localizer.slamera!!.stop()
        }

        waitForStart()
        while (opModeIsActive()) { }
        T265Localizer.slamera!!.stop()
        if(isStopRequested){
            T265Localizer.slamera!!.stop()
        }
    }
}

class DepthPipeline(val hardwareMap: HardwareMap): OpenCvPipeline() {
    init {
        Log.d("Depth", "Started Init")
        T265Localizer.slamera = T265Camera(
                Transform2d(
                    T265Localizer.cameraRobotOffset.toFtcLib().translation,
                    T265Localizer.cameraRobotOffset.toFtcLib().rotation
                ), 0.8, hardwareMap.appContext
                )
        Log.d("Depth", "Created Cam")
        T265Localizer.slamera!!.start()
        Log.d("Depth", "Started Cam")
    }
    override fun processFrame(input: Mat?): Mat {
        Log.d("Depth", "Starting Depth Calc")
        T265Localizer.slamera!!.getDepthMat()
        return input!! //T265Localizer.slamra!!.depthMat
    }

}