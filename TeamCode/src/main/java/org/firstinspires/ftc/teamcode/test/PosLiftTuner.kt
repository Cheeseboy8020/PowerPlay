package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.Lift.Companion.rpmToTicksPerSecond
import org.firstinspires.ftc.teamcode.util.PositionPIDFController
import org.firstinspires.ftc.teamcode.util.VelocityPIDFController

@Config
@TeleOp
class PosLiftTuner : LinearOpMode() {
    private val dashboard = FtcDashboard.getInstance()
    private val posTimer = ElapsedTime()


    override fun runOpMode() {



        // Change my id
        val myMotor1 = hardwareMap.get(DcMotorEx::class.java, "lift")
        val myMotor2 = hardwareMap.get(DcMotorEx::class.java, "lift2")

        // Reverse as appropriate
        // myMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        // myMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        myMotor1.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        myMotor2.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }
        var posController = PositionPIDFController(MOTOR_POS_PID, kV, kA, kStatic)
        val posTuningController = PosTuningController()
        posController.controller!!.setOutputBounds(rpmToTicksPerSecond(-5960/((68.0/13)*(84.0/29)*(84.0/29))), rpmToTicksPerSecond(
            5960/((68.0/13)*(84.0/29)*(84.0/29))))
        var lastTargetPos = 0.0
        var lastTargetVelo = 0.0
        var lastKv = kV
        var lastKa = kA
        var lastKstatic = kStatic
        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)
        telemetry.addLine("Ready!")
        telemetry.update()
        telemetry.clearAll()
        waitForStart()
        if (isStopRequested) return
        posTuningController.start()
        posTimer.reset()
        while (!isStopRequested && opModeIsActive()) {
            val targetPos: Double = posTuningController.update()
            posController.controller!!.targetPosition = targetPos
            posController.controller!!.targetVelocity = (targetPos - lastTargetPos) / posTimer.seconds()
            lastTargetPos = targetPos
            telemetry.addData("targetPosition", targetPos)
            val motorPos = myMotor1.currentPosition.toDouble()
            val motorVelo = myMotor1.velocity
            val targetVelo: Double = posController.update(motorPos, motorVelo)
            telemetry.addData("velo", targetVelo)
            posTimer.reset()
            myMotor1.power = targetVelo
            myMotor2.power = targetVelo
            if (lastKv != kV || lastKa != kA || lastKstatic != kStatic) {
                lastKv = kV
                lastKa = kA
                lastKstatic = kStatic
                posController = PositionPIDFController(MOTOR_POS_PID, kV, kA, kStatic)
            }
            telemetry.addData("position", motorPos)
            telemetry.addData("error", targetPos - motorPos)
            telemetry.addData(
                "upperBound", PosTuningController.TESTING_MAX_POS * 1.15
            )
            telemetry.addData("lowerBound", 0)
            telemetry.update()
        }
    }

    companion object {
        @JvmField var MOTOR_POS_PID = PIDCoefficients(0.0, 0.0, 0.0)
        @JvmField var kV: Double = 1 / PosTuningController.MOTOR_MAX_POS
        @JvmField var kA = 0.0
        @JvmField var kStatic = 0.0
    }
}