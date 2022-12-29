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
import org.firstinspires.ftc.teamcode.util.PositionPIDFController

@Config
@TeleOp
class LiftTuner : LinearOpMode() {
    private val dashboard = FtcDashboard.getInstance()
    private val posTimer = ElapsedTime()

    override fun runOpMode() {
        // Change my id
        val myMotor1 = hardwareMap.get(DcMotorEx::class.java, "leftLift")
        val myMotor2 = hardwareMap.get(DcMotorEx::class.java, "rightLift")

        // Reverse as appropriate
        // myMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        // myMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        myMotor1.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        myMotor2.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }
        var posController = PositionPIDFController(MOTOR_POS_PID, kV, kA, kStatic)
        val tuningController = TuningController()
        var lastTargetPos = 0.0
        var lastKv = kV
        var lastKa = kA
        var lastKstatic = kStatic
        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)
        telemetry.addLine("Ready!")
        telemetry.update()
        telemetry.clearAll()
        waitForStart()
        if (isStopRequested) return
        tuningController.start()
        posTimer.reset()
        while (!isStopRequested && opModeIsActive()) {
            val targetPos: Double = tuningController.update()
            posController.controller!!.targetPosition = targetPos
            posController.controller!!.targetVelocity = (targetPos - lastTargetPos) / posTimer.seconds()
            posTimer.reset()
            lastTargetPos = targetPos
            telemetry.addData("targetPosition", targetPos)
            val motorPos = myMotor1.currentPosition.toDouble()
            val motorVelo = myMotor1.velocity
            val power: Double = posController.update(motorPos, motorVelo)
            myMotor1.power = power
            myMotor2.power = power
            if (lastKv != kV || lastKa != kA || lastKstatic != kStatic) {
                lastKv = kV
                lastKa = kA
                lastKstatic = kStatic
                posController = PositionPIDFController(MOTOR_POS_PID, kV, kA, kStatic)
            }
            telemetry.addData("position", motorPos)
            telemetry.addData("error", targetPos - motorPos)
            telemetry.addData(
                "upperBound",
                TuningController.rpmToTicksPerSecond(TuningController.TESTING_MAX_POS * 1.15)
            )
            telemetry.addData("lowerBound", 0)
            telemetry.update()
        }
    }

    companion object {
        @JvmStatic
        var MOTOR_POS_PID = PIDCoefficients(0.0, 0.0, 0.0)
        @JvmStatic
        var kV: Double = 1 / TuningController.rpmToTicksPerSecond(TuningController.MOTOR_MAX_POS)
        @JvmStatic
        var kA = 0.0
        @JvmStatic
        var kStatic = 0.0
    }
}