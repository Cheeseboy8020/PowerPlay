package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.util.PositionPIDFController

@Config
@TeleOp
class LinkedMotorTuner : LinearOpMode() {
    private val dashboard = FtcDashboard.getInstance()
    private val posTimer = ElapsedTime()

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        // Change my id
        val myMotor1 = hardwareMap.get(DcMotorEx::class.java, "lift")
        //val myMotor2 = hardwareMap.get(DcMotorEx::class.java, "flywheelMotor2")

        // Reverse as appropriate
        myMotor1.direction = DcMotorSimple.Direction.REVERSE;
        // myMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        myMotor1.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        //myMotor2.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }
        var posController = PositionPIDFController()
        val tuningController = PosTuningController()
        var lastTargetPos = 0.0
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
            posController.targetPos = targetPos
            posController.targetVelo = (targetPos - lastTargetPos) / posTimer.seconds()
            posTimer.reset()
            lastTargetPos = targetPos
            telemetry.addData("targetPosition", targetPos)
            telemetry.addData("targetVelocity", posController.targetVelo)
            val motorPos = myMotor1.currentPosition.toDouble()
            val motorVelo = myMotor1.velocity
            val power: Double = posController.update(motorPos, motorVelo)
            myMotor1.velocity = power
            //myMotor2.power = power=
            telemetry.addData("position", motorPos)
            telemetry.addData("velocity", motorVelo)
            telemetry.addData("error", targetPos - motorPos)
            telemetry.addData(
                "upperBound",
                PosTuningController.TESTING_MAX_POS*1.15)
            telemetry.addData("lowerBound", PosTuningController.TESTING_MIN_POS)
            telemetry.update()
        }
    }
}