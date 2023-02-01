package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension
import org.firstinspires.ftc.teamcode.util.ExtPositionPIDFController
import org.firstinspires.ftc.teamcode.util.OpModeType
import org.firstinspires.ftc.teamcode.util.PositionPIDFController


@Config
@TeleOp
class PosExtTuner : LinearOpMode() {
    private val dashboard = FtcDashboard.getInstance()
    private val veloTimer = ElapsedTime()
    lateinit var intakeExtension: IntakeExtension

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        intakeExtension = IntakeExtension(hardwareMap, telemetry, OpModeType.AUTO)
        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }
        val posController = ExtPositionPIDFController()
        val tuningController = PosExtTuningController()
        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)
        telemetry.addLine("Ready!")
        telemetry.update()
        telemetry.clearAll()
        waitForStart()
        if (isStopRequested) return
        tuningController.start()
        veloTimer.reset()
        while (!isStopRequested && opModeIsActive()) {
            posController.updateCoeffs()
            posController.targetPos = tuningController.update()
            veloTimer.reset()
            telemetry.addData("targetPosition", posController.targetPos)
            telemetry.addData("targetVelocity", posController.targetVelo)
            val motorPos = intakeExtension.position
            val power: Double = posController.update(intakeExtension.position.toDouble())
            intakeExtension.power = power
            telemetry.addData("position", motorPos)
            telemetry.addData("error", posController.targetPos - motorPos)
            telemetry.addData(
                "upperBound",
                PosTuningController.TESTING_MAX_POS*1.15)
            telemetry.addData("lowerBound", PosTuningController.TESTING_MIN_POS)
            telemetry.update()
        }
    }
}