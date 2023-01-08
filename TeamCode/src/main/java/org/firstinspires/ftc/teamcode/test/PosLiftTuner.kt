package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.Lift
import org.firstinspires.ftc.teamcode.util.OpModeType
import org.firstinspires.ftc.teamcode.util.PositionPIDFController


@Config
@TeleOp
class PosLiftTuner : LinearOpMode() {
    private val dashboard = FtcDashboard.getInstance()
    private val veloTimer = ElapsedTime()
    lateinit var lift: Lift
    lateinit var batteryVoltageSensor: VoltageSensor

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        lift = Lift(hardwareMap, Lift.Positions.IN_ROBOT, OpModeType.AUTO)
        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }
        var posController = PositionPIDFController(lift)
        val tuningController = PosTuningController()
        var lastTargetVelo = 0.0
        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)
        telemetry.addLine("Ready!")
        telemetry.update()
        telemetry.clearAll()
        waitForStart()
        if (isStopRequested) return
        tuningController.start()
        veloTimer.reset()
        while (!isStopRequested && opModeIsActive()) {
            val targetPos: Double = tuningController.update()
            val targetAccel = (posController.targetVelo - lastTargetVelo) / veloTimer.seconds()
            veloTimer.reset()
            lastTargetVelo = posController.targetVelo
            telemetry.addData("targetPosition", targetPos)
            telemetry.addData("targetVelocity", posController.targetVelo)
            val motorPos = lift.leftLift.currentPosition
            val motorVelo = lift.leftLift.velocity
            val power: Double = posController.update(lift.leftLift.currentPosition.toDouble(), targetAccel)
            lift.setPower(power)
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