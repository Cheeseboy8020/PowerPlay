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
import org.firstinspires.ftc.teamcode.test.VeloTuningController.Companion.MOTOR_MAX_RPM
import org.firstinspires.ftc.teamcode.test.VeloTuningController.Companion.TESTING_MAX_SPEED
import org.firstinspires.ftc.teamcode.util.VelocityPIDFController

@Config
@TeleOp
class VeloLiftTuner : LinearOpMode() {
    private val dashboard = FtcDashboard.getInstance()
    private val veloTimer = ElapsedTime()

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
        var veloController = VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic)
        val tuningController = VeloTuningController()
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
        tuningController.start()
        veloTimer.reset()
        while (!isStopRequested && opModeIsActive()) {
            val targetVelo: Double = tuningController.update()
            veloController.setTargetVelocity(targetVelo)
            veloController.setTargetAcceleration((targetVelo - lastTargetVelo) / veloTimer.seconds())
            veloTimer.reset()
            lastTargetVelo = targetVelo
            telemetry.addData("targetVelocity", targetVelo)
            val motorPos = myMotor1.currentPosition.toDouble()
            val motorVelo = myMotor1.velocity
            val power = veloController.update(motorPos, motorVelo)
            myMotor1.power = power
            myMotor2.power = power
            if (lastKv != kV || lastKa != kA || lastKstatic != kStatic) {
                lastKv = kV
                lastKa = kA
                lastKstatic = kStatic
                veloController = VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic)
            }
            telemetry.addData("velocity", motorVelo)
            telemetry.addData("error", targetVelo - motorVelo)
            telemetry.addData(
                "upperBound",
                rpmToTicksPerSecond(TESTING_MAX_SPEED * 1.15)
            )
            telemetry.addData("lowerBound", 0)
            telemetry.update()
        }
    }

    companion object {
        var MOTOR_VELO_PID = PIDCoefficients(0.0, 0.0, 0.0)
        var kV: Double = 1 / rpmToTicksPerSecond(MOTOR_MAX_RPM)
        var kA = 0.0
        var kStatic = 0.0
    }
}