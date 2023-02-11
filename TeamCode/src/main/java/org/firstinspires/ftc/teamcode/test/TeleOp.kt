package org.firstinspires.ftc.teamcode.test

import android.util.Log
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.*
import com.arcrobotics.ftclib.command.button.Trigger
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.autonomous.AutoBase.Companion.sendLine
import org.firstinspires.ftc.teamcode.autonomous.PoseStorage.pose
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.subsystems.*
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension.Companion.LEFT_IN
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension.Companion.RIGHT_IN
import org.firstinspires.ftc.teamcode.subsystems.LiftTurret.Companion.TURRET_RESET
import org.firstinspires.ftc.teamcode.util.OpModeType


@TeleOp
@Config
class TeleOp: CommandOpMode() {
    lateinit var drive: MecanumDrive
    lateinit var lift: Lift
    lateinit var intake: IntakeExtension
    lateinit var intakeArm: IntakeArm
    lateinit var liftArm: LiftArm
    lateinit var turret: LiftTurret

    companion object{
        @JvmField var extPos = 0.59
        @JvmField var turretAngle = 0.025
        @JvmField var turretAngleMid = 0.52
        @JvmField var armOut = 0.56
    }
    override fun initialize() {
        var dashboard = FtcDashboard.getInstance()
        telemetry = MultipleTelemetry(telemetry, dashboard.telemetry)
        val gamepad1 = GamepadEx(gamepad1)
        val gamepad2 = GamepadEx(gamepad2)


        lift = Lift(hardwareMap, Lift.Positions.IN_ROBOT, OpModeType.TELEOP)
        drive = MecanumDrive(hardwareMap)
        drive.poseEstimate = pose
        intake = IntakeExtension(hardwareMap, telemetry, OpModeType.TELEOP)
        intakeArm = IntakeArm(hardwareMap, telemetry, OpModeType.TELEOP)
        liftArm = LiftArm(hardwareMap, OpModeType.TELEOP)
        turret = LiftTurret(hardwareMap, OpModeType.TELEOP)


        val scoreCone = ScoreCone(intakeArm, liftArm, intake, lift, 1, extPos, Lift.Positions.HIGH, turret, TURRET_RESET)
        val scoreConeAngle = ScoreCone(intakeArm, liftArm, intake, lift, 1, extPos, Lift.Positions.HIGH_ANGLE, turret, turretAngle, armOut)
        val scoreConeMid = ScoreCone(intakeArm, liftArm, intake, lift, 1, extPos, Lift.Positions.MEDIUM, turret, turretAngleMid, armOut)
        val autoPos = AutoPosition(drive, gamepad1)
        Log.w("TeleOp", "Initialized Lift")
        Log.d("TeleOp", pose.x.toString())
        Log.d("TeleOp", pose.y.toString())
        Log.d("TeleOp", pose.heading.toString())
        telemetry.sendLine("Initialized Robot")
        //Auto position - Done
        //One button to flip arm and raise lift (lowers lift also) - Done
        //Lowers arm and lowers lift - Done
        //Full retract extending - Done
        //Tranfer retract - Done
        //Intake pinch/unpinch
        //Intake arm/bring to transfer position

        //Intake Arm out
        //Emergency stop - Done
        gamepad1.getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(autoPos)

        gamepad1.getGamepadButton(GamepadKeys.Button.Y)
            .cancelWhenPressed(autoPos)
            .whenPressed(InstantCommand({Log.d("TeleOp", "Emergency Stop")}))
            .whenPressed(ScheduleCommand(GamepadDrive(drive, { gamepad1.leftY }, { gamepad1.leftX }, { gamepad1.rightX })))

        gamepad2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
            .whenPressed(LiftGoToPos(lift, when(lift.currentPosition){
                Lift.Positions.HIGH->Lift.Positions.MEDIUM
                Lift.Positions.MEDIUM -> Lift.Positions.LOW
                Lift.Positions.LOW -> Lift.Positions.IN_ROBOT
                else -> {Lift.Positions.HIGH}
            }))
            .whenPressed(RaiseLiftArm(liftArm))
            .whenPressed(ResetTurret(turret))
        gamepad2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .whenPressed(LiftGoToPos(lift, Lift.Positions.IN_ROBOT))
            .whenPressed(LowerLiftArm(liftArm))
        gamepad2.getGamepadButton(GamepadKeys.Button.B)
            .whenPressed(scoreConeMid)
        gamepad2.getGamepadButton(GamepadKeys.Button.X)
            .whenPressed(scoreConeAngle)
        gamepad2.getGamepadButton(GamepadKeys.Button.BACK)
            .cancelWhenPressed(scoreCone)
            .cancelWhenPressed(scoreConeAngle)
            .cancelWhenPressed(scoreConeMid)
        gamepad2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
            .whenPressed(SequentialCommandGroup(CloseIntakePinch(intakeArm), RaiseIntakeArm(intakeArm)))
        gamepad2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
            .whenPressed(SequentialCommandGroup(OpenIntakePinch(intakeArm), ParallelCommandGroup(InstantCommand({liftArm.arm.position=0.5}), SequentialCommandGroup(WaitCommand(250), LiftTurretPosition(turret, 0.0)))))
        gamepad2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
            .whenPressed(SequentialCommandGroup(LowerIntakeArm(intakeArm, 1), OpenIntakePinch(intakeArm)))
        Trigger{gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.0 || gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.0}
            .whileActiveContinuous(InstantCommand({lift.setPower((gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)-gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER))/2.0)}, lift))
            .whenInactive(InstantCommand({lift.setPower(0.0)}, lift))

        gamepad2.getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(InstantCommand({LEFT_IN -= 0.01}))
            .whenPressed(InstantCommand({RIGHT_IN += 0.01}))
            .whenPressed(InstantCommand({intake.extLeft.position  = intake.extLeft.position-0.01}))
            .whenPressed(InstantCommand({intake.extRight.position  = intake.extRight.position+0.01}))

        gamepad2.getGamepadButton(GamepadKeys.Button.Y)
            .whenPressed(InstantCommand({LEFT_IN += 0.01}))
            .whenPressed(InstantCommand({RIGHT_IN -= 0.01}))
            .whenPressed(InstantCommand({intake.extLeft.position  = intake.extLeft.position+0.01}))
            .whenPressed(InstantCommand({intake.extRight.position  = intake.extRight.position-0.01}))

        gamepad2.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
            .whenPressed(InstantCommand({intake.retractFull()}))
        gamepad2.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
            .whenPressed(InstantCommand({intake.retract()}))


        schedule(GamepadDrive(drive, { gamepad1.leftY }, { gamepad1.leftX }, { gamepad1.rightX }))
        register(lift, intake, intakeArm, liftArm, drive)
    }

    override fun run() {
        super.run()

        telemetry.addData("x", drive.poseEstimate.x)
        telemetry.addData("y", drive.poseEstimate.y)
        telemetry.addData("heading", drive.poseEstimate.heading)
        telemetry.addData("Lift Position", lift.position)
        telemetry.addData("Intake Position", intakeArm.arm.position)
        telemetry.addData("Lift Arm Position", liftArm.arm.position)
        telemetry.addData("Ext Position", intake.extLeft.position)
        telemetry.update()
    }
}