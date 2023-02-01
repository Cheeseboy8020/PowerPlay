package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension
import org.firstinspires.ftc.teamcode.util.ExtPositionPIDFController
import org.firstinspires.ftc.teamcode.util.PositionPIDFController
import kotlin.math.abs

@Config
class ExtendIntakeProfile(val intakeExtension: IntakeExtension, val goal: Double): CommandBase(){

    var intakeExtensionController = ExtPositionPIDFController()
    val time = ElapsedTime()
    var lastTime = 0.0
    var lastVel = 0.0

    init{
        addRequirements(intakeExtension)
    }

    override fun initialize() {
        //once
        time.reset()
        intakeExtensionController.PROFILED_PID.reset(intakeExtension.position)
        intakeExtensionController.targetPos = goal
    }

    //Run repeatedly while the command is active
    override fun execute() {
        intakeExtension.power = intakeExtensionController.update(intakeExtension.position)
        lastTime = time.seconds()
    }

    override fun isFinished(): Boolean {
        //End if the intakeExtension position is within the tolerance
        return abs(goal - intakeExtension.position) < 10
    }
}