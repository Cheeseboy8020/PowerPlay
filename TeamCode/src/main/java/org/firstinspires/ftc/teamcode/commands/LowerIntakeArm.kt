package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.IntakeArm
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension

class LowerIntakeArm(private var intake: IntakeArm, var stackHeight: Int) : CommandBase() {
    init {
        addRequirements(intake)
    }

    override fun initialize() {
        when(stackHeight) {
            1->intake.arm.position = 0.54 //1 stack
            2->intake.arm.position = 0.5366//2 stack
            3->intake.arm.position = 0.5333 //3 stack
            4->intake.arm.position = 0.53 //4 stack
            5->intake.arm.position = 0.52 //5 stack
        }
    }

    override fun isFinished(): Boolean {
        return true
    }
}