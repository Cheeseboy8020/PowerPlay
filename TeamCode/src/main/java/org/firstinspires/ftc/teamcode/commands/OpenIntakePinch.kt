package org.firstinspires.ftc.teamcode.commands
import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.IntakeArm
import org.firstinspires.ftc.teamcode.subsystems.IntakeExtension

class OpenIntakePinch(private var intake: IntakeArm) : CommandBase() {
    val time = ElapsedTime()
    init {
        addRequirements(intake)
    }

    override fun initialize() {
        time.reset()
        intake.open()
    }

    override fun isFinished(): Boolean {
        return time.milliseconds() >= 250
    }
}