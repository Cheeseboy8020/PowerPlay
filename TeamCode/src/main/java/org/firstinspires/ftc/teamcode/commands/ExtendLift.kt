package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandBase
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.Lift
import org.firstinspires.ftc.teamcode.util.PositionPIDFController

@Config
class ExtendLift(val lift: Lift, val goal: Lift.Positions): CommandBase(){

    var liftController: PositionPIDFController = PositionPIDFController(lift)
    val time = ElapsedTime()
    var lastTime = 0.0
    var lastVel = 0.0

    init{
        addRequirements(lift)
    }

    override fun initialize() {
        //once
        time.reset()
        liftController.PROFILED_PID.reset(lift.leftLift.currentPosition.toDouble(), lift.leftLift.velocity)
        liftController.targetPos = goal.targetPosition.toDouble()
    }

    //Run repeatedly while the command is active
    override fun execute() {
        val targetAccel = (liftController.PROFILED_PID.setpoint.velocity - lift.leftLift.velocity) / (time.seconds() - lastTime)
        lift.setPower(liftController.update(lift.leftLift.currentPosition.toDouble(), targetAccel))
        lastVel = liftController.PROFILED_PID.setpoint.velocity
        lastTime = time.seconds()
    }

    override fun isFinished(): Boolean {
        //End if the lift position is within the tolerance
        return liftController.PROFILED_PID.atGoal()
    }

    override fun end(interrupted: Boolean) {
        if(goal.targetPosition>0){
            lift.setPower(PositionPIDFController.kg)
        }
        else{
            lift.setPower(0.0)
        }
    }
}