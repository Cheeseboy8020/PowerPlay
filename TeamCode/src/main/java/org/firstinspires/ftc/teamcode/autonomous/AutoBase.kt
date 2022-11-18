package org.firstinspires.ftc.teamcode.autonomous

import com.arcrobotics.ftclib.command.CommandOpMode
import org.firstinspires.ftc.robotcore.external.Telemetry

abstract class AutoBase : CommandOpMode() {

    override fun initialize() {

        //Clear the bulk read cache every iteration
        schedule(BulkCacheCommand(hardwareMap))
    }

    companion object {
        fun Telemetry.sendData(tag: String, data: Any) {
            addData(tag, data)
            update()
        }

        fun Telemetry.sendLine(message: String) {
            addLine(message)
            update()
        }
    }

}