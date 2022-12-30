package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.dashboard.config.Config
import com.noahbres.jotai.StateMachine
import com.noahbres.jotai.StateMachineBuilder
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.subsystems.Lift.Companion.rpmToTicksPerSecond

@Config
class VeloTuningController {
    internal enum class State {
        RAMPING_UP, COASTING_1, RAMPING_DOWN, COASTING_2, RANDOM_1, RANDOM_2, RANDOM_3, REST
    }

    private val stateMachine: StateMachine<*>
    private val externalTimer = ElapsedTime()
    private var currentTargetVelo = 0.0

    init {
        stateMachine = StateMachineBuilder<State>()
            .state(State.RAMPING_UP)
            .transitionTimed(ZSTATE1_RAMPING_UP_DURATION)
            .onEnter { externalTimer.reset() }
            .loop {
                val progress =
                    externalTimer.seconds() / ZSTATE1_RAMPING_UP_DURATION
                val target =
                    progress * (TESTING_MAX_SPEED - TESTING_MIN_SPEED) + TESTING_MIN_SPEED
                currentTargetVelo = rpmToTicksPerSecond(target)
            }
            .state(State.COASTING_1)
            .transitionTimed(ZSTATE2_COASTING_1_DURATION)
            .onEnter {
                currentTargetVelo =
                    rpmToTicksPerSecond(TESTING_MAX_SPEED)
            }
            .state(State.RAMPING_DOWN)
            .transitionTimed(ZSTATE3_RAMPING_DOWN_DURATION)
            .onEnter { externalTimer.reset() }
            .loop {
                val progress =
                    externalTimer.seconds() / ZSTATE3_RAMPING_DOWN_DURATION
                val target =
                    -(TESTING_MAX_SPEED - progress * (TESTING_MAX_SPEED - TESTING_MIN_SPEED))
                currentTargetVelo = rpmToTicksPerSecond(target)
            }
            .state(State.COASTING_2)
            .transitionTimed(ZSTATE4_COASTING_2_DURATION)
            .onEnter {
                currentTargetVelo =
                    rpmToTicksPerSecond(TESTING_MIN_SPEED)
            }
            .state(State.RANDOM_1)
            .transitionTimed(ZSTATE5_RANDOM_1_DURATION)
            .onEnter {
                currentTargetVelo =
                    rpmToTicksPerSecond(Math.random() * (TESTING_MAX_SPEED - TESTING_MIN_SPEED) + TESTING_MIN_SPEED)
            }
            .state(State.RANDOM_2)
            .transitionTimed(ZSTATE6_RANDOM_2_DURATION)
            .onEnter {
                currentTargetVelo =
                    rpmToTicksPerSecond(Math.random() * (TESTING_MAX_SPEED - TESTING_MIN_SPEED) + TESTING_MIN_SPEED)
            }
            .state(State.RANDOM_3)
            .transitionTimed(ZSTATE7_RANDOM_3_DURATION)
            .onEnter {
                currentTargetVelo =
                    rpmToTicksPerSecond(Math.random() * (TESTING_MAX_SPEED - TESTING_MIN_SPEED) + TESTING_MIN_SPEED)
            }
            .state(State.REST)
            .transitionTimed(ZSTATE8_REST_DURATION)
            .onEnter { currentTargetVelo = 0.0 }
            .exit(State.RAMPING_UP)
            .build()
        stateMachine.looping = true
    }

    fun start() {
        externalTimer.reset()
        stateMachine.start()
    }

    fun update(): Double {
        stateMachine.update()
        return currentTargetVelo
    }

    companion object {
        @JvmField var MOTOR_MAX_RPM = 5960/((68.0/13)*(84.0/29)*(84.0/29))
        @JvmField var TESTING_MAX_SPEED = 0.9 * MOTOR_MAX_RPM
        @JvmField var TESTING_MIN_SPEED = 0.3 * TESTING_MAX_SPEED

        // These are prefixed with "STATE1", "STATE2", etc. because Dashboard displays variables in
        // alphabetical order. Thus, we preserve the actual order of the process
        // Then we append Z just because we want it to show below the MOTOR_ and TESTING_ because
        // these settings aren't as important
        @JvmField var ZSTATE1_RAMPING_UP_DURATION = 1.0
        @JvmField var ZSTATE2_COASTING_1_DURATION = 2.0
        @JvmField var ZSTATE3_RAMPING_DOWN_DURATION = 1.0
        @JvmField var ZSTATE4_COASTING_2_DURATION = 2.0
        @JvmField var ZSTATE5_RANDOM_1_DURATION = 2.0
        @JvmField var ZSTATE6_RANDOM_2_DURATION = 2.0
        @JvmField var ZSTATE7_RANDOM_3_DURATION = 2.0
        @JvmField var ZSTATE8_REST_DURATION = 1.0
    }
}