package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.dashboard.config.Config
import com.noahbres.jotai.StateMachine
import com.noahbres.jotai.StateMachineBuilder
import com.qualcomm.robotcore.util.ElapsedTime

@Config
class PosTuningController {
    internal enum class State {
        LIFTING_UP, HOLDING_1, LOWERING_DOWN, HOLDING_2, RANDOM_1, RANDOM_2, RANDOM_3, REST
    }

    private val stateMachine: StateMachine<State>
    private val externalTimer = ElapsedTime()
    private var currentTargetPos = 0.0

    init {
        stateMachine = StateMachineBuilder<State>()
            .state(State.LIFTING_UP)
            .transitionTimed(ZSTATE1_RAMPING_UP_DURATION)
            .onEnter { externalTimer.reset() }
            .loop {
                val progress =
                    externalTimer.seconds() / ZSTATE1_RAMPING_UP_DURATION
                val target =
                    progress * (TESTING_MAX_POS - TESTING_MIN_POS) + TESTING_MIN_POS
                currentTargetPos = target
            }
            .state(State.HOLDING_1)
            .transitionTimed(ZSTATE2_COASTING_1_DURATION)
            .onEnter {
                currentTargetPos = TESTING_MAX_POS
            }
            .state(State.LOWERING_DOWN)
            .transitionTimed(ZSTATE3_RAMPING_DOWN_DURATION)
            .onEnter { externalTimer.reset() }
            .loop {
                val progress =
                    externalTimer.seconds() / ZSTATE3_RAMPING_DOWN_DURATION
                val target =
                    TESTING_MAX_POS - progress * (TESTING_MAX_POS - TESTING_MIN_POS)
                currentTargetPos = target
            }
            .state(State.HOLDING_2)
            .transitionTimed(ZSTATE4_COASTING_2_DURATION)
            .onEnter {
                currentTargetPos = TESTING_MIN_POS
            }
            .state(State.RANDOM_1)
            .transitionTimed(ZSTATE5_RANDOM_1_DURATION)
            .onEnter {
                currentTargetPos = Math.random() * (TESTING_MAX_POS - TESTING_MIN_POS) + TESTING_MIN_POS
            }
            .state(State.RANDOM_2)
            .transitionTimed(ZSTATE6_RANDOM_2_DURATION)
            .onEnter {
                currentTargetPos = Math.random() * (TESTING_MAX_POS - TESTING_MIN_POS) + TESTING_MIN_POS
            }
            .state(State.RANDOM_3)
            .transitionTimed(ZSTATE7_RANDOM_3_DURATION)
            .onEnter {
                currentTargetPos = Math.random() * (TESTING_MAX_POS - TESTING_MIN_POS) + TESTING_MIN_POS
            }
            .state(State.REST)
            .transitionTimed(ZSTATE8_REST_DURATION)
            .onEnter { currentTargetPos = 0.0 }
            .exit(State.LIFTING_UP)
            .build()
        stateMachine.looping = true
    }

    fun start() {
        externalTimer.reset()
        stateMachine.start()
    }

    fun update(): Double {
        stateMachine.update()
        return currentTargetPos
    }

    companion object {
        @JvmField var MOTOR_MAX_POS = 1330 //TODO: Figure this out by running the motors to the top of the lift
        @JvmField  var TESTING_MAX_POS = 0.8 * MOTOR_MAX_POS
        @JvmField var TESTING_MIN_POS = 200.0

        // These are prefixed with "STATE1", "STATE2", etc. because Dashboard displays variables in
        // alphabetical order. Thus, we preserve the actual order of the process
        // Then we append Z just because we want it to show below the MOTOR_ and TESTING_ because
        // these settings aren't as important
        @JvmField var ZSTATE1_RAMPING_UP_DURATION = 3.5
        @JvmField var ZSTATE2_COASTING_1_DURATION = 4.0
        @JvmField var ZSTATE3_RAMPING_DOWN_DURATION = 2.0
        @JvmField var ZSTATE4_COASTING_2_DURATION = 2.0
        @JvmField var ZSTATE5_RANDOM_1_DURATION = 2.0
        @JvmField var ZSTATE6_RANDOM_2_DURATION = 2.0
        @JvmField var ZSTATE7_RANDOM_3_DURATION = 2.0
        @JvmField var ZSTATE8_REST_DURATION = 1.0
    }
}