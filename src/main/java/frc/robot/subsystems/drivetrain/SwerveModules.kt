package frc.robot.subsystems.drivetrain

import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState

class SwerveModules(val frontLeft: SwerveModule, val frontRight: SwerveModule, val rearLeft: SwerveModule, val rearRight: SwerveModule) {
    private val modules: Array<SwerveModule>

    init {
        val modules = arrayOfNulls<SwerveModule?>(4)
        modules[Corner.FrontLeft.index] = frontLeft
        modules[Corner.FrontRight.index] = frontRight
        modules[Corner.RearLeft.index] = rearLeft
        modules[Corner.RearRight.index] = rearRight
        this.modules = modules as Array<SwerveModule>
    }

    fun update() {
        for (module in modules) {
            module.update()
        }
    }

    fun asArray(): Array<SwerveModule> {
        return modules
    }

    val states: States
        get() {
            return States(modules.map { it.state }.toTypedArray())
        }
    val positions: Positions
        get() {
            return Positions(modules.map { it.position }.toTypedArray())
        }

    fun resetEncoders() {
        for (module in modules) {
            module.resetEncoders()
        }
    }

    fun setDesiredStates(desiredStates: Array<SwerveModuleState>) {
        for (i in desiredStates.indices) {
            modules[i].setDesiredState(desiredStates[i])
        }
    }

    class States {
        val frontLeft: SwerveModuleState
        val frontRight: SwerveModuleState
        val rearLeft: SwerveModuleState
        val rearRight: SwerveModuleState
        private val states: Array<SwerveModuleState>

        constructor(states: Array<SwerveModuleState>) {
            this.states = states
            frontLeft = this.states[Corner.FrontLeft.index]
            frontRight = this.states[Corner.FrontRight.index]
            rearLeft = this.states[Corner.RearLeft.index]
            rearRight = this.states[Corner.RearRight.index]
        }

        constructor(frontLeft: SwerveModuleState, frontRight: SwerveModuleState, rearLeft: SwerveModuleState, rearRight: SwerveModuleState) {
            this.frontLeft = frontLeft
            this.frontRight = frontRight
            this.rearLeft = rearLeft
            this.rearRight = rearRight
            val states = arrayOfNulls<SwerveModuleState>(4)
            states[Corner.FrontLeft.index] = frontLeft
            states[Corner.FrontRight.index] = frontRight
            states[Corner.RearLeft.index] = rearLeft
            states[Corner.RearRight.index] = rearRight
            this.states = states as Array<SwerveModuleState>
        }

        fun asArray(): Array<SwerveModuleState> {
            return states
        }
    }

    class Positions(private val positions: Array<SwerveModulePosition>) {
        val frontLeft: SwerveModulePosition = positions[Corner.FrontLeft.index]
        val frontRight: SwerveModulePosition = positions[Corner.FrontRight.index]
        val rearLeft: SwerveModulePosition = positions[Corner.RearLeft.index]
        val rearRight: SwerveModulePosition = positions[Corner.RearRight.index]

        fun asArray(): Array<SwerveModulePosition> {
            return positions
        }
    }

    enum class Corner(val index: Int) {
        FrontLeft(0), FrontRight(1), RearLeft(2), RearRight(3);
    }
}