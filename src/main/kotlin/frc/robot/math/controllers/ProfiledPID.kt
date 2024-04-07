package frc.robot.util

import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.util.sendable.Sendable
import edu.wpi.first.util.sendable.SendableBuilder

class ProfiledPID(val p: Double, val i: Double, val d: Double, var trapConstraints: TrapezoidProfile.Constraints) :
    Sendable {
    val pidController = PIDController(p, i, d)
    val trapezoidProfile = TrapezoidProfile(trapConstraints)
    var desiredTrapState = TrapezoidProfile.State()
    var currentTrapState = TrapezoidProfile.State()
    var goal = 0.0
        set(value) {
            desiredTrapState = TrapezoidProfile.State(value, 0.0)
            currentTrapState = TrapezoidProfile.State(value, 0.0)
            field = value
        }

    fun calculate(measured: Double): Double {
        currentTrapState = trapezoidProfile.calculate(pidController.period, currentTrapState, desiredTrapState)

        return pidController.calculate(measured, currentTrapState.position)
    }

    override fun initSendable(builder: SendableBuilder) {
        builder.setSmartDashboardType("ProfiledPID")
        builder.addDoubleProperty("p", pidController::getP, pidController::setP)
        builder.addDoubleProperty("i", pidController::getI, pidController::setI)
        builder.addDoubleProperty("d", pidController::getD, pidController::setD)
        builder.addDoubleProperty(
            "Acceleration",
            { trapConstraints.maxAcceleration },
            { trapConstraints = TrapezoidProfile.Constraints(trapConstraints.maxVelocity, it) })
        builder.addDoubleProperty(
            "Velocity",
            { trapConstraints.maxVelocity },
            { trapConstraints = TrapezoidProfile.Constraints(it, trapConstraints.maxAcceleration) })
        builder.addDoubleProperty("goal", { goal }, { goal = it })
    }
}
