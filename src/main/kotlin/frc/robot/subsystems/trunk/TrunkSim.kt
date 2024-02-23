package frc.robot.subsystems.trunk

import edu.wpi.first.wpilibj.Timer
import java.lang.Math.pow

class TrunkSim : TrunkIO {
    var desiredTrunkRotation = 0.0
    var lastTrunkRotation = 0.0
    var trunkRotationTimer = Timer()

    var trunkRotation = 0.0

    var desiredTraversalPercentage = 0.0
    var lastTraversalPercentage = 0.0
    var traversalPercentageTimer = Timer()
    var traversalPercentage = 0.0

    override fun getPosition(): Double = traversalPercentage

    override fun getRotation(): Double = trunkRotation

    override fun setDesiredPosition(position: Double) {
        traversalPercentageTimer.reset()
        lastTraversalPercentage = desiredTraversalPercentage
        desiredTraversalPercentage = position
        traversalPercentageTimer.start()
    }

    override fun setDesiredRotation(angle: Double) {
        trunkRotationTimer.reset()
        lastTrunkRotation = desiredTrunkRotation
        desiredTrunkRotation = angle
        trunkRotationTimer.start()
    }

    override fun setShootSpeed(left: Double, right: Double) {}

    private fun easeInOutCubic(x: Double): Double {
        return if (x < 0.5) {
            4 * x * x * x
        } else {
            1 - pow(-2 * x + 2, 3.0) / 2
        }
    }

    override fun periodic() {
        if (trunkRotationTimer.get() < 1) {
            trunkRotation = (desiredTrunkRotation - lastTrunkRotation) * easeInOutCubic(trunkRotationTimer.get()) + lastTrunkRotation
        } else {
            trunkRotation = desiredTrunkRotation
        }

        if (traversalPercentageTimer.get() < 1) {
            traversalPercentage = (desiredTraversalPercentage - lastTraversalPercentage) * easeInOutCubic(traversalPercentageTimer.get()) + lastTraversalPercentage
        } else {
            traversalPercentage = desiredTraversalPercentage
        }
    }
}