package frc.robot.subsystems.trunk

import com.revrobotics.CANSparkBase

interface TrunkIO {
    // TODO: I have no wifi, so I can't see how to actually advantagekit this
    var positionBrake: Boolean
    var rotationBrake: Boolean
    fun getRawPosition(): Double
    fun getRawRotation(): Double
    fun setElevatorSpeed(speed: Double)
    fun setRotationSpeed(speed: Double)
    fun setZeroPosition(top: Boolean)
    fun setRotationVoltage(volts: Double)
    fun atTopLimit(): Boolean
    fun periodic()
}