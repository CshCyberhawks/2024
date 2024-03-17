package frc.robot.subsystems.cannon

interface CannonIO {
    fun setLeftShooter(vel: Double)
    fun setRightShooter(vel: Double)
    fun getLeftShooterVel(): Double
    fun getRightShooterVel(): Double
    fun getLeftShooterTBVel(): Double
    fun getRightShooterTBVel(): Double
    fun setInnerIntakePercent(percent: Double)
    fun setOuterIntakePercent(percent: Double)
    fun getExitBeamBreak(): Boolean
    fun getEntryBeamBreak(): Boolean
    fun getLoadedBeamBreak(): Boolean
    fun getIntakePosition(): Double
    fun periodic()
}
