package frc.robot.subsystems.trunk

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
import frc.robot.constants.TrunkConstants

class TrunkIOReal : TrunkIO {

    private val elevatorMotor = CANSparkMax(20, CANSparkLowLevel.MotorType.kBrushless)
    private val positionEncoder = elevatorMotor.getAlternateEncoder(8192)

    private val mainRotationMotor = CANSparkMax(22, CANSparkLowLevel.MotorType.kBrushless)
    private val followerRotationMotor = CANSparkMax(21, CANSparkLowLevel.MotorType.kBrushless)

    private val rotationEncoder = DutyCycleEncoder(TrunkConstants.rotationEncoderID)

    private val topLimit = DigitalInput(0)

    override var positionBrake = true
        set(enabled) {
            if (positionBrake != enabled)
                elevatorMotor.setIdleMode(if (enabled) CANSparkBase.IdleMode.kBrake else CANSparkBase.IdleMode.kCoast)
            field = enabled
        }
    override var rotationBrake = true
        set(enabled) {
            if (positionBrake != enabled) {
                mainRotationMotor.setIdleMode(if (enabled) CANSparkBase.IdleMode.kBrake else CANSparkBase.IdleMode.kCoast)
                followerRotationMotor.setIdleMode(if (enabled) CANSparkBase.IdleMode.kBrake else CANSparkBase.IdleMode.kCoast)
            }
            field = enabled
        }

    init {
        elevatorMotor.restoreFactoryDefaults()
        mainRotationMotor.restoreFactoryDefaults()
        followerRotationMotor.restoreFactoryDefaults()

        elevatorMotor.inverted = false // elevator likes to not be inverted idk why
        mainRotationMotor.inverted = false

        elevatorMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        mainRotationMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        followerRotationMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)

        followerRotationMotor.follow(mainRotationMotor, true)
    }

    override fun setZeroPosition() {
        positionEncoder.setPosition(TrunkConstants.TOP_BREAK_BEAM_POSITION * TrunkConstants.M2ELEVATOR)
    }

    override fun atTopLimit(): Boolean = topLimit.get()

    override fun getRawPosition(): Double = positionEncoder.position

    override fun getRawRotation(): Double = rotationEncoder.absolutePosition

    override fun setElevatorSpeed(speed: Double) {
        elevatorMotor.set(-speed)
    }

    override fun setRotationSpeed(speed: Double) {
        mainRotationMotor.set(speed)
    }

    override fun setRotationVoltage(volts: Double) {
        mainRotationMotor.setVoltage(volts);
    }

    override fun periodic() {
    }
}