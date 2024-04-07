package frc.robot.subsystems.trunk

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.Servo
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.constants.TrunkConstants

class TrunkIOReal : TrunkIO {

    private val elevatorMotor = CANSparkMax(TrunkConstants.ELEVATOR_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless)

    //    private val positionEncoder = elevatorMotor.getAlternateEncoder(8192)
    private val positionEncoder = Encoder(1, 4)

    private val masterRotationMotor = TalonFX(TrunkConstants.MASTER_PIVOT_MOTOR_ID) // Right Motor
    private val followerRotationMotor = TalonFX(TrunkConstants.FOLLOWER_PIVOT_MOTOR_ID) // Left Motor

    private val shaftRotationEncoder = DutyCycleEncoder(TrunkConstants.rotationEncoderID)
    private val topLimit = DigitalInput(0)

    val climbServo = Servo(0)

    var positionEncoderOffset = 0.0

    override var positionBrake = true
        set(enabled) {
            if (positionBrake != enabled)
                elevatorMotor.setIdleMode(if (enabled) CANSparkBase.IdleMode.kBrake else CANSparkBase.IdleMode.kCoast)
            field = enabled
        }

    override var rotationBrake = true
        set(enabled) {
            if (field != enabled) {
                masterRotationMotor.setNeutralMode(if (enabled) NeutralModeValue.Brake else NeutralModeValue.Coast)
                followerRotationMotor.setNeutralMode(if (enabled) NeutralModeValue.Brake else NeutralModeValue.Coast)
            }
            field = enabled
        }

    init {
        // factory reset to make it not be bad
        elevatorMotor.restoreFactoryDefaults()
        val pivotMotorConfiguration =
            TalonFXConfiguration().withCurrentLimits(CurrentLimitsConfigs().withSupplyCurrentLimit(40.0))

//        pivotMotorConfiguration.Slot0.kP = TrunkConstants.rotationKP
//        pivotMotorConfiguration.Slot0.kI = TrunkConstants.rotationKI
//        pivotMotorConfiguration.Slot0.kD = TrunkConstants.rotationKD
//        pivotMotorConfiguration.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
//        pivotMotorConfiguration.Voltage.PeakForwardVoltage = 8.0;
//        pivotMotorConfiguration.Voltage.PeakReverseVoltage = -8.0;

        masterRotationMotor.configurator.apply(pivotMotorConfiguration)
        followerRotationMotor.configurator.apply(pivotMotorConfiguration)

        elevatorMotor.inverted = false // elevator likes to not be inverted idk why
        masterRotationMotor.inverted = false

        // ensure motors are initially braked
        elevatorMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        masterRotationMotor.setNeutralMode(NeutralModeValue.Brake)
        followerRotationMotor.setNeutralMode(NeutralModeValue.Brake)

        followerRotationMotor.setControl(Follower(22, true))

//        falconRotationOffset = (masterRotationMotor.position.value / 125.0) - shaftRotationEncoder.absolutePosition
    }

    override fun setFalconThroughBoreOffset() {
//        falconRotationOffset = (masterRotationMotor.position.value / 125.0) - shaftRotationEncoder.absolutePosition
    }

    private fun getEncoderRawPosition(): Double {
        return positionEncoder.get() / 1024.0
    }

    override fun setZeroPosition() {
//        positionEncoder.setPosition(TrunkConstants.TOP_BREAK_BEAM_POSITION * TrunkConstants.M2ELEVATOR)
        positionEncoderOffset =
            getEncoderRawPosition() - (TrunkConstants.TOP_BREAK_BEAM_POSITION * TrunkConstants.M2ELEVATOR)
        // val = raw - off
        // off = raw - val

//        positionEncoderOffset = TrunkConstants.TOP_BREAK_BEAM_POSITION * TrunkConstants.M2ELEVATOR - getRawPosition()
    }

    override fun atTopLimit(): Boolean = topLimit.get()

    override fun getPosition(): Double = getEncoderRawPosition() - positionEncoderOffset// - positionEncoderOffset

    private fun getRawPosition(): Double = getEncoderRawPosition() - positionEncoderOffset

    override fun getThroughBoreRawRotation(): Double = shaftRotationEncoder.absolutePosition

    override fun getFalconRawRotation(): Double = (masterRotationMotor.position.value / 125.0)

    override fun getPivotVelocity(): Double {
        return masterRotationMotor.velocity.value * 360.0 / 125.0
    }

    override fun setElevatorSpeed(speed: Double) {
        SmartDashboard.putNumber("set elevator speed: ", speed)
        elevatorMotor.set(-speed)
    }

    override fun setRotationVoltage(volts: Double) {
        SmartDashboard.putNumber(
            "set rotation voltage: ",
            MathUtil.clamp(volts, TrunkConstants.MIN_ROT_VOLTS, TrunkConstants.MAX_ROT_VOLTS)
        )
        masterRotationMotor.setVoltage(
            MathUtil.clamp(
                volts,
                TrunkConstants.MIN_ROT_VOLTS,
                TrunkConstants.MAX_ROT_VOLTS
            )
        );
//        masterRotationMotor.setControl(voltageVelocityController.withVelocity(volts))
    }

    override fun setServoAngle(angle: Double) {
        climbServo.angle = angle
    }

    override fun getServoAngle(): Double {
        return climbServo.angle
    }

    override fun periodic() {
    }
}
