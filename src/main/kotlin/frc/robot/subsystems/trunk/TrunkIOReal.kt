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

    //    private val rotationEncoder = mainRotationMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle)
    private val rotationEncoder = DutyCycleEncoder(TrunkConstants.rotationEncoderID)

    private val topLimit = DigitalInput(0)
    private val bottomLimit = DigitalInput(1)


    //    private val positionPID = elevatorMotor.pidController
//    private val positionPID: ProfiledPIDController = ProfiledPIDController(TrunkConstants.positionKP, TrunkConstants.positionKI, TrunkConstants.positionKD, TrapezoidProfile.Constraints(TrunkConstants.positionMaxRPM, TrunkConstants.positionMaxAcceleration))

//    private val rotationPID = mainRotationMotor.pidController

    private var lastRotationTime = 0.0
    private var lastRotationVelo = 0.0
    //    private val rotationPID = ProfiledPIDController(TrunkConstants.rotationKP, TrunkConstants.rotationKI, TrunkConstants.rotationKD, TrapezoidProfile.Constraints(TrunkConstants.rotationMaxVelo, TrunkConstants.rotationMaxAcceleration))


    private var bottomPositionLimit = TrunkConstants.BOTTOM_BREAK_BEAM_POSITION
    private var topPositionLimit = TrunkConstants.TOP_BREAK_BEAM_POSITION
    private var bottomRotationLimit = TrunkConstants.MIN_SAFE_ANGLE
    private var topRotationLimit = TrunkConstants.MAX_ANGLE


    init {
//        SmartDashboard.putNumber("position pid p", TrunkConstants.positionKP)
//        SmartDashboard.putNumber("position pid d", TrunkConstants.positionKD)

//        SmartDashboard.putNumber("Rotation P Gain", TrunkConstants.rotationKP)
//        SmartDashboard.putNumber("Rotation I Gain", TrunkConstants.rotationKI)
//        SmartDashboard.putNumber("Rotation D Gain", TrunkConstants.rotationKD)
//        SmartDashboard.putNumber("Rotation I Zone", TrunkConstants.rotationIz)
//        SmartDashboard.putNumber("position ff", TrunkConstants.positionFF)
//        SmartDashboard.putNumber("Position Max Output", TrunkConstants.positionMax)
//        SmartDashboard.putNumber("Rotation Min Output", TrunkConstants.rotationMin)
//        SmartDashboard.putNumber("Rotation Max Velocity", TrunkConstants.rotationMaxRPM)
//        SmartDashboard.putNumber("Rotation Min Velocity", TrunkConstants.rotationMinRPM)
//        SmartDashboard.putNumber("Rotation Max Acceleration", TrunkConstants.rotationMaxAcceleration)
//        SmartDashboard.putNumber("Rotation Allowed Closed Loop Error", TrunkConstants.rotationMaxError)
//        SmartDashboard.putNumber("Position SetPoint", 0.5)

        elevatorMotor.restoreFactoryDefaults()
        mainRotationMotor.restoreFactoryDefaults()
        followerRotationMotor.restoreFactoryDefaults()

        elevatorMotor.inverted = false // elevator likes to not be inverted idk why
        mainRotationMotor.inverted = false

//        rotationEncoder.positionConversionFactor = 360.0
//        rotationEncoder.distancePerRotation = 1.0 / (2 * Math.PI)

//        positionPID.setFeedbackDevice(positionEncoder)

        elevatorMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, false)
        elevatorMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, false)

        followerRotationMotor.follow(mainRotationMotor, true)

        mainRotationMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        followerRotationMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        elevatorMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)

//        positionPID.setP(TrunkConstants.positionKP)
//        positionPID.setI(TrunkConstants.positionKI)
//        positionPID.setD(TrunkConstants.positionKD)
//        positionPID.setIZone(TrunkConstants.positionIz)
//        positionPID.setFF(TrunkConstants.positionFF)
//        positionPID.setOutputRange(TrunkConstants.positionMin, TrunkConstants.positionMax)
//        positionPID.setSmartMotionMaxVelocity(TrunkConstants.positionMaxRPM, TrunkConstants.SMART_MOTION_SLOT)
//        positionPID.setSmartMotionMinOutputVelocity(TrunkConstants.positionMinRPM, TrunkConstants.SMART_MOTION_SLOT)
//        positionPID.setSmartMotionMaxAccel(TrunkConstants.positionMaxAcceleration, TrunkConstants.SMART_MOTION_SLOT)
//        positionPID.setSmartMotionAllowedClosedLoopError(TrunkConstants.positionMaxError, TrunkConstants.SMART_MOTION_SLOT)

//        rotationPID.setP(TrunkConstants.rotationKP)
//        rotationPID.setI(TrunkConstants.rotationKI)
//        rotationPID.setD(TrunkConstants.rotationKD)
//        rotationPID.setIZone(TrunkConstants.rotationIz)
//        rotationPID.setFF(TrunkConstants.rotationFF)
//        rotationPID.setOutputRange(TrunkConstants.rotationMin, TrunkConstants.rotationMax)
//        rotationPID.setSmartMotionMaxVelocity(TrunkConstants.rotationMaxRPM, TrunkConstants.SMART_MOTION_SLOT)
//        rotationPID.setSmartMotionMinOutputVelocity(TrunkConstants.rotationMinRPM, TrunkConstants.SMART_MOTION_SLOT)
//        rotationPID.setSmartMotionMaxAccel(TrunkConstants.rotationMaxAcceleration, TrunkConstants.SMART_MOTION_SLOT)
//        rotationPID.setSmartMotionAllowedClosedLoopError(TrunkConstants.rotationMaxError, TrunkConstants.SMART_MOTION_SLOT)
//        positionEncoder.positionConversionFactor = TrunkConstants.POSITION_CONVERSION_FACTOR
//        Thread.sleep(1000)
//        mainRotationMotor.burnFlash()
//        followerRotationMotor.burnFlash()
//        elevatorMotor.burnFlash()


//            lastRotationTime = Timer.getFPGATimestamp()
    }

    override fun setZeroPosition(top: Boolean) {
        positionEncoder.setPosition(
            if (top) {
                TrunkConstants.TOP_BREAK_BEAM_POSITION * TrunkConstants.M2ELEVATOR
            } else {
                TrunkConstants.BOTTOM_BREAK_BEAM_POSITION * TrunkConstants.M2ELEVATOR
            }
        )
        if (!top)
            elevatorMotor.inverted = !elevatorMotor.inverted
        setTopPositionLimit(TrunkConstants.TOP_BREAK_BEAM_POSITION)
        setBottomPositionLimit(TrunkConstants.BOTTOM_BREAK_BEAM_POSITION)
    }

    override fun atTopLimit(): Boolean {
        return topLimit.get()
    }

    override fun atBottomLimit(): Boolean {
        return bottomLimit.get()
    }

    override fun setPositionLimits(on: Boolean) {
//        elevatorMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, on)
//        elevatorMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, on)
    }

    override fun getRawPosition(): Double {
        return positionEncoder.position
    }

    override fun getRawRotation(): Double {
        return rotationEncoder.absolutePosition
    }


    override fun setTopPositionLimit(position: Double) {
        if (topPositionLimit != position) {
            topPositionLimit = position
            elevatorMotor.setSoftLimit(
                CANSparkBase.SoftLimitDirection.kForward,
                (position * TrunkConstants.M2ELEVATOR).toFloat()
            )
            elevatorMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true)
        }
    }

    override fun setBottomPositionLimit(position: Double) {
        if (bottomPositionLimit != position) {
            bottomPositionLimit = position
            elevatorMotor.setSoftLimit(
                CANSparkBase.SoftLimitDirection.kReverse,
                (position * TrunkConstants.M2ELEVATOR).toFloat()
            )
            elevatorMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true)
        }
    }

    override fun setTopRotationLimit(angle: Double) {
        if (topRotationLimit != angle) {
            topRotationLimit = angle
            mainRotationMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, angle.toFloat())
            mainRotationMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true)
        }
    }

    override fun setBottomRotationLimit(angle: Double) {
        if (bottomRotationLimit != angle) {
            bottomRotationLimit = angle
            mainRotationMotor.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, angle.toFloat())
            mainRotationMotor.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true)
        }
    }

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
//        val inputFF = SmartDashboard.getNumber("Position Feed Forward", 0.0)
//        if(inputFF != TrunkConstants.positionFF) {
//            positionPID.setFF(inputFF)
//            TrunkConstants.positionFF = inputFF
//        }

//        val posPIDP = SmartDashboard.getNumber("position pid p", TrunkConstants.positionKP)
//        val posPIDD = SmartDashboard.getNumber("position pidte d", TrunkConstants.positionKD)

//        rotationPID.setPID(anglePIDP, 0.0, anglePIDD)

//        positionPID.setPID(posPIDP, 0.0, posPIDD)

    }


}