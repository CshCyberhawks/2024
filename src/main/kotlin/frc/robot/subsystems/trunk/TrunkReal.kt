package frc.robot.subsystems.trunk

import com.revrobotics.CANSparkBase
import com.revrobotics.CANSparkLowLevel
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.constants.TrunkConstants
import frc.robot.subsystems.TrunkSystem

class TrunkReal : TrunkIO {
    private val elevatorMotor = CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushless)

    //    private val leftRotationMotor = CANSparkMax(13, CANSparkLowLevel.MotorType.kBrushless)
    //    private val rightRotationMotor = CANSparkMax(14, CANSparkLowLevel.MotorType.kBrushless)
    private val positionEncoder = elevatorMotor.getAlternateEncoder(TrunkConstants.POSITION_GEAR_RATIO * 8192) // 8192 counts per revolution for a through bore encoder https://www.revrobotics.com/rev-11-1271/
    //
    //    private val mainRotationMotor = rightRotationMotor // remember to make other one follow this
    //    private val followerRotationMotor = leftRotationMotor
    //
    //    private val rotationEncoder = mainRotationMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle)

    private val leftShooter = CANSparkMax(17, CANSparkLowLevel.MotorType.kBrushless)
    private val rightShooter = CANSparkMax(18, CANSparkLowLevel.MotorType.kBrushless)
//    private val positionPID = elevatorMotor.pidController
//    private val rotationPID = mainRotationMotor.pidController

    init {
        //        leftRotationMotor.restoreFactoryDefaults()
        //        rightRotationMotor.restoreFactoryDefaults()
        //
        //        elevatorMotor.inverted = false
        //        mainRotationMotor.inverted = false
        //
        //        mainRotationMotor.getForwardLimitSwitch(kNormallyOpen).enableLimitSwitch(false)
        //        mainRotationMotor.getReverseLimitSwitch(kNormallyOpen).enableLimitSwitch(false)
        //        elevatorMotor.getForwardLimitSwitch(kNormallyOpen).enableLimitSwitch(false)
        //        elevatorMotor.getReverseLimitSwitch(kNormallyOpen).enableLimitSwitch(false)
        //
        //        followerRotationMotor.follow(mainRotationMotor, true)
        //
        //        mainRotationMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        //        followerRotationMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        //        elevatorMotor.setIdleMode(CANSparkBase.IdleMode.kBrake)
        //
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
        //
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

        rightShooter.inverted = true
        leftShooter.inverted = false

        rightShooter.setSmartCurrentLimit(80)
        leftShooter.setSmartCurrentLimit(80)
        //        rightShooter.closedLoopRampRate = 2.5
        //        rightShooter.closedLoopRampRate = 2.5
    }

    override fun periodic() {
        /*
        val inputElevatorP = SmartDashboard.getNumber("Elevator P Gain", 0.0);
        val inputElevatorI = SmartDashboard.getNumber("Elevator I Gain", 0.0);
        val inputElevatorD = SmartDashboard.getNumber("Elevator D Gain", 0.0);
        val inputElevatorIz = SmartDashboard.getNumber("Elevator I Zone", 0.0);
        val inputElevatorFf = SmartDashboard.getNumber("Elevator Feed Forward", 0.0);
        val inputElevatorMax = SmartDashboard.getNumber("Elevator Max Output", 0.0);
        val inputElevatorMin = SmartDashboard.getNumber("Elevator Min Output", 0.0);
        val inputElevatorMaxV = SmartDashboard.getNumber("Elevator Max Velocity", 0.0);
        val inputElevatorMinV = SmartDashboard.getNumber("Elevator Min Velocity", 0.0);
        val inputElevatorMaxA = SmartDashboard.getNumber("Elevator Max Acceleration", 0.0);
        val inputElevatorAllE = SmartDashboard.getNumber("Elevator Allowed Closed Loop Error", 0.0);

        if(inputElevatorP != positionKP) {
            positionPID.setP(inputElevatorP)
            positionKP = inputElevatorP
        }
        if(inputElevatorI != positionKI) {
            positionPID.setI(inputElevatorI)
            positionKI = inputElevatorI
        }
        if(inputElevatorD != positionKD) {
            positionPID.setD(inputElevatorD)
            positionKD = inputElevatorD
        }
        if(inputElevatorIz != positionIz) {
            positionPID.setIZone(inputElevatorIz)
            positionIz = inputElevatorIz
        }
        if(inputElevatorFf != positionFF) {
            positionPID.setFF(inputElevatorFf)
            positionFF = inputElevatorFf
        }
        if((inputElevatorMax != positionMax) || (inputElevatorMin != positionMin)) {
            positionPID.setOutputRange(inputElevatorMin, inputElevatorMax)
            positionMin = inputElevatorMin
            positionMax = inputElevatorMax
        }
        if(inputElevatorMaxV != positionMaxRPM) {
            positionPID.setSmartMotionMaxVelocity(inputElevatorMaxV, SMART_MOTION_SLOT)
            positionMaxRPM = inputElevatorMaxV
        }
        if(inputElevatorMaxA != positionMaxAcceleration) {
            positionPID.setSmartMotionMaxAccel(inputElevatorMaxA, SMART_MOTION_SLOT)
            positionMaxAcceleration = inputElevatorMaxA
        }
        if(inputElevatorMinV != positionMinRPM) {
            positionPID.setSmartMotionMinOutputVelocity(inputElevatorMinV, SMART_MOTION_SLOT)
            positionMinRPM = inputElevatorMinV
        }
        if(inputElevatorAllE != positionMaxError) {
            positionPID.setSmartMotionAllowedClosedLoopError(inputElevatorAllE, SMART_MOTION_SLOT)
            positionMaxError = inputElevatorAllE
        }

        val inputRotationP = SmartDashboard.getNumber("Rotation P Gain", 0.0);
        val inputRotationI = SmartDashboard.getNumber("Rotation I Gain", 0.0);
        val inputRotationD = SmartDashboard.getNumber("Rotation D Gain", 0.0);
        val inputRotationIz = SmartDashboard.getNumber("Rotation I Zone", 0.0);
        val inputRotationFf = SmartDashboard.getNumber("Rotation Feed Forward", 0.0);
        val inputRotationMax = SmartDashboard.getNumber("Rotation Max Output", 0.0);
        val inputRotationMin = SmartDashboard.getNumber("Rotation Min Output", 0.0);
        val inputRotationMaxV = SmartDashboard.getNumber("Rotation Max Velocity", 0.0);
        val inputRotationMinV = SmartDashboard.getNumber("Rotation Min Velocity", 0.0);
        val inputRotationMaxA = SmartDashboard.getNumber("Rotation Max Acceleration", 0.0);
        val inputRotationAllE = SmartDashboard.getNumber("Rotation Allowed Closed Loop Error", 0.0);

        if(inputRotationP != rotationKP) {
            rotationPID.setP(inputRotationP)
            rotationKP = inputRotationP
        }
        if(inputRotationI != rotationKI) {
            rotationPID.setI(inputRotationI)
            rotationKI = inputRotationI
        }
        if(inputRotationD != rotationKD) {
            rotationPID.setD(inputRotationD)
            rotationKD = inputRotationD
        }
        if(inputRotationIz != rotationIz) {
            rotationPID.setIZone(inputRotationIz)
            rotationIz = inputRotationIz
        }
        if(inputRotationFf != rotationFF) {
            rotationPID.setFF(inputRotationFf)
            rotationFF = inputRotationFf
        }
        if((inputRotationMax != rotationMax) || (inputRotationMin != positionMin)) {
            rotationPID.setOutputRange(inputRotationMin, inputRotationMax)
            rotationMin = inputRotationMin
            rotationMax = inputRotationMax
        }
        if(inputRotationMaxV != rotationMaxRPM) {
            rotationPID.setSmartMotionMaxVelocity(inputRotationMaxV, SMART_MOTION_SLOT)
            rotationMaxRPM = inputRotationMaxV
        }
        if(inputRotationMaxA != rotationMaxAcceleration) {
            rotationPID.setSmartMotionMaxAccel(inputRotationMaxA, SMART_MOTION_SLOT)
            rotationMaxAcceleration = inputRotationMaxA
        }
        if(inputRotationMinV != rotationMinRPM) {
            rotationPID.setSmartMotionMinOutputVelocity(inputRotationMinV, SMART_MOTION_SLOT)
            rotationMinRPM = inputRotationMinV
        }
        if(inputRotationAllE != rotationMaxError) {
            rotationPID.setSmartMotionAllowedClosedLoopError(inputRotationAllE, SMART_MOTION_SLOT)
            rotationMaxError = inputRotationAllE
        }

        val rotationSetPoint = SmartDashboard.getNumber("Rotation Set Position", 0.0)
        rotationPID.setReference(rotationSetPoint, CANSparkBase.ControlType.kSmartMotion)
        val rotation = rotationEncoder.position

        val setPoint = SmartDashboard.getNumber("Set Position", 0.0)
        positionPID.setReference(setPoint, CANSparkBase.ControlType.kSmartMotion)
        val pos = positionEncoder.position

        SmartDashboard.putNumber("Elevator SetPoint", setPoint)
        SmartDashboard.putNumber("Elevator Position", pos)
        SmartDashboard.putNumber("Elevator Motor Output", elevatorMotor.getAppliedOutput())
        SmartDashboard.putNumber("Rotation SetPoint", setPoint)
        SmartDashboard.putNumber("Rotation Position", pos)
        SmartDashboard.putNumber("Rotation Motor Output", mainRotationMotor.getAppliedOutput())
        */
    }

    override fun getPosition(): Double {
//        return positionEncoder.position / TrunkConstants.ELEVATOR_LENGTH_REVOLUTIONS
        TODO()
    }

    override fun getRotation(): Double {
//        return rotationEncoder.position
        TODO()
    }

    override fun setDesiredPosition(position: Double) {
//        positionPID.setReference(position * TrunkConstants.ELEVATOR_LENGTH_REVOLUTIONS, CANSparkBase.ControlType.kSmartMotion)
    }

    override fun setDesiredRotation(angle: Double) {
//        rotationPID.setReference(angle, CANSparkBase.ControlType.kSmartMotion)
    }

    override fun setShootSpeed(left: Double, right: Double) {
        SmartDashboard.putNumber("Left Speed", left)
        SmartDashboard.putNumber("Right Speed", right)
        SmartDashboard.putNumber("Left Current", leftShooter.outputCurrent)
        SmartDashboard.putNumber("Right Current", rightShooter.outputCurrent)
        leftShooter.set(left)
        rightShooter.set(right)
    }
}