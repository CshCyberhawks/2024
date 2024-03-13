package frc.robot.subsystems.trunk

import MiscCalculations
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.ElevatorFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotContainer
import frc.robot.TrunkPose
import frc.robot.TrunkState
import frc.robot.constants.TrunkConstants
import frc.robot.util.Telemetry
import frc.robot.RobotContainer.stateMachine
import kotlin.math.max


class TrunkSystem(private val io: TrunkIO) : SubsystemBase() {
    private val positionPID: PIDController =
            PIDController(TrunkConstants.positionKP, TrunkConstants.positionKI, TrunkConstants.positionKD)
    private val positionFF: ElevatorFeedforward = ElevatorFeedforward(0.0001, 0.27, 3.07, 0.09)

    private val rotationFF = ArmFeedforward(
            TrunkConstants.rotationFFkS,
            TrunkConstants.rotationFFkG,
            TrunkConstants.rotationFFkV,
            TrunkConstants.rotationFFkA
    )
    private val rotationPID =
            PIDController(TrunkConstants.rotationKP, TrunkConstants.rotationKI, TrunkConstants.rotationKD)

    private var enableRotationPID = true
    private var enablePositionPID = false

    var shootingAngle = TrunkPose.SPEAKER.angle

    val isAtAngle: Boolean
        get() =
            MiscCalculations.appxEqual(Math.toDegrees(rotationPID.setpoint), currentRotation, TrunkConstants.ANGLE_DEADZONE)
    val isAtPosition: Boolean
        get() =
            MiscCalculations.appxEqual(Math.toDegrees(positionPID.setpoint), currentPosition, TrunkConstants.POSITION_DEADZONE)

    private var currentPosition: Double = 0.0
    private var currentRotation: Double = getRotation()

    private fun setPIDs(on: Boolean) {
        enableRotationPID = on
        enablePositionPID = on
    }

    fun goManual() {
        io.setElevatorSpeed(0.0)
        io.setRotationSpeed(0.0)
        setPIDs(false)
        brakeMotors()
    }

    fun calibrate() {
        io.setRotationSpeed(0.0)
        setPIDs(false)
        stateMachine.trunkState = TrunkState.CALIBRATING
        io.setElevatorSpeed(0.2)
    }

    fun rotate(speed: Double) {
        if (stateMachine.trunkState == TrunkState.MANUAL)
            io.setRotationSpeed(speed)
    }

    fun elevate(speed: Double) {
        if (stateMachine.trunkState == TrunkState.MANUAL) {
            io.setElevatorSpeed(speed)
        }
    }

    private fun getRotation() = frc.robot.util.Math.wrapAroundAngles((-io.getRawRotation() * 360.0) - TrunkConstants.ROTATION_OFFSET)

    private fun getPosition() = io.getRawPosition() * TrunkConstants.ELEVATOR2M

    private fun setDesiredPosition(position: Double) { positionPID.setpoint = position }

    private fun setDesiredRotation(angle: Double) { rotationPID.setpoint = Math.toRadians(angle) }

    fun kill() {
        setPIDs(false)
        stateMachine.trunkState = TrunkState.STOP
        io.setElevatorSpeed(0.0)
        io.setRotationSpeed(0.0)
    }
    fun goToPose() {
        val state = stateMachine.trunkState
        val pose = stateMachine.targetTrunkPose
        updatePosition()
        when(state) {
            TrunkState.TRAVELING -> setDesiredPosition(pose.position)
            TrunkState.INTAKE -> if(pose != TrunkPose.INTAKE) leaveIntake()
            else -> {
                if(state == TrunkState.POSTTRAVEL && isAtPosition)
                    goToFinalAngle()
                else {
                    brakeMotors()
                    setPIDs(true)
                    stateMachine.trunkState = TrunkState.PRETRAVEL
                    setDesiredRotation(max(TrunkConstants.TRAVEL_ANGLE, stateMachine.targetTrunkPose.angle))
                }
            }
        }
        //    val targetPoseIsAboveCrossbar = stateMachine.targetTrunkPose.position >= TrunkConstants.CROSSBAR_TOP
//    val targetPoseIsBelowCrossbar = stateMachine.targetTrunkPose.position <= TrunkConstants.CROSSBAR_BOTTOM
//        val isSafe = MiscCalculations.appxEqual(currentPosition, TrunkConstants.STOW_POSITION, TrunkConstants.POSITION_DEADZONE)
    }

    private fun goToTravelPeriodic() {
        if(currentRotation >= TrunkConstants.TRAVEL_ANGLE - TrunkConstants.ANGLE_DEADZONE) {
            stateMachine.trunkState = TrunkState.TRAVELING
            setDesiredPosition(stateMachine.targetTrunkPose.position)
        }
    }

    private fun updatePosition() {
        currentRotation = getRotation()
        currentPosition = getPosition()
    }
    private fun goToIntake() {
        enableRotationPID = false
        io.rotationBrake = false
        setDesiredPosition(TrunkConstants.INTAKE_POSITION)
        stateMachine.trunkState = TrunkState.INTAKE
    }

    private fun leaveIntake() {
        setDesiredPosition(TrunkConstants.SAFE_TO_DROP_INTAKE_POSITION)
        stateMachine.trunkState = TrunkState.LEAVING_INTAKE
    }

    private fun leaveIntakePeriodic() {
        if(isAtPosition) {
            stateMachine.trunkState = TrunkState.PRETRAVEL
            goToPose()
        }
    }

    private fun goToFinalAngle() {
        when(stateMachine.targetTrunkPose) {
            TrunkPose.INTAKE -> goToIntake()
            TrunkPose.SPEAKER -> setDesiredRotation(shootingAngle)
            else -> setDesiredRotation(stateMachine.targetTrunkPose.angle)
        }
    }

    override fun periodic() {
        if (io.atTopLimit())
            io.setZeroPosition()

        updatePosition()

        when(stateMachine.trunkState) {
            TrunkState.POSTTRAVEL -> goToFinalAngle()
            TrunkState.PRETRAVEL -> goToTravelPeriodic()
            TrunkState.TRAVELING -> travelPeriodic()
            TrunkState.LEAVING_INTAKE -> leaveIntakePeriodic()
            TrunkState.CALIBRATING -> calibratePeriodic()
            else -> {}
        }

        if (enablePositionPID)
            io.setElevatorSpeed(positionPID.calculate(currentPosition) + TrunkConstants.positionFF)

        if (enableRotationPID) {
            var pidVal: Double = rotationPID.calculate(Math.toRadians(currentRotation))

            if (Math.toDegrees(rotationPID.setpoint) < currentRotation) {
                pidVal *= .5
            }
//          var rotationMiniFF = 0.0
            //The * (.1) is to convert to volts from the angle distance and the .6 is just a fudge
//                if (abs(pidVal) <= .4) {
//                    rotationMiniFF = MathUtil.clamp(((Math.toDegrees(rotationPID.setpoint) - getRotation()) * .2), -.8, 0.8)
//                }
            val twistVolts =  MathUtil.clamp((pidVal
                    + rotationFF.calculate(rotationPID.setpoint - (Math.PI / 2.0), 0.0)), -.5, 2.0)
            io.setRotationVoltage(twistVolts)
        }
        io.periodic()
    }


    fun freeMotors() {
        io.rotationBrake = false
        io.positionBrake = false
        io.setRotationSpeed(0.0)
    }

    fun brakeMotors() {
        io.rotationBrake = true
        io.positionBrake = true
    }

    private fun calibratePeriodic() {
        if (io.atTopLimit()) {
            io.setElevatorSpeed(0.0)
            io.setZeroPosition()
            stateMachine.targetTrunkPose = TrunkPose.STOW
        }
    }

    private fun travelPeriodic() {
        if(MiscCalculations.appxEqual(currentPosition, stateMachine.targetTrunkPose.position, TrunkConstants.POSITION_DEADZONE)) {
            stateMachine.trunkState = TrunkState.POSTTRAVEL
            setDesiredRotation(stateMachine.targetTrunkPose.angle)
        }
    }
}