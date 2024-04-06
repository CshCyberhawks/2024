package frc.robot.subsystems.trunk

import MiscCalculations
import com.ctre.phoenix6.controls.NeutralOut
import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.TrunkConstants
import frc.robot.util.ProfiledPID
import frc.robot.util.Timer

class TrunkSystem(val io: TrunkIO) : SubsystemBase() {

    val lowRotationPIDController = ProfiledPIDController(
        TrunkConstants.lowRotationKP,
        TrunkConstants.lowRotationKI,
        TrunkConstants.lowRotationKD,
        TrapezoidProfile.Constraints(TrunkConstants.lowRotationMaxVelo, TrunkConstants.lowRotationMaxAcceleration)
    )

    val highRotationPIDController = ProfiledPIDController(
        TrunkConstants.highRotationKP,
        TrunkConstants.highRotationKI,
        TrunkConstants.highRotationKD,
        TrapezoidProfile.Constraints(TrunkConstants.highRotationMaxVelo, TrunkConstants.highRotationMaxAcceleration)
    )

    //    val rotationPIDController = PIDController(TrunkConstants.rotationKP, TrunkConstants.rotationKI, TrunkConstants.rotationKD)
    val lowRotationFeedForward: ArmFeedforward = ArmFeedforward(
        TrunkConstants.lowRotationFFkS,
        TrunkConstants.lowRotationFFkG,
        TrunkConstants.lowRotationFFkV,
        TrunkConstants.lowRotationFFkA
    )

    val highRotationFeedForward: ArmFeedforward = ArmFeedforward(
        TrunkConstants.highRotationFFkS,
        TrunkConstants.highRotationFFkG,
        TrunkConstants.highRotationFFkV,
        TrunkConstants.highRotationFFkA
    )

    val climbRotationPIDController: ProfiledPIDController = ProfiledPIDController(
        TrunkConstants.climbRotationKP,
        TrunkConstants.climbRotationKI,
        TrunkConstants.climbRotationKD,
        TrapezoidProfile.Constraints(TrunkConstants.climbRotationMaxVelo, TrunkConstants.climbRotationMaxAcceleration)
    )

    var positionLocked = false
        set(value) {
            if (value)
                io.setServoAngle(0.0)
            else
                io.setServoAngle(90.0)
            field = value
        }

    val elevatorPIDController =
        ProfiledPIDController(
            TrunkConstants.positionKP,
            TrunkConstants.positionKI,
            TrunkConstants.positionKD,
            TrapezoidProfile.Constraints(0.5, 2.0)
        )

    var trunkDesiredRotation = TrunkConstants.STOW_ANGLE
    var trunkDesiredPosition = TrunkConstants.STOW_POSITION

    var falconRotationOffset = 0.0

    var falconRotationZeroed = false

    init {
        SmartDashboard.putNumber("Trunk Target Position", TrunkConstants.STOW_BREAK_BEAM_POSITION)
        SmartDashboard.putData("High Profiled PID", highRotationPIDController)
        SmartDashboard.putData("Low Profiled PID", lowRotationPIDController)
        SmartDashboard.putData("Climb Profiled PID", climbRotationPIDController)
        SmartDashboard.putData("Position PID", elevatorPIDController)

        positionLocked = false
    }

    var isAtPose: Boolean = false

    var lastVelocity = 0.0

    val clock = Timer()

    var lastRotationBrakeMode = true
    var lastPositionBrakeMode = true

    var lastDesiredPosition = -1.0

    override fun periodic() {
        SmartDashboard.putNumber("Angle val", getThroughboreRotation())
        SmartDashboard.putNumber("position val", getPosition())

//        if (atDesiredRotation()) {
//            io.setFalconThroughBoreOffset()
//        }
        var time = clock.get()
        time = if (time > 0.0) time else 1.0
        val velocity = io.getPivotVelocity()
        val acceleration = (velocity - lastVelocity) / time

        SmartDashboard.putNumber("Pivot Velocity", velocity)
        SmartDashboard.putNumber("Pivot Acceleration", acceleration)


        SmartDashboard.putNumber(
            "Elevator velocity",
            io.getElevatorVelocity() * TrunkConstants.ELEVATOR_ROTATIONS_TO_METERS
        )

        SmartDashboard.putNumber(
            "Elevator Acceleration",
            io.getElevatorMotorAccel() * (2.0 / 15.0) * TrunkConstants.ELEVATOR_ROTATIONS_TO_METERS
        )

        lastVelocity = velocity
        clock.restart()

//        if (io.atTopLimit() && getPosition() < 0.35) {
//            io.setZeroPosition()
//        }

        if (lastRotationBrakeMode != io.rotationBrake) {
            lowRotationPIDController.reset(getThroughboreRotation())

            lastRotationBrakeMode = io.rotationBrake
        }

        if (lastPositionBrakeMode != io.positionBrake) {
            elevatorPIDController.reset(getPosition())

            lastPositionBrakeMode = io.positionBrake
        }

        if (io.rotationBrake && atDesiredRotation() && !falconRotationZeroed) {
            falconRotationOffset = getRawFalconRotation() - getThroughboreRotation()
            falconRotationZeroed = true
//            println("Zeroed falcon rotation")
        }

        SmartDashboard.putNumber("Falcon Offset", falconRotationOffset)

        SmartDashboard.putBoolean("Falcon Rotation Zeroed", falconRotationZeroed)
        SmartDashboard.putBoolean("Trunk Rotation Brake", io.rotationBrake)
        SmartDashboard.putBoolean("Trunk At Desired Rotation", atDesiredRotation())

//        lowRotationPIDController.logStates("Low Rotation PID")

    }

    fun setDesiredRotation(desiredRot: Double) {
//        println("Trunk Desired Rot: ${desiredRot}")
        lowRotationPIDController.goal = TrapezoidProfile.State(desiredRot, 0.0)
        highRotationPIDController.goal = TrapezoidProfile.State(desiredRot, 0.0)
        climbRotationPIDController.goal = TrapezoidProfile.State(desiredRot, 0.0)
        trunkDesiredRotation = desiredRot
    }

    fun setDesiredPosition(desiredPos: Double) {
//        println("Desired Position: ${desiredPos}")
        elevatorPIDController.goal = TrapezoidProfile.State(desiredPos, 0.0)
        trunkDesiredPosition = desiredPos
    }

    fun calculateRotationOut(desiredRot: Double, climb: Boolean = false): Double {
        trunkDesiredRotation = desiredRot
        SmartDashboard.putNumber("SOMETHING STUPID Trunk PID Desired Rot", trunkDesiredRotation)
        SmartDashboard.putNumber("SOMETHING STUPID Trunk PID Rot", getFalconRotation())
        SmartDashboard.putNumber("SOMETHING STUPID Trunk PID TB Rot", getThroughboreRotation())
        val rotationPIDOut = if (climb) {
            climbRotationPIDController.calculate(getFalconRotation(), trunkDesiredRotation)
//        } else if (trunkDesiredRotation > 100.0) {// || getThroughboreRotation() > 100.0) {
//            highRotationPIDController.calculate(getFalconRotation(), trunkDesiredRotation)
        } else {
            lowRotationPIDController.calculate(getThroughboreRotation())
        }
//        SmartDashboard.putNumber("Trunk Trapezoid Velocity Error", `lowRotationPIDController`.velocityError)

//        println("rotation PID out: " + rotationPIDOut)
        val rotationFFOut = if (trunkDesiredRotation > 100.0) {// || getThroughboreRotation() > 100.0) {
            highRotationFeedForward.calculate(Math.toRadians(trunkDesiredRotation - 90.0), 0.0)
        } else {
            lowRotationFeedForward.calculate(Math.toRadians(trunkDesiredRotation - 90.0), 0.0)
        }
        SmartDashboard.putNumber("SOMETHING STUPID Trunk Rotation PID", rotationPIDOut)
        SmartDashboard.putNumber("SOMETHING STUPID Trunk Rotation Error", highRotationPIDController.positionError)
        SmartDashboard.putNumber("SOMETHING STUPID Trunk Rotation FF", rotationFFOut)
        SmartDashboard.putNumber("SOMETHING STUPID Uncapped rotation voltage: ", rotationPIDOut + rotationFFOut)
        return MathUtil.clamp(
            rotationPIDOut
                    + rotationFFOut, TrunkConstants.MIN_ROT_VOLTS, TrunkConstants.MAX_ROT_VOLTS
        )
    }

//    fun calculateClimbRotationOut(desiredRot: Double): Double {
//        trunkDesiredRotation = desiredRot
//        SmartDashboard.putNumber("SOMETHING STUPID Trunk PID Desired Rot", trunkDesiredRotation)
//        SmartDashboard.putNumber("SOMETHING STUPID Trunk PID Rot", getFalconRotation())
//        SmartDashboard.putNumber("SOMETHING STUPID Trunk PID TB Rot", getThroughboreRotation())
//        val rotationPIDOut = highRotationPIDController.calculate(getFalconRotation(), trunkDesiredRotation)
//
//        SmartDashboard.putNumber("Trunk Trapezoid Velocity Error", climbRotationPIDController.velocityError)
//
//        //        println("rotation PID out: " + rotationPIDOut)
//        val rotationFFOut = lowRotationFeedForward.calculate(Math.toRadians(trunkDesiredRotation - 90.0), 0.0)
//
//        SmartDashboard.putNumber("SOMETHING STUPID Trunk Rotation PID", rotationPIDOut)
//        SmartDashboard.putNumber("SOMETHING STUPID Trunk Rotation Error", climbRotationPIDController.positionError)
//        SmartDashboard.putNumber("SOMETHING STUPID Trunk Rotation FF", rotationFFOut)
//        SmartDashboard.putNumber("SOMETHING STUPID Uncapped rotation voltage: ", rotationPIDOut + rotationFFOut)
//        return MathUtil.clamp(
//            rotationPIDOut
//                    + rotationFFOut, TrunkConstants.MIN_ROT_VOLTS, TrunkConstants.MAX_ROT_VOLTS
//        )
//    }

    fun calculatePositionOut(inputDesiredPosition: Double): Double {
        if (positionLocked) {
            return 0.0
        }
//        if (inputDesiredPosition != lastDesiredPosition) {
//            elevatorPIDController.goal = TrapezoidProfile.State(inputDesiredPosition, 0.0)
//
//            lastDesiredPosition = inputDesiredPosition
//        }
        SmartDashboard.putNumber("Trunk Target Position", inputDesiredPosition)
        val posPIDOut = elevatorPIDController.calculate(getPosition())
        SmartDashboard.putNumber("Elevator PID Out", posPIDOut)
        val posFF = TrunkConstants.positionFF
        SmartDashboard.putNumber("Elevator FF", posFF)
        SmartDashboard.putNumber("Elevator Out", posPIDOut + posFF)
        SmartDashboard.putNumber("Elevator Velocity Error", elevatorPIDController.velocityError)


        return posPIDOut + posFF
    }

    fun checkAtPose(
        pivotAngle: Double,
        elevatorPosition: Double,
        angleDeadzone: Double = TrunkConstants.ANGLE_DEADZONE
    ): Boolean {
        SmartDashboard.putBoolean(
            "Pivot at Pose", MiscCalculations.appxEqual(
                pivotAngle,
                getThroughboreRotation(),
                angleDeadzone
            )
        )
        SmartDashboard.putNumber("Pivot Angle Diff", pivotAngle - getThroughboreRotation())
        SmartDashboard.putBoolean(
            "Elevator at Pose",
            MiscCalculations.appxEqual(elevatorPosition, getPosition(), TrunkConstants.ELEVATOR_DEADZONE)
        )
        SmartDashboard.putNumber("Angle Deadzone", angleDeadzone)
        return MiscCalculations.appxEqual(
            pivotAngle,
            getThroughboreRotation(),
            angleDeadzone
        ) && MiscCalculations.appxEqual(elevatorPosition, getPosition(), TrunkConstants.ELEVATOR_DEADZONE)
    }

    fun checkAtClimbPose(pivotAngle: Double, elevatorPosition: Double): Boolean {
        return MiscCalculations.appxEqual(
            pivotAngle,
            getFalconRotation(),
            TrunkConstants.ANGLE_DEADZONE
        ) && MiscCalculations.appxEqual(elevatorPosition, getPosition(), TrunkConstants.ELEVATOR_DEADZONE)
    }

    fun checkAtPosition(position: Double): Boolean {
        return MiscCalculations.appxEqual(position, getPosition(), TrunkConstants.ELEVATOR_DEADZONE)
    }

    fun atDesiredRotation(): Boolean {
        return MiscCalculations.appxEqual(
            trunkDesiredRotation,
            getThroughboreRotation(),
            TrunkConstants.ANGLE_DEADZONE
        )
    }

    fun getThroughboreRotation(): Double {
        return frc.robot.util.Math.wrapAroundAngles((-io.getThroughBoreRawRotation() * 360.0) - TrunkConstants.throughboreRotationOffset)
//        return frc.robot.util.Math.wrapAroundAngles(io.getFalconRawRotation() * 360.0 - TrunkConstants.rotationOffset)
    }

    fun getFalconRotation(): Double {
        return frc.robot.util.Math.wrapAroundAngles((io.getFalconRawRotation() * 360.0) - falconRotationOffset)
    }

    private fun getRawFalconRotation(): Double {
        return frc.robot.util.Math.wrapAroundAngles((io.getFalconRawRotation() * 360.0))
    }

    fun getPosition(): Double {
        return io.getPosition() * TrunkConstants.ELEVATOR_ROTATIONS_TO_METERS
    }

    fun freeMotors() {
        io.rotationBrake = false
        io.positionBrake = false
    }

    fun brakeMotors() {
        io.rotationBrake = true
        io.positionBrake = true
    }
}
