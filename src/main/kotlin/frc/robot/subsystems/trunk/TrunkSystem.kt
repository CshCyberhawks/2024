package frc.robot.subsystems.trunk

import MiscCalculations
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.TrunkConstants


class TrunkSystem(val io: TrunkIO) : SubsystemBase() {

    val rotationPIDController: ProfiledPIDController = ProfiledPIDController(TrunkConstants.rotationKP, TrunkConstants.rotationKI, TrunkConstants.rotationKD, TrapezoidProfile.Constraints(TrunkConstants.rotationMaxVelo, TrunkConstants.rotationMaxAcceleration))

    //    val rotationPIDController = PIDController(TrunkConstants.rotationKP, TrunkConstants.rotationKI, TrunkConstants.rotationKD)
    val rotationFeedForward: ArmFeedforward = ArmFeedforward(TrunkConstants.rotationFFkS, TrunkConstants.rotationFFkG, TrunkConstants.rotationFFkV, TrunkConstants.rotationFFkA)

    val elevatorPIDController: PIDController = PIDController(TrunkConstants.positionKP, TrunkConstants.positionKI, TrunkConstants.positionKD)

    var trunkDesiredRotation = TrunkConstants.STOW_ANGLE

    init {
        SmartDashboard.putNumber("Trunk Target Position", TrunkConstants.TOP_BREAK_BEAM_POSITION)
    }

    var isAtPose: Boolean = false

    override fun periodic() {
        SmartDashboard.putNumber("Angle val", getRotation())
        SmartDashboard.putNumber("position val", getPosition())

//        if (atDesiredRotation()) {
//            io.setFalconThroughBoreOffset()
//        }
    }

    fun setDesiredRotation(desiredRot: Double) {
        rotationPIDController.goal = TrapezoidProfile.State(desiredRot, 0.0)
        trunkDesiredRotation = desiredRot
    }

    fun calculateRotationOut(desiredRot: Double): Double {
        trunkDesiredRotation = desiredRot
        SmartDashboard.putNumber("Trunk PID Desired Rot", trunkDesiredRotation)
        val rotationPIDOut = rotationPIDController.calculate(getRotation(), trunkDesiredRotation)
//        println("rotation PID out: " + rotationPIDOut)
        val rotationFFOut = rotationFeedForward.calculate(Math.toRadians(trunkDesiredRotation - 90.0), 0.0)
        SmartDashboard.putNumber("Trunk Rotation PID", rotationPIDOut)
        SmartDashboard.putNumber("Trunk Rotation FF", rotationFFOut)
        SmartDashboard.putNumber("Uncapped rotation voltage: ", rotationPIDOut + rotationFFOut)
        return MathUtil.clamp(rotationPIDOut
                + rotationFFOut, TrunkConstants.MIN_ROT_VOLTS, TrunkConstants.MAX_ROT_VOLTS)
    }

    fun calculatePositionOut(inputDesiredPosition: Double): Double {
        SmartDashboard.putNumber("Trunk Target Position", inputDesiredPosition)
        val posPIDOut = elevatorPIDController.calculate(getPosition(), inputDesiredPosition)
        val posFF = TrunkConstants.positionFF
        return posPIDOut + posFF
    }


    fun checkAtPose(pivotAngle: Double, elevatorPosition: Double): Boolean {
        return MiscCalculations.appxEqual(pivotAngle, getRotation(), TrunkConstants.ANGLE_DEADZONE) && MiscCalculations.appxEqual(elevatorPosition, getPosition(), TrunkConstants.ELEVATOR_DEADZONE)
    }

    fun checkAtPosition(position: Double): Boolean {
        return MiscCalculations.appxEqual(position, getPosition(), TrunkConstants.ELEVATOR_DEADZONE)
    }

    fun atDesiredRotation(): Boolean {
        return MiscCalculations.appxEqual(trunkDesiredRotation, getRotation(), TrunkConstants.ANGLE_DEADZONE)
    }

    fun getRotation(): Double {
        return frc.robot.util.Math.wrapAroundAngles((-io.getThroughBoreRawRotation() * 360.0) - TrunkConstants.rotationOffset)
//        return frc.robot.util.Math.wrapAroundAngles(io.getFalconRawRotation() * 360.0 - TrunkConstants.rotationOffset)
    }

    fun getPosition(): Double {
        return io.getRawPosition() * TrunkConstants.ELEVATOR2M
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
