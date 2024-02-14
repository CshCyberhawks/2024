package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.constants.ArmConstants
import frc.robot.subsystems.ArmSystem
import frc.robot.subsystems.ClawState
import frc.robot.subsystems.ClawSystem
import frc.robot.util.ControllerIO

/**
 * @property subsystem
 */
class TeleopArmCommand(private val subsystem: ArmSystem, private val clawSystem: ClawSystem) : Command() {
    private val armQueue = arrayListOf<GenericArmMovement>()

    // Called when the command is initially scheduled.
    override fun initialize() {
        subsystem.desiredArmAngle = 35.0

//        setCurrentCommand(AutoArmPosition(subsystem, 90.0, 0.0, false, true))
    }

    private fun armLogic() {
        if (armQueue.size != 0) {
            if (!armQueue[0].isRunning) {
                armQueue[0].run()
            }

            if (armQueue[0].isDone()) {
                armQueue.removeAt(0)
            }

            return
        }

        if (ControllerIO.toggleTilt) {
            subsystem.desiredTilt = !subsystem.desiredTilt
            println("TILTING")
        }

        if (ControllerIO.extensionExtended) {
            subsystem.desiredExtensionPosition = ArmConstants.armExtensionOut
//            subsystem.desiredExtensionPosition = ExtensionPosition.EXTENDED
            println("EXTENDING")
        }
        if (ControllerIO.extensionRetracted) {
            if (!subsystem.extensionInBeamBreak.get()) {
                subsystem.desiredExtensionPosition = 0.0
                println("RETRACTING")
            }
//            subsystem.desiredExtensionPosition = ExtensionPosition.RETRACTED
        }

        subsystem.desiredArmAngle += ControllerIO.controlArmAngle * 2

        if (ControllerIO.armAlignClosed) {
            armQueue.clear()
            armQueue.add(ExtensionMovement(subsystem, ArmConstants.ARM_EXTENSION_IN))
            armQueue.add(TiltMovement(subsystem, false))
            armQueue.add(AngleMovement(subsystem, ArmConstants.ARM_IN_ANGLE))
            println("ALIGN CLOSED")
        }

        if (ControllerIO.armAlignShelf) {
            armQueue.clear()
            subsystem.desiredArmAngle = ArmConstants.armShelfAngle
            println("ALIGN SHELF")
        }

        if (ControllerIO.armAlignFloorCube) {
            armQueue.clear()
            subsystem.desiredArmAngle = ArmConstants.armFloorCubeAngle
            subsystem.desiredTilt = true
            subsystem.desiredExtensionPosition = ArmConstants.ARM_EXTENSION_IN
            println("ALIGN FLOOR CUBE")
        }

        if (ControllerIO.armAlignFloorCone) {
            armQueue.clear()
            subsystem.desiredArmAngle = ArmConstants.armFloorConeAngle
            subsystem.desiredTilt = true
            subsystem.desiredExtensionPosition = ArmConstants.armExtensionCone
            println("ALIGN FLOOR CONE")
        }

        if (ControllerIO.armAlignHigh) {
            armQueue.clear()
            armQueue.add(AngleMovement(subsystem, ArmConstants.armMidAngle))
            armQueue.add(TiltMovement(subsystem, true))
            armQueue.add(AngleMovement(subsystem, ArmConstants.armHighAngle))
            armQueue.add(ExtensionMovement(subsystem, ArmConstants.armExtensionOut))
            println("ALIGN HIGH")
        }

        if (ControllerIO.armPlaceHigh) {
            armQueue.clear()
            armQueue.add(AngleMovement(subsystem, ArmConstants.armPlaceHighAngle))
            armQueue.add(ClawAction(clawSystem, ClawState.Spitting))
            armQueue.add(AngleMovement(subsystem, ArmConstants.armPlaceHighAngle - ArmConstants.armPlaceAngleDecrease))
            armQueue.add(ExtensionMovement(subsystem, ArmConstants.ARM_EXTENSION_IN))
            armQueue.add(TiltMovement(subsystem, false))
            armQueue.add(AngleMovement(subsystem, ArmConstants.ARM_IN_ANGLE))
            armQueue.add(ClawAction(clawSystem, ClawState.Idle))
            println("PLACE HIGH")
        }

        if (ControllerIO.armAlignMid) {
            armQueue.clear()
            armQueue.add(AngleMovement(subsystem, ArmConstants.armMidAngle))
            armQueue.add(ExtensionMovement(subsystem, ArmConstants.armExtensionMid))
            println("ALIGN MID")
        }

        if (ControllerIO.armPlaceMid) {
            armQueue.clear()
            armQueue.add(AngleMovement(subsystem, ArmConstants.armPlaceMidAngle))
            armQueue.add(ClawAction(clawSystem, ClawState.Spitting))
            armQueue.add(AngleMovement(subsystem, ArmConstants.armPlaceMidAngle - ArmConstants.armPlaceAngleDecrease))
            armQueue.add(ExtensionMovement(subsystem, ArmConstants.ARM_EXTENSION_IN))
            armQueue.add(AngleMovement(subsystem, ArmConstants.ARM_IN_ANGLE))
            armQueue.add(ClawAction(clawSystem, ClawState.Idle))
            println("PLACE MID")
        }
//
//        if (ControllerIO.armAlignMid) {
//            subsystem.desiredArmAngle = ArmConstants.armMidAngle
//            subsystem.desiredExtensionPosition = ArmConstants.armExtensionOut
//            subsystem.desiredTilt = false
//        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        armLogic()

        subsystem.run()
    }

    // Called once the command ends or is interruppted.
    override fun end(interrupted: Boolean) {}

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}
