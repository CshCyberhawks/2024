package frc.robot.commands.automatic

import MiscCalculations
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.RobotContainer
import frc.robot.TrunkPose
import frc.robot.commands.trunk.GoToClimbPoseTrunk
import frc.robot.commands.trunk.HoldPoseTrunk
import frc.robot.commands.trunk.LerpToPoseTrunk
import frc.robot.constants.TrunkConstants

class AutoClimbCommand : Command() {
    var climbCommand = GoToClimbPoseTrunk(TrunkPose.CLIMB).andThen(HoldPoseTrunk(TrunkPose.CLIMB))
//    val climbCommand = GoToClimbPoseTrunk(TrunkPose.CLIMB).andThen(holdCommand)

    var climbed = false

    // Go to climb pos
    // Position to .22
    // Angle to 62

    // or
    // Position to .23
    // Angle to 78
    // Position to .31

    override fun initialize() {
        RobotContainer.stateMachine.currentTrunkCommand = climbCommand
        RobotContainer.stateMachine.currentTrunkCommandLocked = true
        RobotContainer.actuallyDoClimb = false
        climbed = false
        TrunkConstants.MIN_ROT_VOLTS = -4.0
        climbCommand = GoToClimbPoseTrunk(TrunkPose.CLIMB).andThen(HoldPoseTrunk(TrunkPose.CLIMB))
    }

    override fun execute() {
        val positionSpeed = -MiscCalculations.calculateDeadzone(RobotContainer.xboxController.rightX, 0.1) / 1000.0
        val rotationSpeed = -MiscCalculations.calculateDeadzone(RobotContainer.xboxController.leftX, 0.1)
        //        climbCommand.goToPose.currentTargetPosition += speed
//        holdCommand.currentTargetPosition += positionSpeed
//        holdCommand.currentTargetRotation += rotationSpeed

        if (RobotContainer.actuallyDoClimb && !climbed) {
            RobotContainer.stateMachine.currentTrunkCommandLocked = false
            RobotContainer.stateMachine.currentTrunkCommand = LerpToPoseTrunk(
                TrunkPose.CLIMB_STAGE_1,
                0.5
            )/*.andThen(ParallelRaceGroup(HoldPoseTrunk(TrunkPose.CLIMB_STAGE_1), WaitCommand(1.0)))*/.andThen(
                LerpToPoseTrunk(TrunkPose.CLIMB_STAGE_2, 2.0)
            )/*.andThen(ParallelRaceGroup(HoldPoseTrunk(TrunkPose.CLIMB_STAGE_2), WaitCommand(1.0)))*/
                .andThen(LerpToPoseTrunk(TrunkPose.CLIMB_STAGE_FINAL, 0.5))
                .andThen(HoldPoseTrunk(TrunkPose.CLIMB_STAGE_FINAL))
            RobotContainer.stateMachine.currentTrunkCommandLocked = true


            climbed = true
        }

        if (RobotContainer.trunkSystem.isAtPose && RobotContainer.trunkSystem.trunkDesiredRotation == TrunkPose.CLIMB_STAGE_FINAL.angle) {
            RobotContainer.trunkSystem.positionLocked = true
        }
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.stateMachine.currentTrunkCommandLocked = false
        RobotContainer.actuallyDoClimb = false
        TrunkConstants.MIN_ROT_VOLTS = -2.0
        SmartDashboard.putBoolean("Pulldown Climb?", false)
    }
}
