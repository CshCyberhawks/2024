package frc.robot.commands.automatic

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.DriveState
import frc.robot.NoteState
import frc.robot.RobotContainer
import frc.robot.commands.cannon.AutoIntake
import frc.robot.constants.AutoConstants

class FloorIntakeAndSeek : Command() {
    val autoIntake: AutoIntake = AutoIntake()

    var initTime: Double = -1.0
    var firstTrackTime: Double = -1.0

    override fun initialize() {
        autoIntake.schedule()
        initTime = Timer.getFPGATimestamp()
        RobotContainer.stateMachine.driveState = DriveState.Auto
    }

    override fun execute() {
//        if (LimelightHelpers.getTV(RobotContainer.intakeLimelight) && firstTrackTime == -1.0) {
//            firstTrackTime = Timer.getFPGATimestamp()
//        }

//        if (LimelightHelpers.getTV(RobotContainer.intakeLimelight)) {
//            val fudgedLLOffset = LimelightHelpers.getTX(RobotContainer.intakeLimelight) * (1 / 27)
//            //Milan - everything needs to be negated bc the front is STUPID "sHoOtTeR sHoUlD be FrONt"
//            RobotContainer.swerveSystem.driveTrain.applyRequest { RobotContainer.swerveSystem.forwardStraight.withVelocityX(-1.0).withRotationalRate(-fudgedLLOffset) }
//        }
    }

    override fun isFinished(): Boolean {
        if (firstTrackTime == -1.0) {
            val timeSinceInit = Timer.getFPGATimestamp() - initTime
            if (timeSinceInit > AutoConstants.llFloorSeekTime) {
                return true
            }
        }

        //Doesn't use the auto intake just in case there is some sort of an issue with its ending and starting with a note already in the thing...idk - Milan
        return RobotContainer.stateMachine.noteState == NoteState.Stored
    }

    override fun end(interrupted: Boolean) {
        RobotContainer.stateMachine.driveState = DriveState.Teleop
        if (!autoIntake.isScheduled) {
            autoIntake.cancel()
        }
    }
}