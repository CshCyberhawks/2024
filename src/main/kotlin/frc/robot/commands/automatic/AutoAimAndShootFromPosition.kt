package frc.robot.commands.automatic


import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.*
import frc.robot.commands.cannon.AutoShootCommand


class AutoAimAndShootFromPosition(val position: Pose2d) : Command() {
    private val autoShoot: AutoShootCommand = AutoShootCommand()

    override fun initialize() {
        RobotContainer.stateMachine.shooterState = ShooterState.Shooting

        RobotContainer.stateMachine.targetTrunkPose = TrunkPose.SPEAKER
        RobotContainer.trunkSystem.goToAim()

//        if (RobotContainer.stateMachine.targetTrunkPose != TrunkPosition.SPEAKER && RobotContainer.stateMachine.targetTrunkPose != TrunkPosition.SPEAKER_FROM_STAGE) {
//            if (RobotContainer.stateMachine.currentRobotZone == GlobalZones.Stage) {
//                RobotContainer.stateMachine.targetTrunkPose = TrunkPosition.SPEAKER_FROM_STAGE
//            } else {
//                RobotContainer.stateMachine.targetTrunkPose = TrunkPosition.SPEAKER
//            }
//        }
    }

    override fun execute() {
        val shotSetup = RobotContainer.targetingSystem.getShotNoVelocityFromPosition(position)

        //Handle the cannon aiming component
//        val shooterAngle = clamp(shotSetup.shooterAngle, TrunkConstants.MIN_SHOOT_ANGLE, TrunkConstants.MAX_SHOOT_ANGLE)
//        SmartDashboard.putBoolean("shot is possible?", shooterAngle == shotSetup.shooterAngle)
//        RobotContainer.trunkSystem.setShootingAngle(90 - shooterAngle)
//        RobotContainer.trunkSystem.goToCustom()
        //Handle the cannon aiming component
        val shooterAngle = 58.0


        //70 = angle from back bumper on the wing line (5.37m, 6.37)
        //angle = 58 from the preload line (2.90m, 5.55)
        // angle = X from amp (1.78, 7.32)



//        println("shooting angle " + shooterAngle)
        RobotContainer.trunkSystem.setShootingAngle(shooterAngle)
//                RobotContainer.trunkSystem.setShootingAngle(51.2)
//        RobotContainer.trunkSystem.setShootingAngle(51.2)
//

        SmartDashboard.putBoolean("Is at angle", RobotContainer.trunkSystem.isAtAngle)

        SmartDashboard.putBoolean("Shooter Ready & Aimed", RobotContainer.stateMachine.shooterReady && RobotContainer.trunkSystem.isAtAngle)


        //Can we shoot?
        if (RobotContainer.xboxController.leftTrigger().asBoolean && !autoShoot.isScheduled) {
            autoShoot.schedule()
        }
//        if (RobotContainer.leftJoystick.button(2).asBoolean) {
//            autoShoot.schedule()
//            println("scheduling auto shoot")
//        }
    }

    override fun isFinished(): Boolean {
        return (autoShoot.isFinished)
    }

    override fun end(interrupted: Boolean) {
        println("Shootyboi Done")
        RobotContainer.stateMachine.shooterState = ShooterState.Stopped
        RobotContainer.stateMachine.driveState = DriveState.Teleop
        RobotContainer.stateMachine.targetTrunkPose = TrunkPose.STOW
        RobotContainer.trunkSystem.goToCustom()
    }
}