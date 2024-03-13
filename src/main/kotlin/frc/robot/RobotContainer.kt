package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.commands.TeleopSwerveDriveCommand
import frc.robot.commands.UnBreakTheIK
import frc.robot.commands.automatic.AutoAimAndShoot
import frc.robot.commands.automatic.AutoAimAndShootFromPosition
import frc.robot.commands.automatic.FloorIntakeAndSeek
import frc.robot.commands.cannon.AutoAmp
import frc.robot.commands.cannon.AutoIntake
import frc.robot.commands.cannon.AutoSpit
import frc.robot.subsystems.VisionSystem
import frc.robot.subsystems.cannon.CannonIOReal
import frc.robot.subsystems.cannon.CannonSystem
import frc.robot.subsystems.swerve.SwerveSystem
import frc.robot.subsystems.trunk.TrunkIOReal
import frc.robot.subsystems.trunk.TrunkSystem
import frc.robot.util.TargetingSystem
import frc.robot.util.TelemetryToggles

object RobotContainer {
    val leftJoystick: CommandJoystick = CommandJoystick(0)
    val rightJoystick: CommandJoystick = CommandJoystick(1)
    val xboxController: CommandXboxController = CommandXboxController(2)

    val telemetry = TelemetryToggles()


    val trunkSystem = TrunkSystem(TrunkIOReal())

    val stateMachine: RobotStateMachine = RobotStateMachine()

    val cannonSystem: CannonSystem = CannonSystem(CannonIOReal())

    val autonomousCommand: Command = Commands.run({})

    var teleopSwerveCommand: Command = TeleopSwerveDriveCommand()


    val targetingSystem: TargetingSystem = TargetingSystem()

    val visionSystem: VisionSystem = VisionSystem()


//    val autoChooser: SendableChooser<Command> = AutoBuilder.buildAutoChooser()

    val autoStateManagementEnableButton: Boolean
        get() = SmartDashboard.getBoolean("Enable Automatic State Management", false)

    val robotActionSendable: SendableChooser<RobotAction> = SendableChooser<RobotAction>()
    val shootPositionSendable: SendableChooser<ShootPosition> = SendableChooser<ShootPosition>()
    val trunkPoseSendable: SendableChooser<TrunkPose> = SendableChooser<TrunkPose>()

    val swerveSystem: SwerveSystem = SwerveSystem()

    val intakeLimelight = "limelight-back"

    init {
        configureBindings()

        RobotAction.entries.forEach {
            robotActionSendable.addOption(it.name, it)
        }

        ShootPosition.entries.forEach {
            shootPositionSendable.addOption(it.name, it)
        }

        TrunkPose.entries.forEach {
            trunkPoseSendable.addOption(it.name, it)
        }
    }

    private fun configureBindings() {
//        xboxController.a().toggleOnTrue(AutoIntake())
//        xboxController.x().toggleOnTrue(AutoShootCommand())
////        xboxController.x().onTrue(Commands.runOnce({
////            println("x button pressed")
////            cannonSystem.shoot()
////        }))
//        xboxController.b().onTrue(Commands.runOnce({
//            cannonSystem.killShooter()
//        }))
//        xboxController.y().toggleOnTrue(AutoAmp())
//        //MURDER...KILL IT ALL
//        xboxController.start().onTrue(Commands.runOnce({
//            cannonSystem.killShooter()
//            cannonSystem.killIntake()
//        }))

        rightJoystick.button(3).onTrue(Commands.runOnce({
            when (stateMachine.robotAction) {
                RobotAction.Speaker -> {
                    if (stateMachine.shootPosition == ShootPosition.AutoAim) {
                        AutoAimAndShoot()
                    } else {
                        AutoAimAndShootFromPosition(stateMachine.shootPosition.position)
                    }
                };
                RobotAction.Amp -> AutoAmp();
                RobotAction.SourceIntake -> TODO("Not yet implemented");
                RobotAction.FloorIntake -> AutoIntake()
                RobotAction.Trap -> TODO("Not yet implemented")
                //Does literally nothing
                RobotAction.Chill -> println("*Hits blunt* Yoooooooo sup bra (currently in chill mode)")
            }
        }))


        xboxController.x().onTrue(Commands.runOnce({
            stateMachine.targetTrunkPose = TrunkPose.STOW
        }))
        xboxController.back().onTrue(Commands.runOnce({
            trunkSystem.STOP()
        }))
        xboxController.y().onTrue(UnBreakTheIK())
        xboxController.b().toggleOnTrue(AutoIntake())
        xboxController.leftBumper().onTrue(AutoAimAndShootFromPosition(Pose2d(Translation2d(2.89, 5.54), Rotation2d())))
        xboxController.a().onTrue(AutoAimAndShoot())
        xboxController.rightBumper().toggleOnTrue(AutoSpit())

        leftJoystick.button(2).whileTrue(FloorIntakeAndSeek())
    }

//    val autoChooser: SendableChooser<Command> = AutoBuilder.buildAutoChooser()
//        SmartDashboard.putData("Auto Chooser", autoChooser)

}

