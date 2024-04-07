package frc.robot

import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.commands.AutoAmp
import frc.robot.commands.AutoIntake
import frc.robot.commands.KillTrunk
import frc.robot.commands.TeleopSwerveDriveCommand
import frc.robot.commands.automatic.AutoAimAndShoot
import frc.robot.commands.automatic.AutoAimAndShootPrep
import frc.robot.commands.automatic.AutoAimDumbTwistAndShoot
import frc.robot.commands.automatic.AutoClimbCommand
import frc.robot.commands.automatic.AutoFloorIntakeAndSeek
import frc.robot.commands.automatic.FloorIntakeAndSeek
import frc.robot.commands.automatic.TeleopAimTwistAndShoot
import frc.robot.commands.cannon.AutoSpit
import frc.robot.commands.trunk.CalibrateTrunk
import frc.robot.commands.trunk.GoToPoseAndHoldTrunk
import frc.robot.constants.TargetingConstants
import frc.robot.subsystems.VisionSystem
import frc.robot.subsystems.cannon.CannonIOReal
import frc.robot.subsystems.cannon.CannonSystem
import frc.robot.subsystems.swerve.SwerveSystem
import frc.robot.subsystems.trunk.TrunkIOReal
import frc.robot.subsystems.trunk.TrunkSystem
import frc.robot.util.ControllerUtil
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

//    val climbLatch: Servo = Servo(TrunkConstants.CLIMB_LATCH_ID)

    val autonomousCommand: Command = Commands.run({})

    var teleopSwerveCommand: Command = TeleopSwerveDriveCommand()

//    val climbCommand: Command = GoToPoseTrunk(TrunkPose.CLIMB).andThen(GoToPoseTrunk(TrunkPose.CLIMB_DOWN)).andThen({
//        //climbLatch.angle = 90.0 // tune before testing
//    }).alongWith(HoldPoseTrunk(TrunkPose.CLIMB_DOWN))

    val targetingSystem: TargetingSystem = TargetingSystem()

    val visionSystem: VisionSystem = VisionSystem()

    var actuallyDoShoot: Boolean = false
    var actuallyDoAmp: Boolean = false
    var actuallyDoClimb: Boolean = false

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
        configureAutoCommands()

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
        rightJoystick.button(3).onTrue(Commands.runOnce({
            when (stateMachine.robotAction) {
                RobotAction.Speaker -> {
                    if (stateMachine.shootPosition == ShootPosition.AutoAim) {
                        AutoAimAndShoot()
                    } else {
                        AutoAimAndShoot()
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

        xboxController.leftBumper().onTrue(Commands.runOnce({
            actuallyDoShoot = true
        }))
        xboxController.start().onTrue(Commands.runOnce({
            actuallyDoAmp = true
        }))
        rightJoystick.button(4).toggleOnTrue(FloorIntakeAndSeek())

        ControllerUtil.betterToggleOnTrue(xboxController.b(), AutoIntake())
        ControllerUtil.betterToggleOnTrue(xboxController.a(), AutoAmp())
        ControllerUtil.betterToggleOnTrue(xboxController.y(), TeleopAimTwistAndShoot())
//        xboxController.y().onTrue(AutoShootCommand())
        xboxController.povUp().onTrue(Commands.runOnce({ TargetingConstants.endpointZ += .01 }))
        xboxController.povDown().onTrue(Commands.runOnce({ TargetingConstants.endpointZ -= .01 }))
//        xboxController.b().onTrue(Commands.runOnce({
//            RobotContainer.trunkSystem.io.setServoAngle(0.0)
//        }))
//        xboxController.a().onTrue(Commands.runOnce({
//            RobotContainer.trunkSystem.io.setServoAngle(45.0)
//        }))
//        xboxController.y().onTrue(Commands.runOnce({
//            RobotContainer.trunkSystem.io.setServoAngle(90.0)
//        }))
//        xboxController.a().onTrue(Commands.runOnce({
//            stateMachine.currentTrunkCommand = GoToPoseAndHoldTrunk(TrunkPose.CalibrationAngle)
//        }))
//        xboxController.a().onTrue(AutoIntakeAndShoot())
//        val calibrationAngleCommand = HoldPoseTrunk(TrunkPose.CalibrationAngle)
//        xboxController.a()
//            .onTrue(Commands.runOnce({
//                stateMachine.currentTrunkCommand =
//                    GoToPoseTrunk(TrunkPose.CalibrationAngle).andThen(calibrationAngleCommand)
//            }))
//        xboxController.b().onTrue(Commands.runOnce({ calibrationAngleCommand.currentTargetPosition = 0.1 }))
//        xboxController.y().onTrue(Commands.runOnce({ calibrationAngleCommand.currentTargetPosition = 0.381 }))
        xboxController.x()
            .onTrue(Commands.runOnce({ stateMachine.currentTrunkCommand = GoToPoseAndHoldTrunk(TrunkPose.STOW) }))
        xboxController.rightBumper().onTrue(AutoSpit())
        ControllerUtil.betterToggleOnTrue(xboxController.leftTrigger(), AutoClimbCommand())
        xboxController.rightTrigger().onTrue(Commands.runOnce({
            actuallyDoClimb = true
        }))
        xboxController.back().whileTrue(KillTrunk())
//        xboxController.povRight().onTrue()

//        leftJoystick.button(2).whileTrue(FloorIntakeAndSeek())

        leftJoystick.button(4).onTrue(Commands.runOnce({
            stateMachine.limelightReset = true
        }))
        leftJoystick.button(4).onFalse(Commands.runOnce({
            stateMachine.limelightReset = false
        }))

//        leftJoystick.button(10).toggleOnTrue(KillTrunk())
    }

    private fun configureAutoCommands() {
        NamedCommands.registerCommand("AutoFloorIntakeAndSeek", AutoFloorIntakeAndSeek())
        NamedCommands.registerCommand("AutoIntake", AutoIntake())
        NamedCommands.registerCommand("AutoAimDumbTwistAndShoot", AutoAimDumbTwistAndShoot())
        NamedCommands.registerCommand(
            "Stow",
            Commands.runOnce({ stateMachine.currentTrunkCommand = GoToPoseAndHoldTrunk(TrunkPose.STOW) })
        )
        NamedCommands.registerCommand("AutoAimAndShootPrep", AutoAimAndShootPrep())
        NamedCommands.registerCommand("CalibrateTrunk", CalibrateTrunk())
    }

//    val autoChooser: SendableChooser<Command> = AutoBuilder.buildAutoChooser()
//        SmartDashboard.putData("Auto Chooser", autoChooser)

}

