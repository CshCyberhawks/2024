package frc.robot

import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.wpilibj.Servo
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.commands.*
import frc.robot.commands.automatic.*
import frc.robot.commands.cannon.AutoShootCommand
import frc.robot.commands.cannon.AutoSpit
import frc.robot.commands.trunk.*
import frc.robot.constants.TargetingConstants
import frc.robot.constants.TrunkConstants
import frc.robot.subsystems.VisionSystem
import frc.robot.subsystems.cannon.CannonIOReal
import frc.robot.subsystems.cannon.CannonSystem
import frc.robot.subsystems.swerve.SwerveSystem
import frc.robot.subsystems.trunk.TrunkIOReal
import frc.robot.subsystems.trunk.TrunkSystem
import frc.robot.util.TargetingSystem
import frc.robot.util.TelemetryToggles
import frc.robot.util.betterToggleOnTrue

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
    val stowTypeSendable: SendableChooser<StowType> = SendableChooser<StowType>()
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

        StowType.entries.forEach {
            stowTypeSendable.addOption(it.name, it)
        }

        TrunkPose.entries.forEach {
            trunkPoseSendable.addOption(it.name, it)
        }

        stowTypeSendable.setDefaultOption("Internal", StowType.Internal)
        SmartDashboard.putData("Stow Type", stowTypeSendable)

        robotActionSendable.setDefaultOption("FloorIntake", RobotAction.FloorIntake)
        SmartDashboard.putData("Robot Action", robotActionSendable)
    }

    private fun configureBindings() {
        rightJoystick.button(3).onTrue(Commands.runOnce({
            when (stateMachine.robotAction) {
                RobotAction.Speaker -> TeleopAimTwistAndShoot().schedule()
                RobotAction.Amp -> AutoAmp().schedule();
                RobotAction.SourceIntake -> println("Tried to source intake (not yet implemented)")
                RobotAction.FloorIntake -> AutoIntake().schedule()
                RobotAction.Trap -> println("Tried to trap score (not yet implemented)")
                RobotAction.Climb -> AutoClimbCommand().schedule()
                //Does literally nothing
                RobotAction.Chill -> println("*Hits blunt* Yoooooooo sup bra (currently in chill mode)")
            }
        }))

        leftJoystick.button(3).onTrue(Commands.runOnce({
            when (stateMachine.robotAction) {
                RobotAction.Speaker -> actuallyDoShoot = true
                RobotAction.Amp -> actuallyDoAmp = true
                RobotAction.SourceIntake -> println("Tried to trigger source intake (not yet implemented)")
                RobotAction.FloorIntake -> println("nothin - action floor intake")
                RobotAction.Trap -> println("Tried to trigger trap score (not yet implemented)")
                RobotAction.Climb -> actuallyDoClimb = true
                //Does literally nothing
                RobotAction.Chill -> println("*Hits blunt* Yoooooooo sup bra (currently in chill mode)")
            }
        }))

        leftJoystick.button(4).onTrue(Commands.runOnce({
            stateMachine.limelightReset = true
        }))
        leftJoystick.button(4).onFalse(Commands.runOnce({
            stateMachine.limelightReset = false
        }))

        xboxController.leftBumper().onTrue(Commands.runOnce({
            actuallyDoShoot = true
        }))
        xboxController.start().onTrue(Commands.runOnce({
            actuallyDoAmp = true
        }))
        rightJoystick.button(4).toggleOnTrue(FloorIntakeAndSeek())

        xboxController.b().betterToggleOnTrue(AutoIntake())
        xboxController.a().onTrue(Commands.runOnce({
            stateMachine.currentTrunkCommand = GoToPoseAndHoldTrunk(TrunkPose.CalibrationAngle)
        }))
        xboxController.a().betterToggleOnTrue(AutoAmp())

        xboxController.y().betterToggleOnTrue(AutoAimAndShoot())
        xboxController.povUp().onTrue(Commands.runOnce({ TargetingConstants.endpointZ += .01 }))
        xboxController.povDown().onTrue(Commands.runOnce({ TargetingConstants.endpointZ -= .01 }))
        xboxController.x()
            .onTrue(Commands.runOnce({ stateMachine.currentTrunkCommand = StowTrunkCommand() }))
        xboxController.rightBumper().onTrue(AutoSpit())
        xboxController.leftTrigger().betterToggleOnTrue(AutoClimbCommand())
        xboxController.rightTrigger().onTrue(Commands.runOnce({
            actuallyDoClimb = true
        }))
        xboxController.back().whileTrue(KillTrunk())
//        leftJoystick.button(10).toggleOnTrue(KillTrunk())
    }

    private fun configureAutoCommands() {
        NamedCommands.registerCommand("AutoFloorIntakeAndSeek", AutoFloorIntakeAndSeek())
        NamedCommands.registerCommand("AutoIntake", AutoIntake())
        NamedCommands.registerCommand("AutoAimDumbTwistAndShoot", AutoAimDumbTwistAndShoot())
        NamedCommands.registerCommand(
            "Stow",
            Commands.runOnce({ stateMachine.currentTrunkCommand = StowTrunkCommand() })
        )
        NamedCommands.registerCommand("AutoAimAndShootPrep", AutoAimAndShootPrep())
        NamedCommands.registerCommand("CalibrateTrunk", CalibrateTrunk())
    }

//    val autoChooser: SendableChooser<Command> = AutoBuilder.buildAutoChooser()
//        SmartDashboard.putData("Auto Chooser", autoChooser)


}

