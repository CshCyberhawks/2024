package frc.robot

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.PointWheelsAt
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveDriveBrake
import com.pathplanner.lib.commands.PathPlannerAuto
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.commands.TeleopSwerveDriveCommand
import frc.robot.commands.cannon.AutoAmp
import frc.robot.commands.cannon.AutoIntake
import frc.robot.commands.cannon.AutoShootCommand
import frc.robot.constants.TunerConstants
import frc.robot.subsystems.cannon.CannonIOReal
import frc.robot.subsystems.cannon.CannonSystem
import frc.robot.subsystems.swerve.SwerveSystem
import frc.robot.subsystems.swerve.Telemetry
import frc.robot.subsystems.trunk.TrunkIOReal
import frc.robot.subsystems.trunk.TrunkSystem
import java.io.File

object RobotContainer {
    val leftJoystick: CommandJoystick = CommandJoystick(0)
    val rightJoystick: CommandJoystick = CommandJoystick(1)
    val xboxController: CommandXboxController = CommandXboxController(2)

    private val MaxSpeed: Double = TunerConstants.kSpeedAt12VoltsMps // kSpeedAt12VoltsMps desired top speed
    private val MaxAngularRate = 1.5 * Math.PI // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */ //  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick

    // driving in open loop
    private val brake = SwerveDriveBrake()
    private val point = PointWheelsAt()
    private val logger: Telemetry = Telemetry(MaxSpeed)

    val trunkSystem = TrunkSystem(TrunkIOReal())

    val stateMachine: RobotStateMachine = RobotStateMachine()

    val cannonSystem: CannonSystem = CannonSystem(CannonIOReal())

    var teleopSwerveCommand: Command = TeleopSwerveDriveCommand()

    val intakeLimelight: String = "limelight-intake"

    val autoStateManagementEnableButton: Boolean
        get() = SmartDashboard.getBoolean("Enable Automatic State Management", false)

    val robotActionSendable: SendableChooser<RobotAction> = SendableChooser<RobotAction>()

    private val autos: SendableChooser<String> = SendableChooser<String>()

    val swerveSystem: SwerveSystem = SwerveSystem()

//    private var runAuto: Command? = PathPlannerAuto("OneNote")

    init {
        configureBindings()

        for (file in File(Filesystem.getDeployDirectory(), "pathplanner/autos").walkTopDown()) {
            autos.addOption(
                file.nameWithoutExtension,
                file.nameWithoutExtension
            )
        }
        SmartDashboard.putData("Autos", autos)

        RobotAction.entries.forEach {
            robotActionSendable.addOption(it.name, it)
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
                RobotAction.Speaker -> AutoShootCommand();
                RobotAction.Amp -> AutoAmp();
                RobotAction.SourceIntake -> TODO("Not yet implemented");
                RobotAction.FloorIntake -> AutoIntake()
                RobotAction.Trap -> TODO("Not yet implemented")
                //Does literally nothing
                RobotAction.Chill -> println("*Hits blunt* Yoooooooo sup bra (currently in chill mode)")
            }
        }))

        xboxController.a().onTrue(Commands.runOnce({
            RobotContainer.stateMachine.targetTrunkPose = TrunkPosition.STOW
        }))
        xboxController.b().onTrue(Commands.runOnce({
            RobotContainer.stateMachine.targetTrunkPose = TrunkPosition.INTAKE
        }))
        xboxController.x().onTrue(Commands.runOnce({
            RobotContainer.trunkSystem.calibrate()
        }))
        xboxController.back().onTrue(Commands.runOnce({
            RobotContainer.trunkSystem.STOP()
        }))
        xboxController.start().onTrue(Commands.runOnce({
            println("pressed start")
            RobotContainer.trunkSystem.goManual()
        }))
        xboxController.y().onTrue(Commands.runOnce({
            println("pressed y")
            RobotContainer.trunkSystem.goToCustom()
        }))
    }

    //    fun getAutonomousCommand(): Command? = runAuto
    fun getAutonomousCommand(): Command = try {
        PathPlannerAuto(autos.selected)
    } catch (e: Exception) {
        Commands.runOnce({ System.err.println(e) })
    }
}

