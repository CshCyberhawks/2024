package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandJoystick
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
//import frc.robot.commands.TeleopSwerveDriveCommand
import frc.robot.subsystems.GUNSystem
import frc.robot.subsystems.SwerveSystem
//import frc.robot.subsystems.SwerveSystemIOReal
import java.io.File

object RobotContainer {
    val leftJoystick: CommandJoystick = CommandJoystick(0)
    val rightJoystick: CommandJoystick = CommandJoystick(1)
    private val xboxController: CommandXboxController = CommandXboxController(2)

//    val swerveSystem: SwerveSystem
    val gunSystem = GUNSystem()

    val autonomousCommand: Command = Commands.run({})
  
    lateinit var teleopSwerveCommand: Command
    lateinit var teleopElevateCommand: Command
    lateinit var teleopRotateCommand: Command
    lateinit var teleopShootCommand: Command

    val autoChooser: SendableChooser<Command> = AutoBuilder.buildAutoChooser()

    init {
//        when (Constants.currentMode) {
//            Constants.Mode.REAL -> {
//                swerveSystem = SwerveSystem(
//                    SwerveSystemIOReal(),
//                    File(Filesystem.getDeployDirectory(), "yagsl_configs/slippy")
//                )
//            }
//            Constants.Mode.SIM -> {
//                // change these later
//                swerveSystem = SwerveSystem(
//                    SwerveSystemIOReal(),
//                    File(Filesystem.getDeployDirectory(), "yagsl_configs/slippy")
//                )
//            }
//            Constants.Mode.REPLAY -> {
//                // change these later
//                swerveSystem = SwerveSystem(
//                    SwerveSystemIOReal(),
//                    File(Filesystem.getDeployDirectory(), "yagsl_configs/slippy")
//                )
//            }
//        }
        SmartDashboard.putData("Auto Chooser", autoChooser)

//        teleopSwerveCommand = TeleopSwerveDriveCommand()
//        teleopSwerveCommand.schedule()

        configureButtonBindings()

        SmartDashboard.putData("Auto Chooser", autoChooser)
    }

    private fun configureButtonBindings() {
        teleopShootCommand = Commands.run({
            gunSystem.shoot(0.0, xboxController.leftY, xboxController.rightY)
        })
//        teleopElevateCommand = Commands.run({
//            gunSystem.elevate(xboxController.leftY)
//        })
//        teleopRotateCommand = Commands.run({ gunSystem.rotate(xboxController.rightY) })
    }
}