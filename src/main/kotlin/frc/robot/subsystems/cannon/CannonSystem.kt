package frc.robot.subsystems.cannon

import MiscCalculations
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.IntakeState
import frc.robot.NoteState
import frc.robot.RobotContainer
import frc.robot.ShooterState
import frc.robot.constants.CannonConstants
import frc.robot.util.Telemetry

class CannonSystem(private val io: CannonIO) : SubsystemBase() {
    //Desired shooter velocities
    private var desiredLeftVel = 0.0
    private var desiredRightVel = 0.0

    //Desired intake percentages
    private var desiredOuterPercent = 0.0
    private var desiredInnerPercent = 0.0

    var noteEntryTime = -1.0

    val leftShooterPID =
            PIDController(CannonConstants.leftShooterKP, CannonConstants.leftShooterKI, CannonConstants.leftShooterKD)
    val rightShooterPID =
            PIDController(CannonConstants.leftShooterKP, CannonConstants.leftShooterKI, CannonConstants.leftShooterKD)


    private var exitBreakBeamTriggerTime: Double = -1.0; //

    init {
        SmartDashboard.putBoolean("Cannon Telemetry", RobotContainer.telemetry.cannonTelemetry)

    }


    fun killShooter() {
        io.setLeftShooter(0.0)
        io.setRightShooter(0.0)
        RobotContainer.stateMachine.shooterState = ShooterState.Stopped
    }

    fun prep() {
        RobotContainer.stateMachine.shooterState = ShooterState.Prepped;
    }

    fun shoot() {
        RobotContainer.stateMachine.shooterState = ShooterState.Shooting;
        println("called cannonsystem.shoot")

    }


    fun ampSpit() {
        RobotContainer.stateMachine.intakeState = IntakeState.AmpSpitting
    }


    fun intake() {
        RobotContainer.stateMachine.intakeState = IntakeState.Intaking
    }

    fun spit() {
        RobotContainer.stateMachine.intakeState = IntakeState.Spitting
    }

    fun feed() {
        RobotContainer.stateMachine.intakeState = IntakeState.Feeding
    }

    fun killIntake() {
        RobotContainer.stateMachine.intakeState = IntakeState.Stopped
    }

    fun shooterReady(): Boolean {
        //Shooter up to speed
//        return MiscCalculations.appxEqual(
//            desiredLeftVel,
//            io.getLeftShooterVel(),
//            CannonConstants.SHOOTER_VELOCITY_DEADZONE
//        ) && MiscCalculations.appxEqual(
//            desiredRightVel,
//            io.getRightShooterVel(),
//            CannonConstants.SHOOTER_VELOCITY_DEADZONE
//        )
        if (MiscCalculations.appxEqual(desiredLeftVel, io.getLeftShooterVel(), CannonConstants.SHOOTER_VELOCITY_DEADZONE)
                && desiredLeftVel != 0.0 && desiredRightVel != 0.0 && RobotContainer.stateMachine.shooterState == ShooterState.Shooting) {
            println("shooter ready")
            return true
        }
//        else if (MiscCalculations.appxEqual(desiredLeftVel, io.getLeftShooterVel(), CannonConstants.SHOOTER_VELOCITY_DEADZONE) && !MiscCalculations.appxEqual(desiredRightVel, io.getRightShooterVel(), CannonConstants.SHOOTER_VELOCITY_DEADZONE)) {
//            println("left ready but not right")
//            return false
//        }
//        else if (desiredLeftVel == 0.0 || desiredRightVel == 0.0) {
//            println("desired vel is 0")
//            return false
//        }
//        println("shooter not ready")
        return false
    }

    override fun periodic() {
        RobotContainer.telemetry.cannonTelemetry = SmartDashboard.getBoolean("Cannon Telemetry", RobotContainer.telemetry.cannonTelemetry)

//        Telemetry.putBoolean("Stow Beam Break", io.getLoadedBeamBreak(), RobotContainer.telemetry.cannonTelemetry)
        Telemetry.putNumber("Current left Cannon Speed", io.getLeftShooterVel(), RobotContainer.telemetry.cannonTelemetry)
        Telemetry.putNumber("Desired left speed", desiredLeftVel, RobotContainer.telemetry.cannonTelemetry)
        Telemetry.putNumber("Current right cannon speed", io.getRightShooterVel(), RobotContainer.telemetry.cannonTelemetry)
//        Telemetry.putNumber("Left PID setpoint", leftShooterPID.setpoint, RobotContainer.telemetry.cannonTelemetry)

//        println("stow beam break " + io.getLoadedBeamBreak())
//        println("note state " + RobotContainer.stateMachine.noteState)

        Telemetry.putBoolean("Intake Beam Break", io.getEntryBeamBreak(), RobotContainer.telemetry.cannonTelemetry)
        Telemetry.putBoolean("Stowed Beam Break", io.getLoadedBeamBreak(), RobotContainer.telemetry.cannonTelemetry)

        /*--------------------
             Beam Breaks
        -----------------------*/

        if (RobotContainer.stateMachine.shooterState == ShooterState.Shooting && RobotContainer.stateMachine.intakeState == IntakeState.Feeding && exitBreakBeamTriggerTime <= 0.0) {
            exitBreakBeamTriggerTime = Timer.getFPGATimestamp()
        }

        //Note is shot
//        if (io.getExitBeamBreak()) {
//            exitBreakBeamTriggerTime = Timer.getFPGATimestamp()
//        }
        //Note is stored

        if (noteEntryTime > 0.0) {
            SmartDashboard.putNumber("entry time - time now", Timer.getFPGATimestamp() - noteEntryTime)
        }
        if (io.getLoadedBeamBreak() && (Timer.getFPGATimestamp() - noteEntryTime) >= CannonConstants.INTAKE_STOW_DELAY) {
            noteEntryTime = -1.0
            RobotContainer.stateMachine.noteState = NoteState.Stored
        } else if (io.getLoadedBeamBreak() && noteEntryTime <= -1.0) {
            RobotContainer.stateMachine.noteState = NoteState.Stored
        }
        //Note is not stored
        else if (!io.getLoadedBeamBreak() && RobotContainer.stateMachine.shooterState != ShooterState.Shooting && !io.getEntryBeamBreak()) {
            RobotContainer.stateMachine.noteState = NoteState.Empty
        }
        //Note is intaking
        else if (io.getEntryBeamBreak() && !io.getLoadedBeamBreak() && RobotContainer.stateMachine.intakeState != IntakeState.Spitting && RobotContainer.stateMachine.intakeState != IntakeState.AmpSpitting) {
            RobotContainer.stateMachine.noteState = NoteState.Intaking;
            noteEntryTime = Timer.getFPGATimestamp()
            println("entry beam break broken second if stat")
        } else if (io.getEntryBeamBreak() && !io.getLoadedBeamBreak()) {
            RobotContainer.stateMachine.noteState = NoteState.Intaking;
        }


        //Note is shot delay handling
        if (exitBreakBeamTriggerTime > 0.0) {
            SmartDashboard.putNumber("time since began shot", Timer.getFPGATimestamp() - exitBreakBeamTriggerTime)
            if (Timer.getFPGATimestamp() - exitBreakBeamTriggerTime >= CannonConstants.NOTE_EXIT_BEAMBREAK_DELAY) {
                RobotContainer.stateMachine.noteState = NoteState.Empty;
                exitBreakBeamTriggerTime = -1.0;
            }
        }

        /*-----------------
            Shooter
           ---------------- */
        if (desiredLeftVel != RobotContainer.stateMachine.shooterState.leftVel || desiredRightVel != RobotContainer.stateMachine.shooterState.rightVel) {
            desiredLeftVel = RobotContainer.stateMachine.shooterState.leftVel
            desiredRightVel = RobotContainer.stateMachine.shooterState.rightVel

            println("set the shooter PID")
        }

        /*----------------
            Intake
         ------------------*/
        if (desiredInnerPercent != RobotContainer.stateMachine.intakeState.innerPercent || desiredOuterPercent != RobotContainer.stateMachine.intakeState.outerPercent) {
            desiredInnerPercent = RobotContainer.stateMachine.intakeState.innerPercent
            desiredOuterPercent = RobotContainer.stateMachine.intakeState.outerPercent

            io.setInnerIntakePercent(desiredInnerPercent)
            io.setOuterIntakePercent(desiredOuterPercent)

//            println("set inner and outer percents")
        }


        //Actually run the motors with the PIDs and the feed forwards
        val currentLeftVelo = io.getLeftShooterVel()
        val currentRightVelo = io.getRightShooterVel()

        val leftFF = (desiredLeftVel) * 1.0
        val rightFF = (desiredRightVel) * 1.0

        val leftPIDOut = leftShooterPID.calculate(currentLeftVelo, desiredLeftVel)
        val rightPIDOut = rightShooterPID.calculate(currentRightVelo, desiredRightVel)

//        println("currentLeftVelo " + io.getLeftShooterVel())
//        println("left ff" + leftFF)
//        println("left pid out" + leftPIDOut)

        val leftPercent = (leftFF + leftPIDOut) / CannonConstants.SHOOTER_MAX_RPM
        val rightPercent = (rightFF + rightPIDOut) / CannonConstants.SHOOTER_MAX_RPM

        SmartDashboard.putNumber("left shooter pid", leftPIDOut)
//        SmartDashboard.putNumber("left shooter percent", leftPercent)

//        Telemetry.putNumber("right percent", rightPercent, RobotContainer.telemetry.cannonTelemetry)

        io.setLeftShooter(leftPercent)
        io.setRightShooter(rightPercent)
//        println("Cannon periodic end; percent: " + leftPercent)

    }
}