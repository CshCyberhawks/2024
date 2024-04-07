package frc.robot.subsystems.cannon

import MiscCalculations
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.IntakeState
import frc.robot.NoteState
import frc.robot.RobotContainer
import frc.robot.ShooterState
import frc.robot.constants.CannonConstants
import frc.robot.util.Telemetry

class CannonSystem(val io: CannonIO) : SubsystemBase() {
    //Desired shooter velocities
    private var desiredShooterVel = 0.0

    //Desired intake percentages
    private var desiredOuterPercent = 0.0
    private var desiredInnerPercent = 0.0

    var noteEntryTime = -1.0

    val leftShooterPID =
        PIDController(CannonConstants.shooterKP, CannonConstants.shooterKI, CannonConstants.shooterKD)
    val rightShooterPID =
        PIDController(CannonConstants.shooterKP, CannonConstants.shooterKI, CannonConstants.shooterKD)

    private var exitBreakBeamTriggerTime: Double = -1.0

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

    fun ampShoot() {
        RobotContainer.stateMachine.shooterState = ShooterState.Amping
    }

    fun closeShoot() {
        RobotContainer.stateMachine.shooterState = ShooterState.CloseShooting
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

    //Shooter up to speed
    fun shooterReady() =
        MiscCalculations.appxEqual(
            desiredShooterVel,
            io.getRightShooterVel(),
            CannonConstants.SHOOTER_VELOCITY_DEADZONE
        ) &&
                MiscCalculations.appxEqual(
                    desiredShooterVel,
                    io.getLeftShooterVel(),
                    CannonConstants.SHOOTER_VELOCITY_DEADZONE
                )
                && desiredShooterVel != 0.0 && RobotContainer.stateMachine.shooterState == ShooterState.Shooting

    override fun periodic() {
        RobotContainer.telemetry.cannonTelemetry =
            SmartDashboard.getBoolean("Cannon Telemetry", RobotContainer.telemetry.cannonTelemetry)

//        println("stow beam break " + io.getLoadedBeamBreak())
//        println("note state " + RobotContainer.stateMachine.noteState)

        Telemetry.putBoolean("Intake Beam Break", io.getEntryBeamBreak(), RobotContainer.telemetry.cannonTelemetry)
        Telemetry.putBoolean("Stowed Beam Break", io.getLoadedBeamBreak(), RobotContainer.telemetry.cannonTelemetry)

        /*--------------------
             Beam Breaks
        -----------------------*/

        if (RobotContainer.stateMachine.shooterState == ShooterState.Shooting && RobotContainer.stateMachine.intakeState == IntakeState.Feeding && exitBreakBeamTriggerTime <= 0.0)
            exitBreakBeamTriggerTime = Timer.getFPGATimestamp()

        //Note is shot
//        if (io.getExitBeamBreak()) {
//            exitBreakBeamTriggerTime = Timer.getFPGATimestamp()
//        }

        //Note is not stored
        if (!io.getLoadedBeamBreak() && RobotContainer.stateMachine.shooterState != ShooterState.Shooting && !io.getEntryBeamBreak())
            RobotContainer.stateMachine.noteState = NoteState.Empty
        //Note is intaking
        else if (io.getEntryBeamBreak() && !io.getLoadedBeamBreak()) {
            RobotContainer.stateMachine.noteState = NoteState.Intaking
        } else if (io.getLoadedBeamBreak()) {
            RobotContainer.stateMachine.noteState = NoteState.Stored
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
        if (desiredShooterVel != RobotContainer.stateMachine.shooterState.shooterVel) {
            desiredShooterVel = RobotContainer.stateMachine.shooterState.shooterVel
//            println("shooter velocity setpoint changed")
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


        if (RobotContainer.stateMachine.shooterState != ShooterState.Stopped) {

            //Actually run the motors with the PIDs and the feed forwards
            val currentLeftVel = io.getLeftShooterVel()
            val currentRightVel = io.getRightShooterVel()

//            desiredShooterVel = SmartDashboard.getNumber("desired shooter velocity asdgerg", 0.0)
//
//            CannonConstants.shooterKP = SmartDashboard.getNumber("shooter kp", CannonConstants.shooterKP)
//            CannonConstants.shooterKI = SmartDashboard.getNumber("shooter ki", CannonConstants.shooterKI)
//            CannonConstants.shooterKD = SmartDashboard.getNumber("shooter kd", CannonConstants.shooterKD)
//
//            leftShooterPID.p = CannonConstants.shooterKP
//            leftShooterPID.i = CannonConstants.shooterKI
//            leftShooterPID.d = CannonConstants.shooterKD
//
//            rightShooterPID.p = CannonConstants.shooterKP
//            rightShooterPID.i = CannonConstants.shooterKI
//            rightShooterPID.d = CannonConstants.shooterKD
//
//            CannonConstants.shooterFFMultiplier = SmartDashboard.getNumber("shooter ff multiplier", CannonConstants.shooterFFMultiplier)

            SmartDashboard.putNumber("Shooter Velocity Desired", desiredShooterVel)
            SmartDashboard.putNumber("Shooter Velocity Current Right", currentRightVel)
            SmartDashboard.putNumber("Shooter Velocity Current Left", currentLeftVel)

            val shooterFF = desiredShooterVel * CannonConstants.shooterFFMultiplier

            val leftPIDOut = leftShooterPID.calculate(currentLeftVel, desiredShooterVel)
            val rightPIDOut = rightShooterPID.calculate(currentRightVel, desiredShooterVel)

            val leftPercent = (shooterFF + leftPIDOut) / CannonConstants.SHOOTER_MAX_RPM
            val rightPercent = (shooterFF + rightPIDOut) / CannonConstants.SHOOTER_MAX_RPM

            SmartDashboard.putNumber("left shooter pid", leftPIDOut)
            SmartDashboard.putNumber("right shooter pid", rightPIDOut)

            SmartDashboard.putNumber("left percent", leftPercent)
            SmartDashboard.putNumber("right percent", rightPercent)

            io.setLeftShooter(leftPercent)
            io.setRightShooter(rightPercent)
//        println("Cannon periodic end; percent: " + leftPercent)
        } else {
            io.setLeftShooter(0.0)
            io.setRightShooter(0.0)
        }
    }
}
