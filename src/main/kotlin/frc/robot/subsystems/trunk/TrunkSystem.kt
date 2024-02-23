package frc.robot.subsystems

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.TrunkConstants
import frc.robot.subsystems.trunk.TrunkIO
import frc.robot.util.visualization.Mechanism2d
import frc.robot.util.visualization.MechanismLigament2d
import kotlin.math.sqrt

enum class TrunkPosition {
    AMP,
    SPEAKER,
    INTAKE,
    STOW,
}

class TrunkSystem(private val io: TrunkIO) : SubsystemBase() {
    var targetPosition = TrunkPosition.SPEAKER

    var rotationSetPoint = TrunkConstants.TARGET_SAFE_ANGLE
    var positionSetPoint = TrunkConstants.SPEAKER_POSITION

    var shootingAngle = TrunkConstants.TARGET_SAFE_ANGLE
    var isDefinitelyAboveCrossbar = false

    val superstructureMechanism = Mechanism2d(1.0, 1.0)
    val elevatorMechanismRoot = superstructureMechanism.getRoot("Elevator Root", 0.0, 0.0)
    val trunkMechanismRoot = superstructureMechanism.getRoot("Trunk Root", io.getPosition(), io.getPosition())
    val trunkMechanism = trunkMechanismRoot.append(MechanismLigament2d("Trunk", -.25, io.getRotation(), color = Color8Bit(0, 0, 255)))

    init {
        elevatorMechanismRoot.append(MechanismLigament2d("Elevator", sqrt(2.0), 45.0))
    }

    fun setZeroPosition() {

    }

    fun goToIntake() {
        targetPosition = TrunkPosition.INTAKE
    }

    fun goToAmp() {
        targetPosition = TrunkPosition.AMP
    }

    fun setDesiredPosition(position: Double) {
        positionSetPoint = position
        io.setDesiredPosition(position)
    }

    fun setDesiredRotation(angle: Double) {
        rotationSetPoint = angle + TrunkConstants.rotationOffset
        io.setDesiredRotation(angle)
    }

    fun goToShoot(angle: Double) {
        shootingAngle = angle
        targetPosition = TrunkPosition.SPEAKER
    }

    fun shoot(angle: Double, leftPower: Double, rightPower: Double) {
        goToShoot(angle)
        io.setShootSpeed(leftPower, rightPower)
    }


//    fun elevate(speed: Double) {
//        elevatorMotor.set(speed)
//    }
//
//    fun rotate(speed: Double) {
//        mainRotationMotor.set(speed)
//    }

    val keyboard = GenericHID(0)

    override fun periodic() {
        trunkMechanismRoot.setPosition(io.getPosition(), io.getPosition())
        trunkMechanism.angle = io.getRotation()

        SmartDashboard.putData("Trunk Mechanism", superstructureMechanism)
        SmartDashboard.putString("Trunk Position", targetPosition.name)

//        // TODO: Remove simulation test code. Anything that sets position and rotation will be reflected in the sim. I don't really want to deal with Desmond's code 
//        io.setDesiredPosition(io.getPosition() + 0.001)
//        io.setDesiredRotation(io.getRotation() + 5.0)

        // TODO: Remove for actual robot
        if (keyboard.getRawButton(1)) {
            targetPosition = TrunkPosition.STOW
        }
        if (keyboard.getRawButton(2)) {
            targetPosition = TrunkPosition.INTAKE
        }
        if (keyboard.getRawButton(3)) {
            targetPosition = TrunkPosition.SPEAKER
        }
        if (keyboard.getRawButton(4)) {
            targetPosition = TrunkPosition.AMP
        }

        when (targetPosition) {
            TrunkPosition.AMP -> {
                if (rotationSetPoint != TrunkConstants.AMP_ANGLE)
                    setDesiredRotation(TrunkConstants.AMP_ANGLE)
                if (positionSetPoint != TrunkConstants.AMP_POSITION) {
                    if (isDefinitelyAboveCrossbar || io.getPosition() > TrunkConstants.CROSSBAR_TOP) {
                        isDefinitelyAboveCrossbar = true
                        setDesiredPosition(TrunkConstants.AMP_POSITION)
                    } else if (io.getRotation() > TrunkConstants.MIN_SAFE_ANGLE)
                        setDesiredPosition(TrunkConstants.AMP_POSITION)
                }
            }

            TrunkPosition.INTAKE -> {
                isDefinitelyAboveCrossbar = false
                if (rotationSetPoint != TrunkConstants.INTAKE_ANGLE) {
                    if (io.getPosition() < TrunkConstants.CROSSBAR_BOTTOM)
                        setDesiredRotation(TrunkConstants.INTAKE_ANGLE)
                    else if (rotationSetPoint != TrunkConstants.TARGET_SAFE_ANGLE)
                        setDesiredRotation(TrunkConstants.TARGET_SAFE_ANGLE)
                }
                if (positionSetPoint != TrunkConstants.INTAKE_POSITION) {
                    if (io.getRotation() > TrunkConstants.MIN_SAFE_ANGLE)
                        setDesiredPosition(TrunkConstants.INTAKE_POSITION)
                    else if (positionSetPoint != TrunkConstants.CROSSBAR_TOP)
                        setDesiredPosition(TrunkConstants.CROSSBAR_TOP)
                }
            }

            TrunkPosition.SPEAKER -> {
                if (rotationSetPoint != shootingAngle) {
                    if (isDefinitelyAboveCrossbar || io.getPosition() > TrunkConstants.CROSSBAR_TOP) {
                        isDefinitelyAboveCrossbar = true
                        setDesiredRotation(shootingAngle) // maybe no
                    } else if (rotationSetPoint != TrunkConstants.TARGET_SAFE_ANGLE)
                        setDesiredRotation(TrunkConstants.TARGET_SAFE_ANGLE)
                }
                if (positionSetPoint != TrunkConstants.SPEAKER_POSITION) {
                    if (isDefinitelyAboveCrossbar || io.getRotation() > TrunkConstants.MIN_SAFE_ANGLE)
                        setDesiredPosition(TrunkConstants.SPEAKER_POSITION)
                    else
                        setDesiredPosition(TrunkConstants.CROSSBAR_BOTTOM)
                }
            }

            TrunkPosition.STOW -> {
                if (rotationSetPoint != TrunkConstants.STOW_ANGLE) {
                    if (isDefinitelyAboveCrossbar || io.getPosition() > TrunkConstants.CROSSBAR_TOP) {
                        isDefinitelyAboveCrossbar = true
                        setDesiredRotation(TrunkConstants.STOW_ANGLE) // maybe no
                    } else if (rotationSetPoint != TrunkConstants.TARGET_SAFE_ANGLE)
                        setDesiredRotation(TrunkConstants.TARGET_SAFE_ANGLE)
                }
                if (positionSetPoint != TrunkConstants.STOW_POSITION) {
                    if (isDefinitelyAboveCrossbar || io.getRotation() > TrunkConstants.MIN_SAFE_ANGLE)
                        setDesiredPosition(TrunkConstants.STOW_POSITION)
                    else
                        setDesiredPosition(TrunkConstants.CROSSBAR_BOTTOM)
                }
            }
        }

        io.periodic()
    }


    override fun simulationPeriodic() {

    }
}