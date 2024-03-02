package frc.robot.constants

object CannonConstants {

    val SHOOTER_MAX_RPM = 800.0

    val OUTER_INTAKE_PERCENT: Double = .8
    val INNER_INTAKE_PERCENT: Double = .8

    val OUTER_FEED_PERCENT: Double = .8
    val INNER_FEED_PERCENT: Double = .8

    val OUTER_SPIT_PERCENT: Double = -.8
    val INNER_SPIT_PERCENT: Double = -.8

    //percent of max rpm
    val LEFT_SHOOTER_SHOOT_VELOCITY: Double = .6* SHOOTER_MAX_RPM
    val RIGHT_SHOOTER_SHOOT_VELOCITY: Double = .6 * SHOOTER_MAX_RPM

    val RIGHT_SHOOTER_PREP_VELOCITY: Double = .1 * SHOOTER_MAX_RPM
    val LEFT_SHOOTER_PREP_VELOCITY: Double = .1 * SHOOTER_MAX_RPM

    val SHOOTER_VELOCITY_DEADZONE: Double = .1

    //measure in ms
    val NOTE_EXIT_BEAMBREAK_DELAY: Double = 50.0


    val LEFT_SHOOTER_MOTOR_ID = 40
    val RIGHT_SHOOTER_MOTOR_ID = 41
    val OUTER_INTAKE_MOTOR_ID = 50
    val INNER_INTAKE_MOTOR_ID = 51

    val outerIntakePercent = .5
    val innerIntakePercent = .5

    var leftShooterKP = 0.0
    var leftShooterKI = 0.0
    var leftShooterKD = 0.0
    var leftShooterIz = 0.0
    var leftShooterFF = 0.0
    var leftShooterMax = -1.0
    var leftShooterMin = 1.0


    var rightShooterKP = 0.0
    var rightShooterKI = 0.0
    var rightShooterKD = 0.0
    var rightShooterIz = 0.0
    var rightShooterFF = 0.0
    var rightShooterMax = -1.0
    var rightShooterMin = 1.0

}
