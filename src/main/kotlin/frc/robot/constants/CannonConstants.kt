package frc.robot.constants

object CannonConstants {

    val SHOOTER_MAX_RPM = 7000.0
    val INTAKE_STOW_DELAY = .3

    val OUTER_INTAKE_PERCENT: Double = -.8
    val INNER_INTAKE_PERCENT: Double = .8

    val OUTER_FEED_PERCENT: Double = -.8
    val INNER_FEED_PERCENT: Double = .8

    val OUTER_SPIT_PERCENT: Double = .8
    val INNER_SPIT_PERCENT: Double = -.8

    val OUTER_AMP_PERCENT: Double = -.8
    val INNER_AMP_PERCENT: Double = -.8

    //percent of max rpm
//    val LEFT_SHOOTER_SHOOT_VELOCITY: Double = .8 * SHOOTER_MAX_RPM
//    val RIGHT_SHOOTER_SHOOT_VELOCITY: Double = .8 * SHOOTER_MAX_RPM
    val LEFT_SHOOTER_SHOOT_VELOCITY: Double = 4010.2032905044503

//        val LEFT_SHOOTER_SHOOT_VELOCITY: Double = 3350.0
//    val RIGHT_SHOOTER_SHOOT_VELOCITY: Double = 3350.0

    val RIGHT_SHOOTER_SHOOT_VELOCITY: Double = 4010.2032905044503

    val RIGHT_SHOOTER_PREP_VELOCITY: Double = .8 * RIGHT_SHOOTER_SHOOT_VELOCITY
    val LEFT_SHOOTER_PREP_VELOCITY: Double = .8 * LEFT_SHOOTER_SHOOT_VELOCITY

    val SHOOTER_VELOCITY_DEADZONE: Double = 150.0

    //measure in ms
    val NOTE_EXIT_BEAMBREAK_DELAY: Double = 1.0

    val LEFT_SHOOTER_MOTOR_ID = 40
    val RIGHT_SHOOTER_MOTOR_ID = 41
    val OUTER_INTAKE_MOTOR_ID = 50
    val INNER_INTAKE_MOTOR_ID = 51

    val outerIntakePercent = .5
    val innerIntakePercent = .5

    var leftShooterKP = 1.2
    var leftShooterKI = 0.0
    var leftShooterKD = 0.0
    var leftShooterIz = 0.0
    var leftShooterFF = 0.0
    var leftShooterMax = -1.0
    var leftShooterMin = 1.0

    var rightShooterKP = 1.2
    var rightShooterKI = 0.0
    var rightShooterKD = 0.0
    var rightShooterIz = 0.0
    var rightShooterFF = 0.0
    var rightShooterMax = -1.0
    var rightShooterMin = 1.0

}
