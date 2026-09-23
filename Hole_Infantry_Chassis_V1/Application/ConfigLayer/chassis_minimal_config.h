#ifndef CHASSIS_MINIMAL_CONFIG_H
#define CHASSIS_MINIMAL_CONFIG_H
/* All speed limits are MOTOR ROTOR rpm (C620 feedback bytes 2..3).
 * Existing RM driver: output-shaft rad/s = rotor rpm * 2*pi/(60*19).
 * Keep the team hard ceiling; change only the default for initial tests. */
#define CHASSIS_HARD_MAX_RPM       5000
#define CHASSIS_DEFAULT_MAX_RPM    1500
#define CHASSIS_RC_DEADBAND        20
#define CHASSIS_RC_TIMEOUT_MS      60U
#define CHASSIS_MOTOR_TIMEOUT_MS   100U
#define CHASSIS_PID_KP             1.0f
#define CHASSIS_PID_KI             0.0f
#define CHASSIS_PID_KD             0.0f
#define CHASSIS_PID_INTEGRAL_MAX   0.0f
#define CHASSIS_TORQUE_LIMIT_NM    1.5f /* 5 A, raw current 4096 */
#define CHASSIS_FRONT_RC_SIGN      (-1.0f) /* original -ch3 */
#define CHASSIS_LEFT_RC_SIGN       (+1.0f) /* original +ch2 */
/* LF, LB, RF, RB: feedback ID AND 0x200 transmit slot derive from here. */
#define CHASSIS_LF_ESC_ID          1
#define CHASSIS_LB_ESC_ID          2
#define CHASSIS_RF_ESC_ID          3
#define CHASSIS_RB_ESC_ID          4
/* Change a sign here only if the physical installation differs.
 * These flip kinematic targets, NEVER just the feedback in the PID. */
#define CHASSIS_LF_SIGN            (+1.0f)
#define CHASSIS_LB_SIGN            (+1.0f)
#define CHASSIS_RF_SIGN            (+1.0f)
#define CHASSIS_RB_SIGN            (+1.0f)
#if CHASSIS_HARD_MAX_RPM > 5000 || CHASSIS_HARD_MAX_RPM <= 0
#error Invalid team speed ceiling
#endif
#if CHASSIS_DEFAULT_MAX_RPM > CHASSIS_HARD_MAX_RPM || CHASSIS_DEFAULT_MAX_RPM <= 0
#error Invalid initial speed limit
#endif
#if CHASSIS_LF_ESC_ID < 1 || CHASSIS_LF_ESC_ID > 4 || CHASSIS_LB_ESC_ID < 1 || CHASSIS_LB_ESC_ID > 4 || CHASSIS_RF_ESC_ID < 1 || CHASSIS_RF_ESC_ID > 4 || CHASSIS_RB_ESC_ID < 1 || CHASSIS_RB_ESC_ID > 4
#error ESC IDs must be in 1..4
#endif
#if CHASSIS_LF_ESC_ID == CHASSIS_LB_ESC_ID || CHASSIS_LF_ESC_ID == CHASSIS_RF_ESC_ID || CHASSIS_LF_ESC_ID == CHASSIS_RB_ESC_ID || CHASSIS_LB_ESC_ID == CHASSIS_RF_ESC_ID || CHASSIS_LB_ESC_ID == CHASSIS_RB_ESC_ID || CHASSIS_RF_ESC_ID == CHASSIS_RB_ESC_ID
#error Duplicate ESC IDs
#endif
#endif
