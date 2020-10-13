/********************************************************************
 * Kyle Leong
 * 123456789
 * 
 * TI RSLK code for ECE 3 Spring 2020.
 *******************************************************************/

#include <ECE3.h>

/*
 * Utility data types (to prevent repetitive and tedious typing), and
 * utility functions to increase readability.
 */
#define u16 uint16_t
#define u32 uint32_t
#define i32 int32_t
#define NUM_SENSORS (8)
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define IN(lo, hi, x) (((lo) < x && x < (hi)) ? 1 : 0)

/*
 * Macro used to conditionally compile debug statements for Serial.
 * To disable the debug statements and thus increase performance,
 * comment out the '#define DEBUG' statement.
 */

// #define DEBUG

#ifdef DEBUG
#define DBG(x) do { Serial.print(x); } while (0)
#else
#define DBG(x)
#endif

/*
 * Global flag as to whether we should allow the robot to do a donut.
 * Allowing it to do the donut makes it hard to debug via Serial
 * because the main board is always on, so its always recording/
 * updating the global state.
 */
#define ENABLE_DONUT

enum Configuration {
        /* Taken from the PIN chart on CCLE. */
        LEFT_NSLP_PIN = 31,
        LEFT_DIR_PIN = 29,
        LEFT_PWM_PIN = 40,
        RIGHT_NSLP_PIN = 11,
        RIGHT_DIR_PIN = 30,
        RIGHT_PWM_PIN = 39,

        /*
         * These constants were calibrated experimentally by trying values
         * that would allow the car to run in as straight a line as possible.
         */
        INIT_LEFT_WHEEL_SPEED = 30,
        INIT_RIGHT_WHEEL_SPEED = 30,

        /* Determined experimentally, what values seem to be "centered". */
        ACCEPTABLE_ERROR = 15000,

        /* How fast should the robot turn while in the donut state? */
        DONUT_LEFT_WHEEL_SPEED = 75,
        DONUT_RIGHT_WHEEL_SPEED = 75,

        /*
         * What value of all the raw sensor data summed together should we
         * take as being at the solid black (i.e. turnaround) line?
         * Determined experimentally by placing a robot over the black
         * line and recording the value of each of the IR sensors.
         */
        TURNAROUND_THRESHOLD = 18000,

        /* How long should the donut remain in the donut state, in ms. */
        DONUT_TIME_MS = 750,

        /* LED pin numbers. */
        FRONT_RIGHT_LED = 41,
        FRONT_LEFT_LED = 51,
};

/*
 * The robot is made stateful so that it can successfully do a donut upon
 * encountering the first solid black bar, and stop on the second encounter.
 */
enum RobotState {
        NORMAL,
        DONUT,
        STOP
};

/*
 * These three arrays essentially recreate the sensor fusion algorithm that I
 * used in the spreadsheet to create a calibration for mappings of sensor values
 * to actual distance from the center of the line.
 */
static const u16 g_sensor_zero_pts[] = {
        666, 643, 596, 549, 619, 619, 619, 738
};
static const double g_norm_coeffs[] = {
        0.5934718, 0.7686395, 0.9000900, 1.1764706,
        1.0319917, 0.7824726, 0.7824726, 0.6734007
};
static const i32 g_weights[] = { -4, -3, -2, -1, 1, 2, 3, 4 };

/*
 * Function to calculate the distance from the line in millimeters given the
 * raw values from each of the eight sensors.
 */
double dist_from_line(u16* raw_data)
{
        i32 i;
        double massaged[NUM_SENSORS], sop = 0;

        /* Recreating the calculations done on the Excel spreadsheet. */
        for (i = 0; i < NUM_SENSORS; i++) {
                massaged[i] = (double) raw_data[NUM_SENSORS - 1 - i];
                massaged[i] = MAX(0, massaged[i] - g_sensor_zero_pts[i]);
                massaged[i] *= g_norm_coeffs[i];
                sop += (double) g_weights[i] * massaged[i];
        }

        /*
         * Because I took sensor measurements with the car in the opposite
         * orientation the signs of the results are flipped. We want positive
         * to be right, negative to be left, thus add an extra negation.
         */
        return -(178.95 * sop - 5085.4);
}

void setup()
{
        /* Initialize the library that collects sensor data. */
        ECE3_Init();

        pinMode(LEFT_NSLP_PIN, OUTPUT);
        pinMode(LEFT_DIR_PIN, OUTPUT);
        pinMode(LEFT_PWM_PIN, OUTPUT);

        pinMode(RIGHT_NSLP_PIN, OUTPUT);
        pinMode(RIGHT_DIR_PIN, OUTPUT);
        pinMode(RIGHT_PWM_PIN, OUTPUT);

        digitalWrite(LEFT_DIR_PIN, LOW);
        digitalWrite(LEFT_NSLP_PIN, HIGH);

        digitalWrite(RIGHT_DIR_PIN, LOW);
        digitalWrite(RIGHT_NSLP_PIN, HIGH);

        pinMode(FRONT_RIGHT_LED, OUTPUT);
        pinMode(FRONT_LEFT_LED, OUTPUT);

        /* Delay otherwise will obtain garbage values in the serial monitor. */
        Serial.begin(9600);
        delay(2000);
}

void loop()
{
        /*
         * PID Constants. Since they are not integers, we cannot put them
         * in the Configuration enum (which would look cleaner).
         */
        const static double KP = 0.00010;

        u16 raw_data[NUM_SENSORS];
        double error, cur_time;
        static double error_rate = 0;
        static double last_error = 0;
        static u32 last_time = 0;
        static u16 left_wheel_speed = INIT_LEFT_WHEEL_SPEED;
        static u16 right_wheel_speed = INIT_RIGHT_WHEEL_SPEED;
        static int state = NORMAL;
        int sum_of_sensor_values = 0;
        static int donut_start_time = 0;
        static int num_turnarounds_encountered = 0;

        /*
         * ERROR DATA COLLECTION
         *
         * Experimentally, the error is between +/- 600000.
         * Adjust constants K_p, K_d, ACCEPTABLE_ERROR accordingly.
         */
        cur_time = millis();
        ECE3_read_IR(raw_data);
        error = dist_from_line(raw_data);
        for (int i = 0; i < NUM_SENSORS; i++) {
                sum_of_sensor_values += raw_data[i];
        }

#ifdef ENABLE_DONUT
        if (sum_of_sensor_values > TURNAROUND_THRESHOLD && state == NORMAL) {
                num_turnarounds_encountered++;
                donut_start_time = millis();
                state = DONUT;

                /* We're back at the start, stop the RSLK. */
                if (num_turnarounds_encountered == 2) {
                        state = STOP;
                }
        }
#endif

        if (state == DONUT) {
                digitalWrite(FRONT_RIGHT_LED, HIGH);
                digitalWrite(FRONT_LEFT_LED, HIGH);

                /* Arbitrarily chose right wheel to be reversed. */
                digitalWrite(RIGHT_DIR_PIN, HIGH);
                left_wheel_speed = DONUT_LEFT_WHEEL_SPEED;
                right_wheel_speed = DONUT_RIGHT_WHEEL_SPEED;

                if ((millis() - donut_start_time) > DONUT_TIME_MS) {
                        /* Make it go forward again after not in donut. */
                        digitalWrite(RIGHT_DIR_PIN, LOW);
                        state = NORMAL;
                        donut_start_time = 0;
                }
        } else if (state == STOP) {
                analogWrite(LEFT_PWM_PIN, 0);
                analogWrite(RIGHT_PWM_PIN, 0);
                return;
        } else if (state == NORMAL) {
                /*
                * PROPORTIONAL CONTROLLER
                *
                * error < 0 : The car is to the left of the line, move right.
                * error > 0 : The car is to the right of the line, move left.
                *
                * If the error is within +/- ACCEPTABLE_ERROR, then we are
                * "centered", do not move until we are out of that range.
                */
                if (IN(-ACCEPTABLE_ERROR, ACCEPTABLE_ERROR, error)) {
                        DBG("Move: STRAIGHT\t");
                        left_wheel_speed = INIT_LEFT_WHEEL_SPEED;
                        right_wheel_speed = INIT_RIGHT_WHEEL_SPEED;
                } else if (error > 0) {
                        digitalWrite(FRONT_RIGHT_LED, LOW);
                        digitalWrite(FRONT_LEFT_LED, HIGH);
                        DBG("Move: LEFT\t");
                        left_wheel_speed = INIT_LEFT_WHEEL_SPEED;
                        right_wheel_speed = INIT_RIGHT_WHEEL_SPEED + KP * error;
                } else if (error < 0) {
                        digitalWrite(FRONT_RIGHT_LED, HIGH);
                        digitalWrite(FRONT_LEFT_LED, LOW);
                        DBG("Move: RIGHT\t");
                        /* The -1 is present because error < 0. Correct it. */
                        left_wheel_speed = INIT_LEFT_WHEEL_SPEED +
                                KP * error * -1;
                        right_wheel_speed = INIT_RIGHT_WHEEL_SPEED;
                } else {
                        /*
                        * This should NOT occur normally, only if Inf or NaN.
                        * For completeness, just tell the car to move
                        * in a straight line.
                        */
                        DBG("Move: [! ERR !]\t");
                        left_wheel_speed = INIT_LEFT_WHEEL_SPEED;
                        right_wheel_speed = INIT_RIGHT_WHEEL_SPEED;
                }
        }

        /* I want to see what values are being sent to the motor. */
        DBG("L: ");
        DBG(left_wheel_speed);
        DBG("\tR:");
        DBG(right_wheel_speed);
        DBG("\t");
        DBG(error);
        DBG("\t");

        /* Update error_rate parameters to values to be used in next loop. */
        last_time = cur_time;
        last_error = error;

        DBG("\n");

        /*
         * Need to put a delay in so that we do not get garbage readings.
         * To small, and we will get garbage serial/debugging readins, too
         * large, and the Robot will not be able to compensate for sharp
         * turns in time.
         */
        delay(10);

        /* Actually tell the motor to update the speed. */
        analogWrite(LEFT_PWM_PIN, left_wheel_speed);
        analogWrite(RIGHT_PWM_PIN, right_wheel_speed);
}
