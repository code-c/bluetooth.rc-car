// Example file - Public Domain
// Need help? https://tinyurl.com/bluepad32-help

#include "bdc_motor.h"
#include "controller/uni_gamepad.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <uni.h>

// Declarations
#define DRIVEMOTOR_MCPWM_GPIO_A 12                    // forward
#define DRIVEMOTOR_MCPWM_GPIO_B 13                    // reverse
#define DRIVEMOTOR_MCPWM_FREQ_HZ 25000                // frequency = 25kHz
#define DRIVEMOTOR_MCPWM_TIMER_RESOLUTION_HZ 80000000 // 160MHz, 1 tick = 0.1us
#define DRIVEMOTOR_MCPWM_DUTY_MAX                                              \
  (DRIVEMOTOR_MCPWM_TIMER_RESOLUTION_HZ / DRIVEMOTOR_MCPWM_FREQ_HZ) // in ticks
#define STEERINGMOTOR_MCPWM_GPIO_A 32                               // right
#define STEERINGMOTOR_MCPWM_GPIO_B 33                               // left
#define STEERINGMOTOR_MCPWM_FREQ_HZ 25000 // frequency = 25kHz
#define STEERINGMOTOR_MCPWM_TIMER_RESOLUTION_HZ                                \
  80000000 // 160MHz, 1 tick = 0.1us
#define STEERINGMOTOR_MCPWM_DUTY_MAX                                           \
  (STEERINGMOTOR_MCPWM_TIMER_RESOLUTION_HZ /                                   \
   STEERINGMOTOR_MCPWM_FREQ_HZ) // in ticks

// brushless DC motor control setup
static const bdc_motor_config_t motor_config = {
    .pwm_freq_hz = DRIVEMOTOR_MCPWM_FREQ_HZ,
    .pwma_gpio_num = DRIVEMOTOR_MCPWM_GPIO_A,
    .pwmb_gpio_num = DRIVEMOTOR_MCPWM_GPIO_B,
};
static const bdc_motor_config_t steering_config = {
    .pwm_freq_hz = STEERINGMOTOR_MCPWM_FREQ_HZ,
    .pwma_gpio_num = STEERINGMOTOR_MCPWM_GPIO_A,
    .pwmb_gpio_num = STEERINGMOTOR_MCPWM_GPIO_B,
};
static const bdc_motor_mcpwm_config_t mcpwm_config = {
    .group_id = 0,
    .resolution_hz = DRIVEMOTOR_MCPWM_TIMER_RESOLUTION_HZ,
};

static bdc_motor_handle_t drive_motor = NULL;
static bdc_motor_handle_t steering_motor = NULL;

// Custom "instance"
typedef struct my_platform_instance_t {
  uni_gamepad_seat_t gamepad_seat; // which "seat" is being used
} my_platform_instance_t;

static void trigger_event_on_gamepad(uni_hid_device_t *d);
static my_platform_instance_t *get_my_platform_instance(uni_hid_device_t *d);

//
// Platform Overrides
//
static void my_platform_init(int argc, const char **argv) {
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);
  bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &drive_motor);
  bdc_motor_new_mcpwm_device(&steering_config, &mcpwm_config, &steering_motor);
  bdc_motor_enable(drive_motor);
  bdc_motor_enable(steering_motor);
#if 0
    uni_gamepad_mappings_t mappings = GAMEPAD_DEFAULT_MAPPINGS;

    // Inverted axis with inverted Y in RY.
    mappings.axis_x = UNI_GAMEPAD_MAPPINGS_AXIS_RX;
    mappings.axis_y = UNI_GAMEPAD_MAPPINGS_AXIS_RY;
    mappings.axis_ry_inverted = true;
    mappings.axis_rx = UNI_GAMEPAD_MAPPINGS_AXIS_X;
    mappings.axis_ry = UNI_GAMEPAD_MAPPINGS_AXIS_Y;

    // Invert A & B
    mappings.button_a = UNI_GAMEPAD_MAPPINGS_BUTTON_B;
    mappings.button_b = UNI_GAMEPAD_MAPPINGS_BUTTON_A;

    uni_gamepad_set_mappings(&mappings);
#endif
  //    uni_bt_service_set_enabled(true);
}

static void my_platform_on_init_complete(void) {
  logi("custom: on_init_complete()\n");

  // Safe to call "unsafe" functions since they are called from BT thread

  // Start scanning
  uni_bt_start_scanning_and_autoconnect_unsafe();
  uni_bt_allow_incoming_connections(true);

  // Based on runtime condition, you can delete or list the stored BT keys.
  if (1)
    uni_bt_del_keys_unsafe();
  else
    uni_bt_list_keys_unsafe();
}

static uni_error_t my_platform_on_device_discovered(bd_addr_t addr,
                                                    const char *name,
                                                    uint16_t cod,
                                                    uint8_t rssi) {
  if (((cod & UNI_BT_COD_MINOR_MASK) & UNI_BT_COD_MINOR_KEYBOARD) ==
      UNI_BT_COD_MINOR_KEYBOARD) {
    logi("Ignoring keyboard\n");
    return UNI_ERROR_IGNORE_DEVICE;
  }
  return UNI_ERROR_SUCCESS;
}

static void my_platform_on_device_connected(uni_hid_device_t *d) {
  logi("custom: device connected: %p\n", d);
  logi("*** Stop scanning\n");
  uni_bt_stop_scanning_safe();
}

static void my_platform_on_device_disconnected(uni_hid_device_t *d) {
  logi("custom: device disconnected: %p\n", d);
  logi("*** Start scanning\n");
  uni_bt_start_scanning_and_autoconnect_safe();
}

static uni_error_t my_platform_on_device_ready(uni_hid_device_t *d) {
  logi("custom: device ready: %p\n", d);
  my_platform_instance_t *ins = get_my_platform_instance(d);
  ins->gamepad_seat = GAMEPAD_SEAT_A;

  trigger_event_on_gamepad(d);
  return UNI_ERROR_SUCCESS;
}

static void my_platform_on_controller_data(uni_hid_device_t *d,
                                           uni_controller_t *ctl) {
  static uint8_t power = false;
  uni_controller_t prev = {0};
  uni_gamepad_t *gp;

  // Optimization to avoid processing the previous data so that the console
  // does not get spammed with a lot of logs, but remove it from your project.
  if (memcmp(&prev, ctl, sizeof(*ctl)) == 0) {
    return;
  }
  prev = *ctl;

  switch (ctl->klass) {
  case UNI_CONTROLLER_CLASS_GAMEPAD:
    gp = &ctl->gamepad;
    /*------------------------------------ *
     * Steering motor logic                *
     *------------------------------------ */
    if (abs(gp->axis_x) > 1) {
      if (gp->axis_x > 115) {
        float_t turn_amount = roundf(
            ((gp->axis_x - 115) * STEERINGMOTOR_MCPWM_DUTY_MAX) / (512 - 115));
        logi("right turn: %f\n", turn_amount);
        bdc_motor_reverse(steering_motor);
        bdc_motor_set_speed(steering_motor, turn_amount);
      }

      if (gp->axis_x < -115) {
        float_t turn_amount =
            roundf(((abs(gp->axis_x) - 115) * STEERINGMOTOR_MCPWM_DUTY_MAX) /
                   (512 - 115));
        logi("left turn: %f\n", turn_amount);
        bdc_motor_forward(steering_motor);
        bdc_motor_set_speed(steering_motor, turn_amount);
      }

      if (gp->axis_x > -30 && gp->axis_x < 30) {
        bdc_motor_coast(steering_motor);
      }
    }
    /*-------------------------------
     * drive motor logic
     * ------------------------------*/

    // coast motor if previously powered but no triggers are depressed
    if ((!(gp->buttons & BUTTON_TRIGGER_L) ||
         !(gp->buttons & BUTTON_TRIGGER_R)) &&
        power) {

      bdc_motor_coast(drive_motor);
      power = false;
    }
    if (((gp->buttons & BUTTON_TRIGGER_L) &&
         (gp->buttons & BUTTON_TRIGGER_R)) &&
        power) {

      bdc_motor_coast(drive_motor);
    }

    // HANDBRAKE takes priority and will imediately stop motors
    if (gp->buttons & BUTTON_A) {

      bdc_motor_brake(drive_motor);
      power = false;

    } else {
      if ((gp->buttons & BUTTON_TRIGGER_L) && !power) {
        float_t speed =
            roundf((gp->brake * DRIVEMOTOR_MCPWM_DUTY_MAX) / 1020.0);
        bdc_motor_reverse(drive_motor);
        bdc_motor_set_speed(drive_motor, speed);
        power = true;
        break;
      } // if right trigger is pressed then we drive forward
      if ((gp->buttons & BUTTON_TRIGGER_R) && !power) {
        float_t speed =
            roundf((gp->throttle * DRIVEMOTOR_MCPWM_DUTY_MAX) / 1020.0);
        // logi("forward: %f\n", speed);
        bdc_motor_forward(drive_motor);
        bdc_motor_set_speed(drive_motor, speed);
        power = true;
        break;
      }
    }

    break;
  default:
    break;
  }
}

static const uni_property_t *my_platform_get_property(uni_property_idx_t idx) {
  ARG_UNUSED(idx);
  return NULL;
}

static void my_platform_on_oob_event(uni_platform_oob_event_t event,
                                     void *data) {
  switch (event) {
  case UNI_PLATFORM_OOB_GAMEPAD_SYSTEM_BUTTON: {
    uni_hid_device_t *d = data;

    if (d == NULL) {
      loge("ERROR: my_platform_on_oob_event: Invalid NULL device\n");
      return;
    }
    logi("custom: on_device_oob_event(): %d\n", event);

    my_platform_instance_t *ins = get_my_platform_instance(d);
    ins->gamepad_seat =
        ins->gamepad_seat == GAMEPAD_SEAT_A ? GAMEPAD_SEAT_B : GAMEPAD_SEAT_A;

    trigger_event_on_gamepad(d);
    break;
  }

  case UNI_PLATFORM_OOB_BLUETOOTH_ENABLED:
    logi("custom: Bluetooth enabled: %d\n", (bool)(data));
    break;

  default:
    logi("my_platform_on_oob_event: unsupported event: 0x%04x\n", event);
    break;
  }
}

//
// Helpers
//
static my_platform_instance_t *get_my_platform_instance(uni_hid_device_t *d) {
  return (my_platform_instance_t *)&d->platform_data[0];
}

static void trigger_event_on_gamepad(uni_hid_device_t *d) {
  my_platform_instance_t *ins = get_my_platform_instance(d);

  if (d->report_parser.play_dual_rumble != NULL) {
    d->report_parser.play_dual_rumble(
        d, 0 /* delayed start ms */, 150 /* duration ms */,
        128 /* weak magnitude */, 40 /* strong magnitude */);
  }

  if (d->report_parser.set_player_leds != NULL) {
    d->report_parser.set_player_leds(d, ins->gamepad_seat);
  }

  if (d->report_parser.set_lightbar_color != NULL) {
    uint8_t red = (ins->gamepad_seat & 0x01) ? 0xff : 0;
    uint8_t green = (ins->gamepad_seat & 0x02) ? 0xff : 0;
    uint8_t blue = (ins->gamepad_seat & 0x04) ? 0xff : 0;
    d->report_parser.set_lightbar_color(d, red, green, blue);
  }
}

//
// Entry Point
//
struct uni_platform *get_my_platform(void) {
  static struct uni_platform plat = {
      .name = "custom",
      .init = my_platform_init,
      .on_init_complete = my_platform_on_init_complete,
      .on_device_discovered = my_platform_on_device_discovered,
      .on_device_connected = my_platform_on_device_connected,
      .on_device_disconnected = my_platform_on_device_disconnected,
      .on_device_ready = my_platform_on_device_ready,
      .on_oob_event = my_platform_on_oob_event,
      .on_controller_data = my_platform_on_controller_data,
      .get_property = my_platform_get_property,
  };

  return &plat;
}
