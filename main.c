#include <stdio.h>
#include "pico/stdlib.h"
#include "robotic_arm.h"
#include "string.h"

/// 初始化機械手臂的伺服馬達參數與 GPIO 腳位
void robotic_arm_starter(robotic_arm* robot_arm, servo* motor) {
    if (!robot_arm || !motor) {
        fprintf(stderr, "Invalid robotic arm or servo pointer.\n");
        return;
    }

    // 將同一顆馬達設定複製給每個軸
    for (uint8_t i = 0; i < robot_arm->number; i++) {
        memcpy(&robot_arm->servos[i], motor, sizeof(servo));
        robotic_arm_set_servo_pin(robot_arm, i, i + 16); // GPIO 16 起始
    }

    // 針對 servo 1 設定角度範圍限制
    robotic_arm_set_servo_limits(robot_arm, 1, 3.0f, 177.0f);

    // 啟動控制 PWM 輸出
    robotic_arm_start(robot_arm);
}

/// 自訂模式：根據輸入字元觸發一連串的預設動作（例如 a/m/g/p）
void robotic_arm_custom_control_mode(robotic_arm* robot_arm) {
    uint8_t control_servos[robot_arm->number];
    float target_angles[robot_arm->number];
    robotic_arm_signal control_signal = {
        .indexes = control_servos,
        .angles = target_angles,
        .number = 0
    };

    // 預先定義四組動作序列（A = all, M = metal, G = glass, P = plastic）
    char pick_and_return[][40] = {
        "3 0 150 1 40 2 126",// 3 = 馬達數量，0 = 馬達編號，150 = 角度，依序是馬達0角度150，馬達1角度40，馬達2角度126
        "1 3 165",
        "1 1 90",
        "1 0 90"
    };
    char throw_and_return[][40] = {
        "1 3 90",
        "3 0 90 1 90 2 90"
    };
    char action_a[][40] = {"3 0 90 1 65 2 145"}; // 動作 A：3個馬達分別到指定角度
    char action_m[][40] = {"3 0 85 1 25 2 47"};
    char action_g[][40] = {"3 0 36 1 65 2 140"};
    char action_p[][40] = {"3 0 51 1 30 2 40"};

    char action_tip[] = "Enter 'a', 'm', 'g', or 'p' to play actions, 'q' to quit:\n";
    printf(action_tip);

    // 等待使用者輸入模式字元
    while (true) {
        int input = getchar();
        if(input == '\n') continue; // 忽略換行符號

        // 根據輸入選擇動作序列
        char (*selected_action)[40] = NULL;
        int action_len = 0;
        selected_action = pick_and_return;
        action_len = sizeof(pick_and_return)/sizeof(pick_and_return[0]);
        for (int i = 0; i < action_len; i++) {
            robotic_arm_signal_from_string(&control_signal, selected_action[i]);
            robotic_arm_move(robot_arm, &control_signal);
            sleep_ms(100); // 每段動作間稍微間隔
        }
        switch (input) {
            case 'a': case 'A': selected_action = action_a; action_len = sizeof(action_a)/sizeof(action_a[0]); break;
            case 'm': case 'M': selected_action = action_m; action_len = sizeof(action_m)/sizeof(action_m[0]); break;
            case 'g': case 'G': selected_action = action_g; action_len = sizeof(action_g)/sizeof(action_g[0]); break;
            case 'p': case 'P': selected_action = action_p; action_len = sizeof(action_p)/sizeof(action_p[0]); break;
            default:
                printf("Invalid command.\n%s", action_tip);
                continue;
        }

        // 執行動作序列（轉成控制信號並移動）
        for (int i = 0; i < action_len; i++) {
            robotic_arm_signal_from_string(&control_signal, selected_action[i]);
            robotic_arm_move(robot_arm, &control_signal);
            sleep_ms(100); // 每段動作間稍微間隔
        }
        selected_action = throw_and_return;
        action_len = sizeof(throw_and_return)/sizeof(throw_and_return[0]);
        for (int i = 0; i < action_len; i++) {
            robotic_arm_signal_from_string(&control_signal, selected_action[i]);
            robotic_arm_move(robot_arm, &control_signal);
            sleep_ms(100); // 每段動作間稍微間隔
        }
    }
}

/// 主程式：初始化 servo 與 robotic_arm，啟用自訂控制模式
int main() {
    stdio_init_all();

    // 等待 USB 串列連線完成
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    // 設定 MG996R 伺服馬達參數
    servo mg996r = {
        .angle_range = 180.0f,
        .period = 20000,
        .min_duty = 500,
        .max_duty = 2500,
        .angle = 90.0f,
        .angle_lower_bound = 0.0f,
        .angle_upper_bound = 180.0f
    };

    // 建立四軸機械手臂
    robotic_arm* robot_arm = robotic_arm_create(4);
    if (!robot_arm) {
        fprintf(stderr, "Failed to create robotic arm.\n");
        return 1;
    }

    // 初始化馬達參數與 GPIO 腳位
    robotic_arm_starter(robot_arm, &mg996r);

    printf("Robotic arm initialized with %d servos.\n", robot_arm->number);

    // 進入自訂控制模式
    robotic_arm_custom_control_mode(robot_arm);

    return 0;
}
