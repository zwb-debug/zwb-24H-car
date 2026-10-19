/**
 * @file  mission.h
 * @brief 赛题流程状态机（第 1~4 题）
 */
#ifndef MISSION_H
#define MISSION_H

#include <stdint.h>
#include "board.h"

typedef struct {
    run_state_t state;
    task_id_t   task;
    uint8_t     seg_idx;
    uint8_t     seg_num;
    uint8_t     lap;
    uint8_t     lap_total;
    float       s_seg;
    float       s_total;
    float       v_ref;
    float       v_now;
    uint32_t    elapsed_ms;
    uint32_t    limit_ms;
    uint8_t     fault_code;
} mission_info_t;

void mission_init(void);
void mission_request_start(task_id_t t);
void mission_request_abort(void);
void mission_tick(void);
const mission_info_t *mission_info(void);

#endif
