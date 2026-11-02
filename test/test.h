/**
  ******************************************************************************
  * @file    test.h
  * @brief   Unit-test entry points.
  *
  *          Each test is an independent build: select it by switching the Keil
  *          target and rebuilding.
  *            - "qibeng_software_TestWS": WS2812 color cycle (red->green->
  *              blue->white, 500ms each), driven by test_ws2812_init/process.
  *            - "qibeng_software_TestM":  motor speed cycle (0->16.5->33->66->100%,
  *              3000ms each, non-blocking soft-start), driven by
  *              test_motor_init/process.
  *          The normal target ("qibeng_software") defines neither TEST_WS2812
  *          nor TEST_MOTOR and runs the normal application instead.
  ******************************************************************************
  */
#ifndef __TEST_H
#define __TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported functions --------------------------------------------------------*/
void test_ws2812_init(void);
void test_ws2812_process(void);
void test_ws2812_red_init(void);
void test_ws2812_red_process(void);

void test_motor_init(void);
void test_motor_process(void);

#ifdef __cplusplus
}
#endif

#endif /* __TEST_H */
