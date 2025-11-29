/*
 * extern.h
 *
 *  Created on: Nov 24, 2025
 *      Author: thpark
 */

#ifndef INC_EXTERN_H_
#define INC_EXTERN_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#include "../../class/task_class/task_class.h"

extern task_class *pTaskManager;

#endif

#endif /* INC_EXTERN_H_ */
