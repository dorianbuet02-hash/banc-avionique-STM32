#include "servo_pwm.h"

/* --------------------------------------------------------------------------
 * FP4 – Initialisation PWM par acces direct aux registres (sans HAL)
 *        PA0 = TIM2_CH1, 50 Hz, SYSCLK = 32 MHz
 * -------------------------------------------------------------------------- */
void PWM_Servo_Init(void) {

    /* ETAPE 1 : Activation des horloges */
    RCC->AHB2ENR  |= (1U << 0);  /* GPIOAEN : horloge GPIOA */
    RCC->APB1ENR1 |= (1U << 0);  /* TIM2EN  : horloge TIM2  */

    /* ETAPE 2 : Configuration de PA0 en Alternate Function AF1 (TIM2_CH1) */
    GPIOA->MODER  &= ~(3U  << 0); /* effacer MODER0            */
    GPIOA->MODER  |=  (2U  << 0); /* 0b10 = Alternate Function */
    GPIOA->AFR[0] &= ~(0xFU << 0);/* effacer AFSEL0            */
    GPIOA->AFR[0] |=  (1U  << 0); /* AF1 = TIM2_CH1            */

    /* ETAPE 3 : Configuration de TIM2 en PWM 50 Hz
     * PSC = 31 => f_cpt = 32MHz/32 = 1MHz => 1 tick = 1us
     * ARR = 19999 => T = 20ms => f = 50Hz                      */
    TIM2->PSC    =  31;
    TIM2->ARR    =  19999;

    TIM2->CCMR1 &= ~(0xFFU);      /* effacer canal 1           */
    TIM2->CCMR1 |=  (6U << 4);    /* OC1M = 110 : PWM mode 1   */
    TIM2->CCMR1 |=  (1U << 3);    /* OC1PE = 1  : preload      */

    TIM2->CCER  |=  (1U << 0);    /* CC1E = 1 : sortie activee */

    TIM2->CCR1   =  1500;         /* position centrale         */

    TIM2->EGR   |=  (1U << 0);    /* UG : mise a jour registres */
    TIM2->CR1   |=  (1U << 0);    /* CEN = 1 : timer demarre   */
}

/* --------------------------------------------------------------------------
 * FP4 – Mise a jour position servo : lacet [-45°,+45°] -> [1000,2000] us
 * -------------------------------------------------------------------------- */
void Mettre_A_Jour_Servomoteur(float angle_lacet) {
    if (angle_lacet >  45.0f) angle_lacet =  45.0f;
    if (angle_lacet < -45.0f) angle_lacet = -45.0f;

    uint32_t impulsion_us = (uint32_t)((angle_lacet * 11.11f) + 1500.0f);

    if (impulsion_us > 2000) impulsion_us = 2000;
    if (impulsion_us < 1000) impulsion_us = 1000;

    TIM2->CCR1 = impulsion_us;
}
