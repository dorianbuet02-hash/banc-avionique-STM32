#ifndef OLED_H
#define OLED_H

#include "main.h"

// L'adresse de l'écran (0x3C) décalée d'un bit vers la gauche pour le HAL
#define OLED_I2C_ADDR 0x78

// Nos fonctions magiques
void OLED_Init(void);
void OLED_Fill(uint8_t color);


void OLED_SetCursor(uint8_t x, uint8_t y);
void OLED_WriteChar(char c);
void OLED_WriteString(char *str, uint8_t x, uint8_t y);

#endif


