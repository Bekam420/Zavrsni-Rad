#ifndef LIQUIDCRYSTAL_I2C_H_
#define LIQUIDCRYSTAL_I2C_H_

#include "stm32f4xx_hal.h"

/* Command Instructions */
#define LCD_CLEARDISPLAY 0x01      // Briše sadržaj ekrana
#define LCD_RETURNHOME 0x02        // Postavlja kursor na početnu poziciju
#define LCD_ENTRYMODESET 0x04      // Postavlja način unosa teksta
#define LCD_DISPLAYCONTROL 0x08    // Kontrola prikaza (uključivanje/isključivanje ekrana, kursora, itd.)
#define LCD_CURSORSHIFT 0x10       // Pomicanje kursora i prikaza
#define LCD_FUNCTIONSET 0x20       // Postavlja osnovne funkcije LCD-a (bitni način rada, broj linija itd.)
#define LCD_SETCGRAMADDR 0x40      // Postavlja adresu CGRAM (memorija za prilagođene znakove)
#define LCD_SETDDRAMADDR 0x80      // Postavlja adresu DDRAM (memorija za prikaz znakova na ekranu)

/* Entry Mode Set */
#define LCD_ENTRYRIGHT 0x00        // Tekst se unosi s desna na lijevo
#define LCD_ENTRYLEFT 0x02         // Tekst se unosi s lijeva na desno (standardno)
#define LCD_ENTRYSHIFTINCREMENT 0x01 // Pomicanje kursora pri unosu teksta
#define LCD_ENTRYSHIFTDECREMENT 0x00 // Nepomicanje kursora pri unosu teksta

/* Display Control */
#define LCD_DISPLAYON 0x04         // Uključuje prikaz na ekranu
#define LCD_DISPLAYOFF 0x00        // Isključuje prikaz na ekranu
#define LCD_CURSORON 0x02          // Uključuje prikaz kursora
#define LCD_CURSOROFF 0x00         // Isključuje prikaz kursora
#define LCD_BLINKON 0x01           // Uključuje treptanje kursora
#define LCD_BLINKOFF 0x00          // Isključuje treptanje kursora

/* Cursor Shift */
#define LCD_DISPLAYMOVE 0x08       // Pomicanje cijelog prikaza na ekranu
#define LCD_CURSORMOVE 0x00        // Pomicanje samo kursora
#define LCD_MOVERIGHT 0x04         // Pomicanje udesno
#define LCD_MOVELEFT 0x00          // Pomicanje ulijevo

/* Function Set */
#define LCD_8BITMODE 0x10          // 8-bitni način rada
#define LCD_4BITMODE 0x00          // 4-bitni način rada
#define LCD_2LINE 0x08             // Dvije linije teksta
#define LCD_1LINE 0x00             // Jedna linija teksta
#define LCD_5x10DOTS 0x04          // Font 5x10 piksela
#define LCD_5x8DOTS 0x00           // Font 5x8 piksela

/* Backlight Control */
#define LCD_BACKLIGHT 0x08         // Uključuje pozadinsko osvjetljenje
#define LCD_NOBACKLIGHT 0x00       // Isključuje pozadinsko osvjetljenje

/* Control Bits */
#define ENABLE 0x04                // Omogućava slanje podataka na LCD
#define RW 0x00                    // Postavlja način rada na pisanje (RW = 0)
#define RS 0x01                    // Postavlja registr za podatke (RS = 1)

/* I2C Device Address */
#define DEVICE_ADDR  (0x27 << 1)   // Adresa I2C uređaja, pomaknuta za 1 bit ulijevo

// Function Prototypes
void HD44780_Init(uint8_t rows);                       // Inicijalizacija LCD-a s određenim brojem linija
void HD44780_Clear();                                  // Brisanje sadržaja ekrana
void HD44780_Home();                                   // Postavljanje kursora na početnu poziciju
void HD44780_NoDisplay();                              // Isključivanje prikaza na ekranu
void HD44780_Display();                                // Uključivanje prikaza na ekranu
void HD44780_NoBlink();                                // Isključivanje treptanja kursora
void HD44780_Blink();                                  // Uključivanje treptanja kursora
void HD44780_NoCursor();                               // Isključivanje prikaza kursora
void HD44780_Cursor();                                 // Uključivanje prikaza kursora
void HD44780_ScrollDisplayLeft();                      // Pomicanje cijelog prikaza ulijevo
void HD44780_ScrollDisplayRight();                     // Pomicanje cijelog prikaza udesno
void HD44780_LeftToRight();                            // Postavljanje smjera unosa s lijeva na desno
void HD44780_RightToLeft();                            // Postavljanje smjera unosa s desna na lijevo
void HD44780_AutoScroll();                             // Automatsko pomicanje prikaza prilikom unosa
void HD44780_NoAutoScroll();                           // Onemogućavanje automatskog pomicanja prikaza
void HD44780_CreateSpecialChar(uint8_t, uint8_t[]);    // Kreiranje prilagođenog znaka
void HD44780_PrintSpecialChar(uint8_t);                // Prikazivanje prilagođenog znaka
void HD44780_SetCursor(uint8_t, uint8_t);              // Postavljanje kursora na određenu poziciju
void HD44780_SetBacklight(uint8_t new_val);            // Postavljanje stanja pozadinskog osvjetljenja
void HD44780_LoadCustomCharacter(uint8_t char_num, uint8_t *rows); // Učitavanje prilagođenog znaka u memoriju
void HD44780_PrintStr(const char[]);                   // Ispisivanje niza znakova na LCD
void HD44780_PrintChar(char c);                        // Ispisivanje pojedinačnog znaka na LCD
void HD44780_Backlight(void);                          // Uključivanje pozadinskog osvjetljenja
void HD44780_NoBacklight(void);                        // Isključivanje pozadinskog osvjetljenja

#endif /* LIQUIDCRYSTAL_I2C_H_ */
