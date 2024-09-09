#include "liquidcrystal_i2c.h"

extern I2C_HandleTypeDef hi2c1;  // Definira vanjsku varijablu za I2C komunikaciju

uint8_t dpFunction;    // Varijabla za pohranu funkcionalnih postavki LCD-a
uint8_t dpControl;     // Varijabla za pohranu postavki kontrole prikaza LCD-a
uint8_t dpMode;        // Varijabla za pohranu postavki ulaza i pomaka LCD-a
uint8_t dpRows;        // Varijabla za pohranu broja redaka na LCD-u
uint8_t dpBacklight;   // Varijabla za pohranu stanja pozadinskog osvjetljenja

// Deklaracije funkcija koje se koriste samo unutar ovog modula
static void SendCommand(uint8_t);
static void SendChar(uint8_t);
static void Send(uint8_t, uint8_t);
static void Write4Bits(uint8_t);
static void ExpanderWrite(uint8_t);
static void PulseEnable(uint8_t);
static void DelayInit(void);
static void DelayUS(uint32_t);

// Definicije posebnih znakova za LCD ć i š
uint8_t special1[8] = {
		 0b01010,
		    0b00100,
		    0b01111,
		    0b01000,
		    0b01111,
		    0b00001,
		    0b01111,
		    0b00000
};

uint8_t special2[8] = {
		   0b00010,
		    0b00100,
		    0b00000,
		    0b01111,
		    0b01000,
		    0b01000,
		    0b01111,
		    0b00000
};

// Inicijalizira LCD prikaz
void HD44780_Init(uint8_t rows)
{
    dpRows = rows;                  // Pohranjuje broj redaka LCD-a
    dpBacklight = LCD_BACKLIGHT;    // Postavlja pozadinsko osvjetljenje

    // Postavlja funkcionalne postavke LCD-a
    dpFunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
    if (dpRows > 1)
    {
        dpFunction |= LCD_2LINE;  // Ako LCD ima više od jednog retka, postavlja 2 retka
    }
    else
    {
        dpFunction |= LCD_5x10DOTS; // Inače, postavlja 5x10 točaka
    }

    DelayInit();    // Inicijalizira odgode
    HAL_Delay(50);  // Čeka 50 ms

    ExpanderWrite(dpBacklight); // Uključuje pozadinsko osvjetljenje
    HAL_Delay(1000);            // Čeka 1 sekundu

    // Inicijalizacija LCD-a u 4-bitnom modu
    Write4Bits(0x03 << 4);
    DelayUS(4500);
    Write4Bits(0x03 << 4);
    DelayUS(4500);
    Write4Bits(0x03 << 4);
    DelayUS(4500);
    Write4Bits(0x02 << 4);
    DelayUS(100);

    SendCommand(LCD_FUNCTIONSET | dpFunction); // Postavlja funkcionalne postavke LCD-a
    dpControl = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF; // Postavlja kontrolu prikaza
    HD44780_Display();   // Uključuje prikaz
    HD44780_Clear();     // Čisti ekran

    dpMode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT; // Postavlja način unosa
    SendCommand(LCD_ENTRYMODESET | dpMode);
    DelayUS(4500);

    // Kreira i učitava posebne znakove
    HD44780_CreateSpecialChar(0, special1);
    HD44780_CreateSpecialChar(1, special2);
    HD44780_Home();  // Postavlja kursor na početnu poziciju
}

// Briše sadržaj LCD-a
void HD44780_Clear(void)
{
    SendCommand(LCD_CLEARDISPLAY);
    DelayUS(2000);  // Čeka 2 ms da se operacija završi
}

// Vraća kursor na početnu poziciju
void HD44780_Home(void)
{
    SendCommand(LCD_RETURNHOME);
    DelayUS(2000);  // Čeka 2 ms da se operacija završi
}

// Postavlja kursor na određenu poziciju
void HD44780_SetCursor(uint8_t col, uint8_t row)
{
    int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 }; // Offseti za retke
    if (row >= dpRows)
    {
        row = dpRows - 1; // Ako je redak veći od broja redaka, postavi na zadnji redak
    }
    SendCommand(LCD_SETDDRAMADDR | (col + row_offsets[row])); // Postavlja adresu u DDRAM
}

// Isključuje prikaz na LCD-u
void HD44780_NoDisplay(void)
{
    dpControl &= ~LCD_DISPLAYON;  // Isključuje prikaz
    SendCommand(LCD_DISPLAYCONTROL | dpControl);
}

// Uključuje prikaz na LCD-u
void HD44780_Display(void)
{
    dpControl |= LCD_DISPLAYON;   // Uključuje prikaz
    SendCommand(LCD_DISPLAYCONTROL | dpControl);
}

// Isključuje pokazivač (kursor) na LCD-u
void HD44780_NoCursor(void)
{
    dpControl &= ~LCD_CURSORON;  // Isključuje kursor
    SendCommand(LCD_DISPLAYCONTROL | dpControl);
}

// Uključuje pokazivač (kursor) na LCD-u
void HD44780_Cursor(void)
{
    dpControl |= LCD_CURSORON;   // Uključuje kursor
    SendCommand(LCD_DISPLAYCONTROL | dpControl);
}

// Isključuje treptanje kursora
void HD44780_NoBlink(void)
{
    dpControl &= ~LCD_BLINKON;  // Isključuje treptanje
    SendCommand(LCD_DISPLAYCONTROL | dpControl);
}

// Uključuje treptanje kursora
void HD44780_Blink(void)
{
    dpControl |= LCD_BLINKON;   // Uključuje treptanje
    SendCommand(LCD_DISPLAYCONTROL | dpControl);
}

// Pomakne prikaz lijevo
void HD44780_ScrollDisplayLeft(void)
{
    SendCommand(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}

// Pomakne prikaz desno
void HD44780_ScrollDisplayRight(void)
{
    SendCommand(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// Postavlja način unosa s lijeva na desno
void HD44780_LeftToRight(void)
{
    dpMode |= LCD_ENTRYLEFT;
    SendCommand(LCD_ENTRYMODESET | dpMode);
}

// Postavlja način unosa s desna na lijevo
void HD44780_RightToLeft(void)
{
    dpMode &= ~LCD_ENTRYLEFT;
    SendCommand(LCD_ENTRYMODESET | dpMode);
}

// Uključuje automatsko pomicanje prikaza
void HD44780_AutoScroll(void)
{
    dpMode |= LCD_ENTRYSHIFTINCREMENT;
    SendCommand(LCD_ENTRYMODESET | dpMode);
}

// Isključuje automatsko pomicanje prikaza
void HD44780_NoAutoScroll(void)
{
    dpMode &= ~LCD_ENTRYSHIFTINCREMENT;
    SendCommand(LCD_ENTRYMODESET | dpMode);
}

// Kreira poseban znak na LCD-u
void HD44780_CreateSpecialChar(uint8_t location, uint8_t charmap[])
{
    location &= 0x7;  // Ograničava lokaciju na raspon 0-7
    SendCommand(LCD_SETCGRAMADDR | (location << 3));  // Postavlja adresu u CGRAM
    for (int i = 0; i < 8; i++)
    {
        SendChar(charmap[i]);  // Učitava svaki redak posebnog znaka
    }
}

// Ispisuje poseban znak na LCD-u
void HD44780_PrintSpecialChar(uint8_t index)
{
    SendChar(index);  // Ispisuje znak prema indeksu
}

// Učitava prilagođeni znak u LCD
void HD44780_LoadCustomCharacter(uint8_t char_num, uint8_t *rows)
{
    HD44780_CreateSpecialChar(char_num, rows);
}

// Ispisuje niz znakova na LCD-u
void HD44780_PrintStr(const char c[])
{
    while (*c)
    {
        SendChar(*c++);  // Ispisuje svaki znak niza
    }
}

// Ispisuje jedan znak na LCD-u
void HD44780_PrintChar(char c)
{
    SendChar(c);  // Ispisuje znak
}

// Postavlja stanje pozadinskog osvjetljenja
void HD44780_SetBacklight(uint8_t new_val)
{
    if (new_val) HD44780_Backlight();
    else HD44780_NoBacklight();
}

// Isključuje pozadinsko osvjetljenje
void HD44780_NoBacklight(void)
{
    dpBacklight = LCD_NOBACKLIGHT;  // Postavlja stanje na bez pozadinskog osvjetljenja
    ExpanderWrite(0);  // Šalje komandu za isključivanje osvjetljenja
}

// Uključuje pozadinsko osvjetljenje
void HD44780_Backlight(void)
{
    dpBacklight = LCD_BACKLIGHT;  // Postavlja stanje na pozadinsko osvjetljenje
    ExpanderWrite(0);  // Šalje komandu za uključivanje osvjetljenja
}

// Šalje komandu na LCD
static void SendCommand(uint8_t cmd)
{
    Send(cmd, 0);  // Poziva funkciju za slanje komande bez RS signala
}

// Šalje znak na LCD
static void SendChar(uint8_t ch)
{
    Send(ch, RS);  // Poziva funkciju za slanje znaka s RS signalom
}

// Šalje podatke na LCD
static void Send(uint8_t value, uint8_t mode)
{
    uint8_t highnib = value & 0xF0;  // Visoki 4 bita
    uint8_t lownib = (value << 4) & 0xF0;  // Niski 4 bita
    Write4Bits((highnib) | mode);  // Šalje visoki nibble s modom
    Write4Bits((lownib) | mode);  // Šalje niski nibble s modom
}

// Šalje 4 bita podataka na LCD
static void Write4Bits(uint8_t value)
{
    ExpanderWrite(value);  // Šalje podatke na ekspanzionu ploču
    PulseEnable(value);    // Generira enable puls
}

// Šalje podatke na ekspanzionu ploču preko I2C-a
static void ExpanderWrite(uint8_t _data)
{
    uint8_t data = _data | dpBacklight;  // Dodaje stanje pozadinskog osvjetljenja
    HAL_I2C_Master_Transmit(&hi2c1, DEVICE_ADDR, &data, 1, 10);  // Šalje podatke preko I2C-a
}

// Generira enable puls za LCD
static void PulseEnable(uint8_t _data)
{
    ExpanderWrite(_data | ENABLE);  // Šalje enable signal
    DelayUS(20);  // Čeka 20 us

    ExpanderWrite(_data & ~ENABLE);  // Onemogućava enable signal
    DelayUS(20);  // Čeka 20 us
}

// Inicijalizira odgode
static void DelayInit(void)
{
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;  // Isključuje trace
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   // Uključuje trace

    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;  // Isključuje brojač ciklusa
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;   // Uključuje brojač ciklusa

    DWT->CYCCNT = 0;  // Postavlja brojač ciklusa na nulu

    __ASM volatile ("NOP");
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");
}

// Čeka određeni broj mikrosekundi
static void DelayUS(uint32_t us)
{
    uint32_t cycles = (SystemCoreClock / 1000000L) * us;  // Izračunava broj ciklusa za željeno vrijeme
    uint32_t start = DWT->CYCCNT;  // Sprema trenutni broj ciklusa
    volatile uint32_t cnt;

    do
    {
        cnt = DWT->CYCCNT - start;  // Mjeri vrijeme koje je prošlo
    } while (cnt < cycles);  // Čeka dok ne prođe željeno vrijeme
}
