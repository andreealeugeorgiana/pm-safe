#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>

// Hardware configuration for ATmega328P
#define F_CPU 16000000UL

// Pin definitions (Arduino Nano mapping)
// Keypad rows: PD2-PD5 (Digital pins 2-5)
// Keypad cols: PD6-PD7, PB0-PB1 (Digital pins 6-9)
// Servo: PB2 (Digital pin 10, OC1B)
// Buzzer: PB3 (Digital pin 11, OC2A)
// PIR: PB4 (Digital pin 12)
// LED Red: PC0 (Analog pin A0)
// LED Green: PC1 (Analog pin A1)
// I2C: PC4 (SDA), PC5 (SCL)

#define KEYPAD_ROWS 4
#define KEYPAD_COLS 4

// System constants
#define MAX_PASSWORD_LENGTH 8
#define TIMEOUT_MOTION_MS 10000
#define UPDATE_INTERVAL_MS 100

// Global variables
static char correct_password[] = "1234";
static char entered_password[MAX_PASSWORD_LENGTH + 1];
static uint8_t password_index = 0;
static uint8_t system_active = 0;
static uint8_t alarm_active = 0;
static uint32_t last_motion_time = 0;
static uint32_t last_update_time = 0;
static uint32_t system_time_ms = 0;
// LCD Definitions
#define LCD_I2C_ADDR 0x27 // Default I2C address for PCF8574
#define LCD_ENABLE 0x04    // Enable bit
#define LCD_BACKLIGHT 0x08 // Backlight control
#define LCD_CMD 0x00       // Command mode
#define LCD_DATA 0x01      // Data mode

// Keypad matrix
static char keypad_matrix[KEYPAD_ROWS][KEYPAD_COLS] = {
    {'D', 'C', 'B', 'A'},
    {'#', '9', '6', '3'},
    {'0', '8', '5', '2'},
    {'*', '7', '4', '1'}
};

// Function prototypes
void system_init(void);
void timer_init(void);
void i2c_init(void);
void pwm_init(void);
void uart_init(void);
void uart_send_string(const char* str);
char keypad_scan(void);
void set_servo_position(uint8_t angle);
void buzzer_tone(uint16_t frequency, uint16_t duration_ms);
void buzzer_off(void);
void lcd_init(void);
void lcd_clear(void);
void lcd_set_cursor(uint8_t col, uint8_t row);
void lcd_print(const char* str);
void i2c_start(void);
void i2c_stop(void);
void i2c_write(uint8_t data);
void display_password_prompt(void);
void process_key(char key);
void verify_password(void);
void reset_password(void);
void play_success_melody(void);
void play_error_melody(void);
void check_pir_sensor(void);
void lock_system(void);
void activate_alarm(void);
void deactivate_alarm(void);
void update_display(void);
uint32_t get_system_time_ms(void);

// Timer0 overflow interrupt for system timing
ISR(TIMER0_OVF_vect) {
    static uint8_t overflow_count = 0;
    overflow_count++;
    
    // Timer0 overflows every ~1.024ms at 16MHz with prescaler 64
    if (overflow_count >= 1) {
        system_time_ms++;
        overflow_count = 0;
    }
}

int main(void) {
    system_init();
    uart_send_string("=== ATmega328P Security System ===\r\n");
    uart_send_string("Initializing...\r\n");
    
    // Startup message on LCD
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("ATmega Security");
    lcd_set_cursor(0, 1);
    lcd_print("System v1.0");
    _delay_ms(2000);
    
    display_password_prompt();
    uart_send_string("System ready!\r\n");
    
    while (1) {
        uint32_t current_time = get_system_time_ms();
        
        // Scan keypad
        char key = keypad_scan();
        if (key != 0) {
            char msg[30];
            sprintf(msg, "Key pressed: %c\r\n", key);
            uart_send_string(msg);
            process_key(key);
        }
        
        // Check PIR sensor if system is active
        if (system_active) {
            check_pir_sensor();
        }
        
        // Update display periodically
        if (current_time - last_update_time >= UPDATE_INTERVAL_MS) {
            last_update_time = current_time;
            update_display();
        }
        
        // Handle timer overflow protection
        if (current_time < last_motion_time) {
            last_motion_time = current_time;
        }
        
        _delay_ms(10); // Small delay to prevent excessive CPU usage
    }
    
    return 0;
}

void system_init(void) {
    // Initialize all hardware
    timer_init();
    uart_init();
    i2c_init();
    pwm_init();
    
    // Configure GPIO pins
    // Keypad rows as inputs with pull-ups (PD2-PD5)
    DDRD &= ~((1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5));
    PORTD |= (1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5);
    
    // Keypad columns as outputs (PD6-PD7, PB0-PB1)
    DDRD |= (1 << PD6) | (1 << PD7);
    DDRB |= (1 << PB0) | (1 << PB1);
    
    // PIR sensor as input (PB4)
    DDRB &= ~(1 << PB4);
    PORTB &= ~(1 << PB4); // No pull-up for PIR
    
    // LEDs as outputs (PC0, PC1)
    DDRC |= (1 << PC0) | (1 << PC1);
    PORTC &= ~((1 << PC0) | (1 << PC1)); // Initially off
    
    // Servo and buzzer pins configured in PWM init
    
    // Test LEDs
    PORTC |= (1 << PC0); // Red LED on
    _delay_ms(200);
    PORTC &= ~(1 << PC0); // Red LED off
    PORTC |= (1 << PC1); // Green LED on
    _delay_ms(200);
    PORTC &= ~(1 << PC1); // Green LED off
    
    // Test buzzer
    buzzer_tone(1000, 100);
    _delay_ms(150);
    
    // Initialize servo to closed position
    set_servo_position(0);
    
    // Initialize LCD
    lcd_init();
    
    // Enable global interrupts
    sei();
}

void timer_init(void) {
    // Timer0 for system timing (8-bit timer)
    TCCR0A = 0; // Normal mode
    TCCR0B = (1 << CS01) | (1 << CS00); // Prescaler 64
    TIMSK0 |= (1 << TOIE0); // Enable overflow interrupt
    
    // Timer1 for servo PWM (16-bit timer)
    TCCR1A = (1 << COM1B1) | (1 << WGM11); // Fast PWM, non-inverting mode
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Fast PWM, prescaler 8
    ICR1 = 19999; // 20ms period for servo (50Hz)
    
    // Timer2 for buzzer PWM (8-bit timer)
    TCCR2A = (1 << COM2A0) | (1 << WGM21); // Toggle OC2A, CTC mode
    TCCR2B = 0; // Timer stopped initially
}

void uart_init(void) {
    // Set baud rate to 9600
    uint16_t ubrr = F_CPU/16/9600-1;
    UBRR0H = (ubrr >> 8);
    UBRR0L = ubrr;
    // Enable transmitter
    UCSR0B = (1 << TXEN0);
    // Set frame format: 8 data bits, 1 stop bit
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_send_string(const char* str) {
    while (*str) {
        // Wait for empty transmit buffer
        while (!(UCSR0A & (1 << UDRE0)));
        UDR0 = *str++;
    }
}

void i2c_init(void) {
    // Set SCL frequency to 100kHz
    TWBR = ((F_CPU/100000) - 16) / 2;
    TWSR = 0; // Prescaler = 1
}

void pwm_init(void) {
    // Configure servo pin (PB2) as output
    DDRB |= (1 << PB2);
    
    // Configure buzzer pin (PB3) as output
    DDRB |= (1 << PB3);
}

char keypad_scan(void) {
    static uint32_t last_scan_time = 0;
    static char last_key = 0;
    static uint8_t key_released = 1; // Flag to track key release
    uint32_t current_time = get_system_time_ms();
    
    // Debounce delay - increased from 50ms to 100ms
    if (current_time - last_scan_time < 100) {
        return 0;
    }
    
    char current_key = 0;
    
    for (uint8_t col = 0; col < KEYPAD_COLS; col++) {
        // Set all columns high
        PORTD |= (1 << PD6) | (1 << PD7);
        PORTB |= (1 << PB0) | (1 << PB1);
        
        // Set current column low
        switch (col) {
            case 0: PORTD &= ~(1 << PD6); break;
            case 1: PORTD &= ~(1 << PD7); break;
            case 2: PORTB &= ~(1 << PB0); break;
            case 3: PORTB &= ~(1 << PB1); break;
        }
        
        _delay_us(10); // Small delay for signal stabilization
        
        // Check rows
        for (uint8_t row = 0; row < KEYPAD_ROWS; row++) {
            uint8_t pin_state = 0;
            switch (row) {
                case 0: pin_state = !(PIND & (1 << PD2)); break;
                case 1: pin_state = !(PIND & (1 << PD3)); break;
                case 2: pin_state = !(PIND & (1 << PD4)); break;
                case 3: pin_state = !(PIND & (1 << PD5)); break;
            }
            
            if (pin_state) {
                current_key = keypad_matrix[row][col];
            }
        }
    }
    
    // Only register a new key if:
    // 1. A key is pressed (current_key != 0)
    // 2. It's different from the last key or the key was previously released
    // 3. The key was actually released before this press
    if (current_key != 0 && (current_key != last_key || key_released)) {
        last_key = current_key;
        last_scan_time = current_time;
        key_released = 0;
        return current_key;
    }
    
    // If no key is pressed, mark as released
    if (current_key == 0) {
        key_released = 1;
    }
    
    return 0;
}

void set_servo_position(uint8_t angle) {
    // Convert angle (0-180) to PWM value (1000-2000 microseconds)
    uint16_t pulse_width = 1000 + (angle * 1000) / 180*12;
    OCR1B = pulse_width - 1;
}

void buzzer_tone(uint16_t frequency, uint16_t duration_ms) {
    if (frequency > 0) {
        // Calculate OCR2A value for desired frequency
        uint16_t ocr_value = (F_CPU / (2 * 64 * frequency)) - 1;
        if (ocr_value > 255) ocr_value = 255;
        
        OCR2A = ocr_value;
        TCCR2B = (1 << CS22); // Start timer with prescaler 64
        
        // Non-blocking delay
        uint32_t start_time = get_system_time_ms();
        while (get_system_time_ms() - start_time < duration_ms) {
            // Allow other operations to continue
        }
        
        buzzer_off();
    }
}

void buzzer_off(void) {
    TCCR2B = 0; // Stop timer
    PORTB &= ~(1 << PB3); // Ensure pin is low
}

// I2C LCD functions
void i2c_start(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

void i2c_stop(void) {
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

void i2c_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

// Send a nibble to the LCD
void lcd_send_nibble(uint8_t nibble, uint8_t mode) {
    // High nibble first (4-bit mode)
    uint8_t data = (nibble & 0xF0) | mode | LCD_BACKLIGHT;
    
    i2c_start();
    i2c_write(LCD_I2C_ADDR << 1);
    i2c_write(data | LCD_ENABLE); // Pulse enable
    _delay_us(1);
    i2c_write(data & ~LCD_ENABLE);
    i2c_stop();
    _delay_us(100);
}

// Send a command to the LCD
void lcd_send_cmd(uint8_t cmd) {
    lcd_send_nibble(cmd, LCD_CMD);
    lcd_send_nibble(cmd << 4, LCD_CMD);
    if (cmd == 0x01 || cmd == 0x02) {
        _delay_ms(2); // Longer delay for clear and home commands
    }
}

// Send data to the LCD
void lcd_send_data(uint8_t data) {
    lcd_send_nibble(data, LCD_DATA);
    lcd_send_nibble(data << 4, LCD_DATA);
    _delay_us(100);
}

void lcd_init(void) {
    _delay_ms(50); // Wait for LCD to power up
    
    // Initialization sequence for 4-bit mode
    lcd_send_nibble(0x30, LCD_CMD);
    _delay_ms(5);
    lcd_send_nibble(0x30, LCD_CMD);
    _delay_us(100);
    lcd_send_nibble(0x30, LCD_CMD);
    _delay_us(100);
    lcd_send_nibble(0x20, LCD_CMD); // Set 4-bit mode
    _delay_us(100);
    
    // Function set: 4-bit, 2 lines, 5x8 font
    lcd_send_cmd(0x28);
    _delay_us(50);
    
    // Display control: Display on, cursor off, blink off
    lcd_send_cmd(0x0C);
    _delay_us(50);
    
    // Clear display
    lcd_send_cmd(0x01);
    _delay_ms(2);
    
    // Entry mode set: Increment, no shift
    lcd_send_cmd(0x06);
    _delay_us(50);
}

void lcd_clear(void) {
    lcd_send_cmd(0x01);
    _delay_ms(2);
}

void lcd_set_cursor(uint8_t col, uint8_t row) {
    uint8_t address;
    if (row == 0) {
        address = 0x80 + col;
    } else {
        address = 0xC0 + col;
    }
    lcd_send_cmd(address);
}

void lcd_print(const char* str) {
    while (*str) {
        lcd_send_data(*str++);
    }
}

// Additional useful LCD functions
void lcd_print_at(uint8_t col, uint8_t row, const char* str) {
    lcd_set_cursor(col, row);
    lcd_print(str);
}

void lcd_backlight(uint8_t on) {
    i2c_start();
    i2c_write(LCD_I2C_ADDR << 1);
    i2c_write(on ? LCD_BACKLIGHT : 0);
    i2c_stop();
}

void display_password_prompt(void) {
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("Enter Password:");
    lcd_set_cursor(0, 1);
    reset_password();
    uart_send_string("Password prompt displayed\r\n");
}

void reset_password(void) {
    memset(entered_password, 0, sizeof(entered_password));
    password_index = 0;
}

void process_key(char key) {
    switch (key) {
        case '#':
            uart_send_string("Password entered: ");
            uart_send_string(entered_password);
            uart_send_string("\r\n");
            verify_password();
            break;
            
        case '*':
            if (alarm_active) {
                deactivate_alarm();
            } else {
                uart_send_string("Password cleared\r\n");
                display_password_prompt();
            }
            break;
            
        case 'A':
        case 'C':
        case 'D':
            uart_send_string("Special key pressed\r\n");
            break;
            
        case 'B':
            if (system_active) {
                uart_send_string("Lock key pressed - Locking system\r\n");
                lock_system();
            }
            break;
            
        default:
            if (key >= '0' && key <= '9' && password_index < MAX_PASSWORD_LENGTH) {
                entered_password[password_index] = key;
                password_index++;
                entered_password[password_index] = '\0';
                
                lcd_set_cursor(password_index - 1, 1);
                lcd_print("*");
                
                char msg[50];
                sprintf(msg, "Digit added. Length: %d\r\n", password_index);
                uart_send_string(msg);
            }
            break;
    }
}

void verify_password(void) {
    lcd_clear();
    lcd_set_cursor(0, 0);
    
    uart_send_string("=== PASSWORD VERIFICATION ===\r\n");
    
    if (strcmp(entered_password, correct_password) == 0) {
        lcd_print("Access Granted!");
        uart_send_string("Password CORRECT - Access granted\r\n");
        
        PORTC |= (1 << PC1);  // Green LED on
        PORTC &= ~(1 << PC0); // Red LED off
        
        play_success_melody();
        
        set_servo_position(90); // Open door
        system_active = 1;
        last_motion_time = get_system_time_ms();
        
        lcd_set_cursor(0, 1);
        lcd_print("Door Opened");
        _delay_ms(2000);
        
        lcd_clear();
        lcd_set_cursor(0, 0);
        lcd_print("System Active");
        lcd_set_cursor(0, 1);
        lcd_print("Press B to lock");
        
        uart_send_string("Door opened - System active\r\n");
        
    } else {
        lcd_print("Access Denied!");
        uart_send_string("Password INCORRECT - Access denied\r\n");
        
        PORTC |= (1 << PC0);  // Red LED on
        PORTC &= ~(1 << PC1); // Green LED off
        
        play_error_melody();
        
        _delay_ms(2000);
        display_password_prompt();
        PORTC &= ~(1 << PC0); // Red LED off
    }
}

void lock_system(void) {
    uart_send_string("=== MANUAL LOCK ACTIVATED ===\r\n");
    
    set_servo_position(0); // Close door
    system_active = 0;
    PORTC &= ~(1 << PC1); // Green LED off
    
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("System Locked");
    lcd_set_cursor(0, 1);
    lcd_print("Manual Lock");
    
    // Lock confirmation sound
    buzzer_tone(800, 200);
    _delay_ms(250);
    buzzer_tone(600, 200);
    _delay_ms(250);
    
    uart_send_string("Door closed - System manually locked\r\n");
    _delay_ms(2000);
    display_password_prompt();
}

void play_success_melody(void) {
    uint16_t frequencies[] = {262, 330, 392, 523};
    uint16_t durations[] = {200, 200, 200, 400};
    
    for (int i = 0; i < 4; i++) {
        buzzer_tone(frequencies[i], durations[i]);
        _delay_ms(50);
    }
}

void play_error_melody(void) {
    uint16_t frequencies[] = {523, 392, 330, 262};
    
    for (int i = 0; i < 4; i++) {
        buzzer_tone(frequencies[i], 150);
        _delay_ms(50);
    }
    
    buzzer_tone(200, 800);
}

void check_pir_sensor(void) {
    uint8_t motion = (PINB & (1 << PB4)) ? 1 : 0;
    uint32_t current_time = get_system_time_ms();
    
    if (motion) {
        last_motion_time = current_time;
        if (!alarm_active) {
            uart_send_string("Motion detected - System active\r\n");
        }
    } else {
        if (current_time - last_motion_time > TIMEOUT_MOTION_MS) {
            set_servo_position(0); // Close door
            system_active = 0;
            PORTC &= ~(1 << PC1); // Green LED off
            
            lcd_clear();
            lcd_set_cursor(0, 0);
            lcd_print("System Closed");
            lcd_set_cursor(0, 1);
            lcd_print("Motion Timeout");
            
            uart_send_string("TIMEOUT: No motion for 10 seconds\r\n");
            uart_send_string("Door closed - System reset\r\n");
            
            _delay_ms(2000);
            display_password_prompt();
        }
    }
}

void activate_alarm(void) {
    alarm_active = 1;
    
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("!! ALARM !!");
    lcd_set_cursor(0, 1);
    lcd_print("Press * to stop");
    
    PORTC |= (1 << PC0); // Red LED on
    
    // Continuous alarm tone
    OCR2A = 124; // ~1000Hz
    TCCR2B = (1 << CS22); // Start timer
    
    uart_send_string("ALARM ACTIVATED!\r\n");
}

void deactivate_alarm(void) {
    if (alarm_active) {
        alarm_active = 0;
        buzzer_off();
        PORTC &= ~(1 << PC0); // Red LED off
        
        uart_send_string("Alarm deactivated\r\n");
        display_password_prompt();
    }
}

void update_display(void) {
    static uint32_t last_blink = 0;
    static uint8_t red_state = 0;
    uint32_t current_time = get_system_time_ms();
    
    // Blink red LED if alarm is active
    if (alarm_active && (current_time - last_blink > 500)) {
        last_blink = current_time;
        red_state = !red_state;
        if (red_state) {
            PORTC |= (1 << PC0);
        } else {
            PORTC &= ~(1 << PC0);
        }
    }
    
    // Update display with remaining time if system is active
    if (system_active && !alarm_active) {
        static uint32_t last_display_update = 0;
        if (current_time - last_display_update > 5000) {
            last_display_update = current_time;
            
            uint32_t time_remaining = TIMEOUT_MOTION_MS - (current_time - last_motion_time);
            if (time_remaining > TIMEOUT_MOTION_MS) time_remaining = 0;
            
            lcd_set_cursor(0, 1);
            char time_str[16];
            sprintf(time_str, "Time: %lus B:Lock", time_remaining / 1000);
            lcd_print(time_str);
        }
    }
}

uint32_t get_system_time_ms(void) {
    uint32_t time;
    cli(); // Disable interrupts for atomic read
    time = system_time_ms;
    sei(); // Re-enable interrupts
    return time;
}
