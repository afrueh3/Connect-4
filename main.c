/**
  ******************************************************************************
  * @file    main.c
  * @author  Aaron Frueh, Pat Cole, Michael Cupka
  * @version V1.0
  * @date    02-December-2018
  * @brief   Default main function.
  ******************************************************************************
*/

#include "stm32f0xx.h"
#include "stm32f0_discovery.h"

void init_i2c(void);
void init_adc(void);
int8_t send_data(uint8_t devaddr, void *pdata, uint8_t size);
void column_select(void); //column selected by voltage in
int i2c_checknack(void);
void i2c_clearnack(void);
int i2c_checkstop(void);
void i2c_clearstop(void);
void i2c_start(uint32_t devaddr, uint8_t size);
void i2c_stop(void);
void i2c_waitidle(void);
void init_matrix(uint8_t devaddr);
int drop_piece(int col_count);
int check_status(int location);
int check_red(int location);
int check_green(int location);
int check_down(int location, char* dataset);
int check_horiz(int location, char* dataset);
int check_diag1(int location, char* dataset);
int check_diag2(int location, char* dataset);
int check_right(int location, char* dataset, int offset);
int check_left(int location, char* dataset, int offset);
int check_up_right(int location, char* dataset, int offset);
int check_down_right(int location, char* dataset, int offset);
int check_up_left(int location, char* dataset, int offset);
int check_down_left(int location, char* dataset, int offset);

char gamedata[] = {0, //first character represents location in memory
            0x00, 0x00, //line 1 green, red
            0x00, 0x00, //line 2
            0x00, 0x00, //line 3
            0x00, 0x00, //line 4
            0x00, 0x00, //line 5
            0x00, 0x00, //line 6
            0x00, 0x00, //line 7
            0x00, 0x00 //line 8
    };

char greendata[] = {
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0
};

char reddata[] = {
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0
};

char blank[] = {0, //first character represents location in memory
            0x00, 0x00, //line 1 green, red
            0x00, 0x00, //line 2
            0x00, 0x00, //line 3
            0x00, 0x00, //line 4
            0x00, 0x00, //line 5
            0x00, 0x00, //line 6
            0x00, 0x00, //line 7
            0x00, 0x00 //line 8
    };

char greenwin1[] = {0, //first character represents location in memory
            0xff, 0x00, //line 1 green, red
            0x00, 0x00, //line 2
            0xff, 0x00, //line 3
            0x00, 0x00, //line 4
            0xff, 0x00, //line 5
            0x00, 0x00, //line 6
            0xff, 0x00, //line 7
            0x00, 0x00 //line 8
    };

char greenwin2[] = {0, //first character represents location in memory
            0x00, 0x00, //line 1 green, red
            0xff, 0x00, //line 2
            0x00, 0x00, //line 3
            0xff, 0x00, //line 4
            0x00, 0x00, //line 5
            0xff, 0x00, //line 6
            0x00, 0x00, //line 7
            0xff, 0x00 //line 8
    };

char redwin1[] = {0, //first character represents location in memory
            0x00, 0xff, //line 1 green, red
            0x00, 0x00, //line 2
            0x00, 0xff, //line 3
            0x00, 0x00, //line 4
            0x00, 0xff, //line 5
            0x00, 0x00, //line 6
            0x00, 0xff, //line 7
            0x00, 0x00 //line 8
    };

char redwin2[] = {0, //first character represents location in memory
            0x00, 0x00, //line 1 green, red
            0x00, 0xff, //line 2
            0x00, 0x00, //line 3
            0x00, 0xff, //line 4
            0x00, 0x00, //line 5
            0x00, 0xff, //line 6
            0x00, 0x00, //line 7
            0x00, 0xff //line 8
    };

char tie1[] = {0, //first character represents location in memory
            0xff, 0xff, //line 1 green, red
            0x00, 0x00, //line 2
            0xff, 0xff, //line 3
            0x00, 0x00, //line 4
            0xff, 0xff, //line 5
            0x00, 0x00, //line 6
            0xff, 0xff, //line 7
            0x00, 0x00 //line 8
    };

char tie2[] = {0, //first character represents location in memory
            0x00, 0x00, //line 1 green, red
            0xff, 0xff, //line 2
            0x00, 0x00, //line 3
            0xff, 0xff, //line 4
            0x00, 0x00, //line 5
            0xff, 0xff, //line 6
            0x00, 0x00, //line 7
            0xff, 0xff //line 8
    };

int turn = 0; //0 for green, 1 for red
int game_status = 0; //0 for continuing, 1 for green wins, 2 for red wins, 3 for tie

static void nano_wait(int t) {
    asm("       mov r0,%0\n"
        "repeat:\n"
        "       sub r0,#83\n"
        "       bgt repeat\n"
        : : "r"(t) : "r0", "cc");
}

static void cmd(char b) {
    while((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
    SPI2->DR = b;
}

static void data(char b) {
    while((SPI2->SR & SPI_SR_TXE) != SPI_SR_TXE);
    SPI2->DR = 0x200 | b;
}

void init_i2c(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    I2C1->CR1 &= ~I2C_CR1_PE;

    GPIOB->MODER &= ~(0xf<<12);
    GPIOB->MODER |= 0xa << 12;
    GPIOB->AFR[0] &= ~(GPIO_AFRL_AFRL6 | GPIO_AFRL_AFRL7);
    GPIOB->AFR[0] |= 1<<24; //sets alternate function for pins PB6 and PB7
    GPIOB->AFR[0] |= 1<<28;

    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    I2C1->TIMINGR |= 0x10420F13; //100 kHz standard timing values

    I2C1->CR1 &= ~I2C_CR1_NOSTRETCH;
    I2C1->CR1 &= ~I2C_CR1_ANFOFF;
    I2C1->CR1 &= ~(0x7f<<1);

    I2C1->OAR1 &= ~I2C_OAR1_OA1;
    I2C1->OAR1 |= 1<<1;
    I2C1->OAR1 |= I2C_OAR1_OA1EN; //sets own address mode

    I2C1->CR2 |= I2C_CR2_NACK; //enable nack
    I2C1->CR2 |= I2C_CR2_AUTOEND; //enable autoend
    I2C1->CR2 &= ~I2C_CR2_ADD10; //7 bit address mode

    I2C1->CR1 |= I2C_CR1_PE; //Enables i2c1
}

void init_adc(void){
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER |= 0x3;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->CR2 |= RCC_CR2_HSI14ON;
    while(!(RCC->CR2 & RCC_CR2_HSI14RDY));
    ADC1->CR |= ADC_CR_ADEN;
    while(!(ADC1->ISR & ADC_ISR_ADRDY));
    while((ADC1->CR & ADC_CR_ADSTART));
}

int8_t send_data(uint8_t devaddr, void *pdata, uint8_t size){
    int i;
    if (size <= 0 || pdata == 0) return -1;
    uint8_t *udata = (uint8_t*)pdata;
    i2c_waitidle();
    i2c_start(devaddr, size);

    for(i=0; i<size; i++) {
        // TXIS bit is set by hardware when the TXDR register is empty and the
        // data to be transmitted must be written in the TXDR register.  It is
        // cleared when the next data to be sent is written in the TXDR reg.
        // The TXIS flag is not set when a NACK is received.
        int count = 0;
        while( (I2C1->ISR & I2C_ISR_TXIS) == 0) {
            count += 1;
            if (count > 1000000)
                return -1;
            if (i2c_checknack()) {
                i2c_clearnack();
                i2c_stop();
                return -1;
            }
        }

        // TXIS is cleared by writing to the TXDR register.
        I2C1->TXDR = udata[i] & I2C_TXDR_TXDATA;
    }

    // Wait until TC flag is set or the NACK flag is set.
    while((I2C1->ISR & I2C_ISR_TC) == 0 && (I2C1->ISR & I2C_ISR_NACKF) == 0);

    if ( (I2C1->ISR & I2C_ISR_NACKF) != 0)
        return -1;
    i2c_stop();
    return 0;
}

int i2c_checknack(void) {
    if (I2C1->ISR & I2C_ISR_NACKF)
        return 1;
    return 0;
}

//===========================================================================
// Clear NACK condition.
void i2c_clearnack(void) {
    if (I2C1->ISR & I2C_ISR_NACKF)
        I2C1->ICR |= I2C_ICR_NACKCF;
}

//===========================================================================
// Clear STOP condition.
int i2c_checkstop(void) {
    if (I2C1->ISR & I2C_ISR_STOPF)
        return 1;
    return 0;
}

//===========================================================================
// Clear STOP condition.
void i2c_clearstop(void) {
    if (I2C1->ISR & I2C_ISR_STOPF)
        I2C1->ICR |= I2C_ICR_STOPCF;
}

void i2c_start(uint32_t devaddr, uint8_t size) {
    uint32_t tmpreg = I2C1->CR2;
    tmpreg &= ~(I2C_CR2_SADD | I2C_CR2_NBYTES |
                I2C_CR2_RELOAD | I2C_CR2_AUTOEND |
                I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP);

    tmpreg &= I2C_CR2_RD_WRN;  // Write to slave
    tmpreg |= ((devaddr<<1) & I2C_CR2_SADD) | ((size << 16) & I2C_CR2_NBYTES);
    tmpreg |= I2C_CR2_START;
    I2C1->CR2 = tmpreg;
}

//===========================================================================
// Generate a stop bit.
void i2c_stop(void) {
    if (I2C1->ISR & I2C_ISR_STOPF)
        return;
    // Master: Generate STOP bit after current byte has been transferred.
    I2C1->CR2 |= I2C_CR2_STOP;
    // Wait until STOPF flag is reset
    while( (I2C1->ISR & I2C_ISR_STOPF) == 0);
    I2C1->ICR |= I2C_ICR_STOPCF; // Write  to clear STOPF flag
}

//===========================================================================
// Check wait for the bus to be idle.
void i2c_waitidle(void) {
    while ( (I2C1->ISR ^ I2C_ISR_BUSY) == I2C_ISR_BUSY);  // while busy, wait.
}

static void init_lcd(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER &= ~((3<<(2*12)) | (3<<(2*13)) | (3<<(2*15)) );
    GPIOB->MODER |=   (2<<(2*12)) | (2<<(2*13)) | (2<<(2*15));
    GPIOB->AFR[1] &= ~( (0xf<<(4*(12-8))) | (0xf<<(4*(13-8))) | (0xf<<(4*(15-8))) );

    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    RCC->APB1RSTR |= RCC_APB1RSTR_SPI2RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_SPI2RST;

    SPI2->CR1 &= ~SPI_CR1_BR;
    SPI2->CR1 |= SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | SPI_CR1_MSTR;
    SPI2->CR1 |= SPI_CR1_BR;
    SPI2->CR2 = SPI_CR2_SSOE | SPI_CR2_NSSP | SPI_CR2_DS_3 | SPI_CR2_DS_0;
    SPI2->CR1 |= SPI_CR1_SPE;

    nano_wait(100000000); // Give it 100ms to initialize
    cmd(0x38);  // 0011 NF00 N=1, F=0: two lines
    cmd(0x0c);  // 0000 1DCB: display on, no cursor, no blink
    cmd(0x01);  // clear entire display
    nano_wait(6200000); // clear takes 6.2ms to complete
    cmd(0x02);  // put the cursor in the home position
    cmd(0x06);  // 0000 01IS: set display to increment
}

//===========================================================================
// Display a string on line 1 by writing to SPI directly.
static void display1(const char *s) {
    cmd(0x02); // put the cursor on the beginning of the first line.
    int x;
    for(x=0; x<16; x+=1)
        if (s[x])
            data(s[x]);
        else
            break;
    for(   ; x<16; x+=1)
        data(' ');
}

//===========================================================================
// Display a string on line 2 by writing to SPI directly.
static void display2(const char *s) {
    cmd(0xc0); // put the cursor on the beginning of the second line.
    int x;
    for(x=0; x<16; x+=1)
        if (s[x] != '\0')
            data(s[x]);
        else
            break;
    for(   ; x<16; x+=1)
        data(' ');
}

void init_matrix(uint8_t devaddr){
    char setup_data[] = {0x21, 0x00};
    send_data(devaddr, setup_data, sizeof(setup_data));

    char on_data[] = {0x81, 0x00};
    send_data(devaddr, on_data, sizeof(on_data));

    char brightness_data[] = {0xE5, 0x00};
    send_data(devaddr, brightness_data, sizeof(brightness_data));
}

void column_select(void){
    int current_turn = turn;
    while(turn == current_turn){ //update while to be until button is pushed
        while(!(ADC1->ISR & ADC_ISR_ADRDY));
        ADC1->CR |= ADC_CR_ADSTART;
        while(!(ADC1->ISR & ADC_ISR_EOC));

        if(turn == 0){ //change green row 1
            gamedata[1] = 1 << ADC1->DR / 512;
        }
        else{ //change red row 1
            gamedata[2] = 1 << ADC1->DR / 512;
        }
        send_data(0x70, gamedata, sizeof(gamedata));
    }
}

void EXTI4_15_IRQHandler(void){
    if(turn == 0){
        turn = 1;
    }
    else{
        turn = 0;
    }
    EXTI->PR |= EXTI_PR_PR8;
    nano_wait(50000000);
}

void init_button(void){
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER &= ~(0xf<<16);
    GPIOA->PUPDR |= 2<<16;

    EXTI->RTSR |= EXTI_RTSR_TR8;
    EXTI->IMR |= EXTI_IMR_MR8;

    NVIC->ISER[0] = 1 << EXTI4_15_IRQn;
}

int drop_piece(int col_count){
    EXTI->IMR &= ~(1<<8);

    int row;
    if(turn == 1){ //red's turn now, drop green piece
        int column = gamedata[1];
        if(((gamedata[5] | gamedata[6]) & column) != 0){
            turn = 0;
        }
        for(int i = 5; i>=0; i--){
            if(((gamedata[15-(2*i)] | gamedata[16-(2*i)]) & column) == 0){
                gamedata[15-(2*i)] += column;
                greendata[(5-i)*8 + col_count] = 1;

                if(i<5){
                    gamedata[15-(2*(i+1))] -= column;
                    greendata[(4-i)*8 + col_count] = 0;
                }
                send_data(0x70, gamedata, sizeof(gamedata));
                nano_wait(200000000);
                row = 5 - i;
            }
        }
    }
    else{ //green's turn now, drop red piece
        int column = gamedata[2];
        if(((gamedata[5] | gamedata[6]) & column) != 0){
            turn = 1;
        }
        for(int i = 5; i>=0; i--){
            if(((gamedata[15-(2*i)] | gamedata[16-(2*i)]) & column) == 0){
                gamedata[16-(2*i)] += column;
                reddata[(5-i)*8 + col_count] = 1;

                if(i<5){
                    gamedata[16-(2*(i+1))] -= column;
                    reddata[(4-i)*8 + col_count] = 0;
                }
                send_data(0x70, gamedata, sizeof(gamedata));
                nano_wait(200000000);
                row = 5 - i;
            }
        }
    }
    gamedata[1] = 0;
    gamedata[2] = 0;
    send_data(0x70, gamedata, sizeof(gamedata));
    EXTI->IMR |= 1<<8;
    return row;
}

int check_status(int location){
    int return_type;
    if (turn == 0){ //red just dropped
        return_type = check_red(location);
    }
    else if (turn == 1){ //green just dropped
        return_type = check_green(location);
    }
    if(return_type == 0){
        for(int i = 0; i < 8; i++){
            if(!(greendata[i] | reddata[i])){
                return 0;
            }
        }
        return_type = 3;
    }
    return return_type;
    //else{
    //    return 0;
    //}
}

int check_red(int location){
    int down = check_down(location, reddata);
    int horiz = check_horiz(location, reddata);
    int diag1 = check_diag1(location, reddata);
    int diag2 = check_diag2(location, reddata);
    if(down || horiz || diag1 || diag2){
        return 2;
    }
    else{
        return 0;
    }
}

int check_green(int location){
    int down = check_down(location, greendata);
    int horiz = check_horiz(location, greendata);
    int diag1 = check_diag1(location, greendata);
    int diag2 = check_diag2(location, greendata);
    if(down || horiz || diag1 || diag2){
        return 1;
    }
    else{
        return 0;
    }
}

int check_down(int location, char* dataset){
    if(location >= 24){
        return 0;
    }
    if (dataset[location] && dataset[location+8] && dataset[location+16] && dataset[location+24]){
        return 1;
    }
    else{
        return 0;
    }
}

int check_horiz(int location, char* dataset){
    int right = check_right(location, dataset, 0);
    int left = check_left(location, dataset, 0);
    if ((right + left + 1) >= 4){
        return 1;
    }
    else{
        return 0;
    }
}

int check_diag1(int location, char* dataset){
    int right = check_down_right(location, dataset, 0);
    int left = check_up_left(location, dataset, 0);
    if ((right + left + 1) >= 4){
        return 1;
    }
    else{
        return 0;
    }
}

int check_diag2(int location, char* dataset){
    int right = check_up_right(location, dataset, 0);
    int left = check_down_left(location, dataset, 0);
    if ((right + left + 1) >= 4){
        return 1;
    }
    else{
        return 0;
    }
}

int check_right(int location, char* dataset, int offset){
    if(location % 8 == 7){ //block on right edge
        return offset;
    }
    if((dataset[location] && dataset[location+1]) != 0){
        offset = check_right(location + 1, dataset, offset+1);
    }
    else{
        return offset;
    }
}

int check_left(int location, char* dataset, int offset){
    if(location % 8 == 0){ //block on left edge
        return offset;
    }
    if((dataset[location] && dataset[location-1]) != 0){
        offset = check_left(location - 1, dataset, offset+1);
    }
    else{
        return offset;
    }
}

int check_up_left(int location, char* dataset, int offset){
    if(location % 8 == 0){ //block on left edge
        return offset;
    }
    if(location <= 7){
        return offset;
    }
    if((dataset[location] && dataset[location-9]) != 0){
        offset = check_up_left(location - 9, dataset, offset + 1);
    }
    else{
        return offset;
    }
}

int check_down_left(int location, char* dataset, int offset){
    if(location % 8 == 0){ //block on left edge
        return offset;
    }
    if(location >= 40){
        return offset;
    }
    if((dataset[location] && dataset[location+7]) != 0){
        offset = check_down_left(location+7, dataset, offset+1);
    }
    else{
        return offset;
    }
}

int check_up_right(int location, char* dataset, int offset){
    if(location % 8 == 7){ //block on right edge
        return offset;
    }
    if(location <= 7){
        return offset;
    }
    if((dataset[location] && dataset[location-7]) != 0){
        offset = check_up_right(location-7, dataset, offset+1);
    }
    else{
        return offset;
    }
}

int check_down_right(int location, char* dataset, int offset){
    if(location % 8 == 7){ //block on right edge
        return offset;
    }
    if(location >= 40){
        return offset;
    }
    if((dataset[location] && dataset[location+9]) != 0){
        offset = check_down_right(location+9, dataset, offset+1);
    }
    else{
        return offset;
    }
}

int main(void){
    init_i2c();
    init_adc();
    init_lcd();
    init_matrix(0x70);
    init_button();

    int column;
    int location;

    while(game_status == 0){ //update while to be until somebody wins
        int col_counter = 0;
        ADC1->CHSELR = 0;
        ADC1->CHSELR |= 1 << 8;
        if (turn == 0){
            display1("Player 1's Turn");
            display2("    (Green)");
        }
        else{
            display1("Player 2's Turn");
            display2("     (Red)");;
        }
        column_select();

        if (turn == 1){ //green just dropped
            column = gamedata[1];
            for (int i = 1; i <= 128; i*=2){
                if(i == column){
                    break;
                }
                col_counter++;
            }
        }
        else{
            column = gamedata[2];
            for (int i = 1; i <= 128; i*=2){
                if(i == column){
                    break;
                }
                col_counter++;
            }
        }
        int row = drop_piece(col_counter);
        location = row * 8 + col_counter;
        game_status = check_status(location);
    }
    if(game_status == 1){ //green wins
        display1("Congrats");
        display2("Player 1!");
        for(int i = 0; i < 10; i++){
            send_data(0x70, greenwin1, sizeof(greenwin1));
            nano_wait(200000000);
            send_data(0x70, greenwin2, sizeof(greenwin2));
            nano_wait(200000000);
        }
        send_data(0x70, gamedata, sizeof(gamedata));
    }
    else if(game_status == 2){ //red wins
        display1("Congrats");
        display2("Player 2!");
        for(int i = 0; i < 10; i++){
            send_data(0x70, redwin1, sizeof(redwin1));
            nano_wait(200000000);
            send_data(0x70, redwin2, sizeof(redwin2));
            nano_wait(200000000);
        }
        send_data(0x70, gamedata, sizeof(gamedata));
    }
    else if(game_status == 3){ //tie
        display1("It's a tie!");
        display2("Press reset");
        nano_wait(200000000);
        for(int i = 0; i < 10; i++){
            send_data(0x70, tie1, sizeof(tie1));
            nano_wait(200000000);
            send_data(0x70, tie2, sizeof(tie2));
            nano_wait(200000000);
        }
        send_data(0x70, gamedata, sizeof(gamedata));
    }
}
