#include <stdio.h>
#include <conio.h>
#include <math.h>

class pwm_calc
{
private:
    int f_pwm;
    int f_tmr;
    int f_tmr_Mhz;
    int timer_type;

    int prscl;
    int period;
public:
    period_calc(void);
    red(void);
    green(void);
    blue(void);
};

pwm_calc::period_calc(void)
{
    printf("\033[1  ;32m");
    printf("- PWM Freq (Hz): ");
    printf("\033[0m");
    scanf("%d", &f_pwm);
    
    printf("\033[1;32m");
    printf("- Timer Freq (MHz): ");
    printf("\033[0m");
    scanf("%d", &f_tmr_Mhz);
    f_tmr = f_tmr_Mhz * 1000000;

    printf("\033[1;32m");
    printf("- Choose Timer type \n");
    printf("\033[1;34m");
    printf("\t[8]  8-bit Timer \n\t[16] 16-bit Timer \n");
    printf("\033[1;32m");
    printf("  Timer type: ");
    printf("\033[0m");
    scanf("%d", &timer_type);

    printf("\033[1;32m");
    printf("- Prescaler: ");
    printf("\033[0m");
    scanf("%d", &prscl);
    period = (f_tmr / (f_pwm * (prscl + 1))) - 1;
    do
    {
        printf("\033[1;31m");
        printf("\tPrescaler value is too small!\n");

        printf("\033[1;32m");
        printf("  Prescaler: ");
        printf("\033[0m");
        scanf("%d", &prscl);
        period = (f_tmr / (f_pwm * (prscl + 1))) - 1;
    } while (period >= pow(2, timer_type));
    
    printf("\033[1;35m");
    printf("### Period result: %d", period);
    printf("\033[0m");
    return 0;
}

int main(void) 
{
    pwm_calc calc_er;
    calc_er.period_calc();
    
    getch();
    return 0;
}